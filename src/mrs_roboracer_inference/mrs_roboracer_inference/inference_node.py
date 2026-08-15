import os
from collections import deque

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from mrs_roboracer_inference.policy_model import load_policy


def resample_scan(ranges, angle_min, angle_increment, range_max, n_beams, fov_rad, max_range):
    """Resample a raw LaserScan onto the policy's fixed body-frame beam layout."""
    # Resampled (via linear interpolation over angle, not naive index slicing) onto
    # n_beams targets evenly spaced across fov_rad centered on 0 rad (straight ahead)
    # -- robust to the incoming scan's beam count/FOV differing from the policy's,
    # e.g. real hardware vs. sim. NaN/inf/out-of-range readings are clamped to
    # range_max first (a sensor "no return" is a free/far reading, not a hazard),
    # matching reactive_driver_node's existing cleaning convention. Output is
    # range_m / max_range, clamped to [0, 1] per MODEL_INFO.md.
    ranges = np.asarray(ranges, dtype=np.float64)
    bad = ~np.isfinite(ranges)
    ranges = np.where(bad, range_max, ranges)

    n_in = len(ranges)
    src_angles = angle_min + np.arange(n_in) * angle_increment

    target_angles = -fov_rad / 2.0 + np.arange(n_beams) * (fov_rad / (n_beams - 1))
    resampled = np.interp(target_angles, src_angles, ranges, left=ranges[0], right=ranges[-1])

    normalized = np.clip(resampled / max_range, 0.0, 1.0)
    return normalized.astype(np.float32)


class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')

        default_checkpoint = os.path.join(
            get_package_share_directory('mrs_roboracer_inference'), 'model', 'policy.pt')

        # Topic parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/safe_drive')
        self.declare_parameter('checkpoint_path', default_checkpoint)

        # Observation layout parameters (must match the checkpoint's training config)
        self.declare_parameter('n_lidar_beams', 64)
        self.declare_parameter('lidar_stack_n', 4)
        self.declare_parameter('fov_deg', 270.0)
        self.declare_parameter('max_range', 30.0)
        # v_max is a training-time normalization constant, NOT a real speed cap
        self.declare_parameter('v_max', 10.0)

        # Action bounds. The policy's raw output is unclamped; the deployer must
        # enforce these itself, per MODEL_INFO.md.
        self.declare_parameter('max_steer_angle', 0.4189)  # rad, steering angle integrator clamp
        self.declare_parameter('max_steer_vel', 3.2)       # rad/s
        self.declare_parameter('max_accel', 9.51)          # m/s^2
        self.declare_parameter('min_speed_cmd', 0.0)
        self.declare_parameter('max_speed_cmd', 12.0)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('deterministic', True)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        checkpoint_path = self.get_parameter(
            'checkpoint_path').get_parameter_value().string_value

        self.n_lidar_beams = self.get_parameter(
            'n_lidar_beams').get_parameter_value().integer_value
        self.lidar_stack_n = self.get_parameter(
            'lidar_stack_n').get_parameter_value().integer_value
        self.fov_rad = np.deg2rad(
            self.get_parameter('fov_deg').get_parameter_value().double_value)
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value

        self.max_steer_angle = self.get_parameter(
            'max_steer_angle').get_parameter_value().double_value
        self.max_steer_vel = self.get_parameter(
            'max_steer_vel').get_parameter_value().double_value
        self.max_accel = self.get_parameter('max_accel').get_parameter_value().double_value
        self.min_speed_cmd = self.get_parameter(
            'min_speed_cmd').get_parameter_value().double_value
        self.max_speed_cmd = self.get_parameter(
            'max_speed_cmd').get_parameter_value().double_value

        control_rate_hz = self.get_parameter(
            'control_rate_hz').get_parameter_value().double_value
        self.control_dt = 1.0 / control_rate_hz
        self.deterministic = self.get_parameter('deterministic').get_parameter_value().bool_value

        # Policy
        self.device = 'cpu'
        self.policy = load_policy(checkpoint_path, device=self.device)
        self.gru_state = self.policy.initial_state(batch_size=1, device=self.device)

        # Rolling state
        self.beam_history = deque(maxlen=self.lidar_stack_n)
        self.latest_beams = None
        self.last_odom_v = None
        self.steering_angle_cmd = 0.0

        # Publisher and subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.lidar_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        self.control_timer = self.create_timer(self.control_dt, self.control_step)

        self.get_logger().info(
            f'InferenceNode initialized: checkpoint={checkpoint_path}, '
            f'scan={scan_topic}, odom={odom_topic}, drive={drive_topic}, '
            f'control_rate={control_rate_hz} Hz'
        )

    def lidar_callback(self, scan: LaserScan):
        """Resample the incoming scan onto the policy's beam layout and cache it."""
        # Actual inference runs on control_step's fixed-rate timer, not here.
        self.latest_beams = resample_scan(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max,
            self.n_lidar_beams, self.fov_rad, self.max_range
        )

    def odom_callback(self, msg: Odometry):
        """Track the latest forward speed."""
        # Used both for the policy's velocity input and for grounding the
        # integrated speed command.
        self.last_odom_v = msg.twist.twist.linear.x

    def control_step(self):
        """Run one 20 Hz policy inference step and publish a drive command."""
        if self.latest_beams is None:
            return  # no scan received yet

        if len(self.beam_history) == 0:
            # First step of an episode: prime all history slots with the first
            # scan rather than leaving stale/zero data (MODEL_INFO.md).
            for _ in range(self.lidar_stack_n):
                self.beam_history.append(self.latest_beams)
        else:
            self.beam_history.append(self.latest_beams)

        v = self.last_odom_v if self.last_odom_v is not None else 0.0

        obs = np.concatenate(
            list(self.beam_history) + [np.array([v / self.v_max], dtype=np.float32)])
        obs_tensor = torch.as_tensor(
            obs, dtype=torch.float32, device=self.device).unsqueeze(0)

        with torch.no_grad():
            dist, _value, self.gru_state = self.policy.forward_eval(obs_tensor, self.gru_state)
            action = dist.mean if self.deterministic else dist.sample()

        steer_vel_cmd = float(
            np.clip(action[0, 0].item(), -self.max_steer_vel, self.max_steer_vel))
        accel_cmd = float(np.clip(action[0, 1].item(), -self.max_accel, self.max_accel))

        # The policy outputs rates; this stack's AckermannDriveStamped is consumed
        # downstream as absolute steering_angle + speed, so integrate here.
        self.steering_angle_cmd = float(np.clip(
            self.steering_angle_cmd + steer_vel_cmd * self.control_dt,
            -self.max_steer_angle, self.max_steer_angle
        ))
        speed_cmd = float(np.clip(
            v + accel_cmd * self.control_dt,
            self.min_speed_cmd, self.max_speed_cmd
        ))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = self.steering_angle_cmd
        msg.drive.speed = speed_cmd
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
