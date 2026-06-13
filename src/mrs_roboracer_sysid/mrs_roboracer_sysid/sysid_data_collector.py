"""
Main sysid node: state machine, path execution, bag recording.

Per the sysid spec: this node generates open-loop (v, delta) command
sequences, executes them via `/safe_drive` (never `/drive` directly --
that's the mux/safety_node's output), and records everything to a
rosbag2 for offline noise-model fitting. It logs raw synced data and
does NOT compute residuals or fit anything.
"""

import json
import signal
import subprocess
from enum import IntEnum

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from mrs_roboracer_sysid_msgs.msg import SysidMeta

from mrs_roboracer_sysid.coverage import CoverageTracker
from mrs_roboracer_sysid.motion_model import quaternion_to_yaw
from mrs_roboracer_sysid.pose_watchdog import PoseStalenessWatchdog, WatchdogStatus
from mrs_roboracer_sysid.spline_paths import generate_path, save_generated_paths
from mrs_roboracer_sysid.velocity_estimators import PoseDerivativeEstimator


class State(IntEnum):
    """MUST mirror mrs_roboracer_sysid_msgs/SysidMeta PHASE_* constants exactly."""

    IDLE = 0
    GENERATE = 1
    ARMED = 2
    EXECUTE = 3
    SETTLE = 4
    DONE = 5


# Dotted ROS parameter name -> default value. Section (before the first '.')
# becomes a top-level key of self.config; the rest becomes the leaf key.
_PARAM_DEFAULTS = {
    "model.L": 0.33,
    "model.ref_point": "rear_axle",
    "model.delta_max": 0.4189,
    "model.delta_dot_max": 3.2,

    "limits.v_min": 0.5,
    "limits.v_max": 3.0,
    "limits.a_max": 2.0,
    "limits.a_lat_max_safe": 4.0,
    "limits.a_lat_probe_max": 8.0,

    "paths.N": 30,
    "paths.spline_order": 5,
    "paths.waypoints_per_path": 6,
    "paths.rng_seed": 42,
    "paths.workspace_bounds": [-5.0, 5.0, -5.0, 5.0],
    "paths.max_generation_attempts": 50,
    "paths.persist_dir": "",
    "paths.check_self_intersection": False,

    "coverage.v_bins": [0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
    # Bin edges must stay within kappa_max = tan(model.delta_max)/model.L
    # (~1.35 for the defaults above), with margin for spline curvature to
    # locally exceed a path's nominal 1/radius. See config/sysid_params.yaml.
    "coverage.kappa_bins": [-1.2, -0.8, -0.3, 0.3, 0.8, 1.2],
    "coverage.target_revisits_per_bin": 3,
    "coverage.include_figure8": True,
    "coverage.figure8_period": 5,
    "coverage.figure8_radius": 1.5,
    "coverage.figure8_speed": 1.5,
    "coverage.prioritize_corner_first": True,
    "coverage.corner_probe_paths": 2,

    "rate.control_rate_hz": 50.0,

    "io.pose_topic": "/sysid/pose",
    "io.twist_topic": "/sysid/twist",
    "io.odom_topic": "/ego_racecar/odom",
    "io.drive_topic": "/safe_drive",
    "io.bag_path": "/home/mrslab3/sysid_bags/session",

    "velocity.velocity_source": "pose_derivative",
    "velocity.sg_window": 9,
    "velocity.sg_order": 2,
    "velocity.sg_dt_jitter_warn_ratio": 0.1,

    "sync.enable_sync_chirp": True,
    "sync.chirp_amplitude": 0.2,
    "sync.chirp_duration": 0.5,
    "sync.chirp_speed": 1.5,

    "safety.pose_timeout_ms": 200.0,
    "safety.path_timeout_s": 60.0,
    "safety.settle_v_eps": 0.05,
    "safety.settle_confirm_s": 0.5,
    "safety.settle_timeout_s": 10.0,
    "safety.watchdog_rate_hz": 20.0,
}


class SysidDataCollector(Node):

    def __init__(self):
        super().__init__('sysid_data_collector')

        self.config = self._build_config()

        self.rng_seed = int(self.config["paths"]["rng_seed"])
        self.rng = np.random.default_rng(self.rng_seed)
        self.get_logger().info(f"sysid_data_collector: rng_seed={self.rng_seed}")

        self.coverage_tracker = CoverageTracker(
            v_bins=self.config["coverage"]["v_bins"],
            kappa_bins=self.config["coverage"]["kappa_bins"],
            target_revisits=self.config["coverage"]["target_revisits_per_bin"],
            include_figure8=self.config["coverage"]["include_figure8"],
            figure8_period=self.config["coverage"]["figure8_period"],
            figure8_speed=self.config["coverage"]["figure8_speed"],
            prioritize_corner_first=self.config["coverage"]["prioritize_corner_first"],
            corner_probe_paths=self.config["coverage"]["corner_probe_paths"],
        )

        persist_dir = self.config["paths"]["persist_dir"]
        self.persist_dir = persist_dir if persist_dir else self.config["io"]["bag_path"] + "_paths"

        self.pose_watchdog = PoseStalenessWatchdog(self.config["safety"]["pose_timeout_ms"])

        # session/state-machine state
        self.state = State.IDLE
        self.generated_paths = []
        self.path_index = 0
        self.step_index = 0
        self.t_start = None
        self.bag_proc = None

        # latest raw inputs (stashed only; rosbag2 captures the raw streams)
        self.last_pose_time_s = None
        self.last_pose_xytheta = None
        self.last_twist_msg = None
        self.last_odom_msg = None
        self.pose_stale = True

        self._settle_confirm_start = None
        self._settle_enter_time = None
        self.settle_velocity_estimator = self._new_settle_estimator()

        # publishers: /safe_drive only -- never /drive (that's safety_node's output)
        self.safe_drive_pub = self.create_publisher(
            AckermannDriveStamped, self.config["io"]["drive_topic"], 10)
        self.meta_pub = self.create_publisher(SysidMeta, '/sysid/meta', 10)

        # subscribers (stash latest msg + receive time only; no online processing)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.config["io"]["pose_topic"], self._pose_callback, 10)
        if self.config["io"]["twist_topic"]:
            self.twist_sub = self.create_subscription(
                TwistStamped, self.config["io"]["twist_topic"], self._twist_callback, 10)
        if self.config["io"]["odom_topic"]:
            self.odom_sub = self.create_subscription(
                Odometry, self.config["io"]["odom_topic"], self._odom_callback, 10)

        # operator-in-the-loop arming services
        self.start_session_srv = self.create_service(
            Trigger, '/sysid/start_session', self._handle_start_session)
        self.arm_path_srv = self.create_service(
            Trigger, '/sysid/arm_path', self._handle_arm_path)

        # main control loop + independent pose-staleness watchdog
        control_rate_hz = self.config["rate"]["control_rate_hz"]
        self.control_dt = 1.0 / control_rate_hz
        self.control_timer = self.create_timer(self.control_dt, self._control_tick)

        watchdog_rate_hz = self.config["safety"]["watchdog_rate_hz"]
        self.watchdog_timer = self.create_timer(1.0 / watchdog_rate_hz, self._watchdog_tick)

        self.get_logger().info('sysid_data_collector initialized, state=IDLE')

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _build_config(self) -> dict:
        config = {}
        for dotted, default in _PARAM_DEFAULTS.items():
            self.declare_parameter(dotted, default)
            value = self.get_parameter(dotted).value
            section, key = dotted.split(".", 1)
            config.setdefault(section, {})[key] = value
        return config

    def _new_settle_estimator(self) -> PoseDerivativeEstimator:
        return PoseDerivativeEstimator(
            window=self.config["velocity"]["sg_window"],
            polyorder=self.config["velocity"]["sg_order"],
            jitter_warn_ratio=self.config["velocity"]["sg_dt_jitter_warn_ratio"],
            logger=self.get_logger(),
        )

    # ------------------------------------------------------------------
    # Subscriptions -- stash raw data only
    # ------------------------------------------------------------------

    def _pose_callback(self, msg: PoseStamped) -> None:
        now = self.get_clock().now()
        self.last_pose_time_s = now.nanoseconds / 1e9
        yaw = quaternion_to_yaw(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w)
        self.last_pose_xytheta = (msg.pose.position.x, msg.pose.position.y, yaw)

    def _twist_callback(self, msg: TwistStamped) -> None:
        self.last_twist_msg = msg

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom_msg = msg

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _watchdog_tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        status = self.pose_watchdog.check(now_s, self.last_pose_time_s)
        self.pose_stale = (status == WatchdogStatus.POSE_STALE)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _handle_start_session(self, request, response):
        if self.state != State.IDLE:
            response.success = False
            response.message = f"cannot start session from state {self.state.name}"
            return response

        n_paths = int(self.config["paths"]["N"])
        self.get_logger().info(
            f"Starting sysid session: rng_seed={self.rng_seed}, N={n_paths} paths")
        self.state = State.GENERATE

        if not self._generate_all_paths():
            response.success = False
            response.message = "path generation failed; see log; returning to IDLE"
            self.state = State.IDLE
            return response

        self._start_bag_recording()

        self.path_index = 0
        self.step_index = 0
        self.state = State.ARMED
        response.success = True
        response.message = (
            f"session started: {len(self.generated_paths)} paths generated, "
            f"recording to {self.config['io']['bag_path']}"
        )
        return response

    def _handle_arm_path(self, request, response):
        if self.state != State.ARMED:
            response.success = False
            response.message = f"cannot arm from state {self.state.name}"
            return response
        if self.pose_stale:
            response.success = False
            response.message = "pose stream stale; refusing to arm"
            return response

        self.step_index = 0
        self.t_start = self.get_clock().now()
        self.state = State.EXECUTE
        path = self.generated_paths[self.path_index]
        response.success = True
        response.message = f"armed path {path.path_id} ({len(path.commands)} steps)"
        return response

    # ------------------------------------------------------------------
    # Bag recording (node-managed `ros2 bag record` subprocess)
    # ------------------------------------------------------------------

    def _start_bag_recording(self) -> None:
        io_cfg = self.config["io"]
        bag_path = io_cfg["bag_path"]
        topics = [io_cfg["drive_topic"], "/drive", "/sysid/meta", io_cfg["pose_topic"]]
        if io_cfg["twist_topic"]:
            topics.append(io_cfg["twist_topic"])
        if io_cfg["odom_topic"]:
            topics.append(io_cfg["odom_topic"])

        cmd = ["ros2", "bag", "record", "-o", bag_path] + topics
        self.get_logger().info(f"Starting bag recording: {' '.join(cmd)}")
        self.bag_proc = subprocess.Popen(cmd)

    def _stop_bag_recording(self) -> None:
        if self.bag_proc is None:
            return
        self.get_logger().info("Stopping bag recording")
        self.bag_proc.send_signal(signal.SIGINT)
        try:
            self.bag_proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("bag record did not exit on SIGINT; sending SIGTERM")
            self.bag_proc.terminate()
            try:
                self.bag_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().error("bag record did not exit on SIGTERM; killing")
                self.bag_proc.kill()
                self.bag_proc.wait()
        self.bag_proc = None

    # ------------------------------------------------------------------
    # Path generation
    # ------------------------------------------------------------------

    def _generate_all_paths(self) -> bool:
        n_paths = int(self.config["paths"]["N"])
        self.generated_paths = []
        for path_id in range(n_paths):
            target = self.coverage_tracker.next_target()
            gp = generate_path(path_id, self.rng_seed, self.rng, self.config, target)
            if gp is None:
                max_attempts = self.config["paths"]["max_generation_attempts"]
                self.get_logger().error(
                    f"failed to generate path {path_id} "
                    f"(profile={target.profile_type}, kappa_center={target.kappa_center:.3f}) "
                    f"after {max_attempts} attempts")
                return False
            self.generated_paths.append(gp)

        save_generated_paths(self.generated_paths, self.persist_dir)
        self.get_logger().info(
            f"Generated {len(self.generated_paths)} paths, persisted to {self.persist_dir}")
        return True

    # ------------------------------------------------------------------
    # Control loop / state machine
    # ------------------------------------------------------------------

    def _control_tick(self) -> None:
        now = self.get_clock().now()
        if self.state == State.EXECUTE:
            self._execute_tick(now)
        elif self.state == State.SETTLE:
            self._settle_tick(now)
        else:
            # IDLE, GENERATE (transient, handled synchronously in the
            # service callback), ARMED, DONE: publish zero.
            self._publish_outputs(now)

    def _execute_tick(self, now) -> None:
        path = self.generated_paths[self.path_index]
        commands = path.commands

        if self.pose_stale:
            self._abort_to_idle("pose became stale during EXECUTE")
            return

        elapsed_s = (now - self.t_start).nanoseconds / 1e9
        if elapsed_s > self.config["safety"]["path_timeout_s"]:
            self._abort_to_idle("path execution timed out")
            return

        if self.step_index >= len(commands):
            self._publish_outputs(now)
            self.state = State.SETTLE
            self._settle_confirm_start = None
            self._settle_enter_time = now
            self.settle_velocity_estimator = self._new_settle_estimator()
            return

        cmd = commands[self.step_index]
        t_intended = self.t_start + Duration(seconds=self.step_index * cmd.dt)
        self._publish_outputs(now, cmd=cmd, path=path, t_intended=t_intended)
        self.coverage_tracker.record_step(cmd.v, cmd.kappa)
        self.step_index += 1

    def _settle_tick(self, now) -> None:
        self._publish_outputs(now)

        settle_v_eps = self.config["safety"]["settle_v_eps"]
        settle_confirm_s = self.config["safety"]["settle_confirm_s"]

        if self.last_pose_xytheta is not None:
            t = (self.last_pose_time_s if self.last_pose_time_s is not None
                 else now.nanoseconds / 1e9)
            x, y, theta = self.last_pose_xytheta
            result = self.settle_velocity_estimator.update(t, x, y, theta)
            if result is not None:
                v_est, _ = result
                if abs(v_est) < settle_v_eps:
                    if self._settle_confirm_start is None:
                        self._settle_confirm_start = now
                    held_s = (now - self._settle_confirm_start).nanoseconds / 1e9
                    if held_s >= settle_confirm_s:
                        self._advance_after_settle()
                        return
                else:
                    self._settle_confirm_start = None

        settle_elapsed_s = (now - self._settle_enter_time).nanoseconds / 1e9
        if settle_elapsed_s > self.config["safety"]["settle_timeout_s"]:
            self.get_logger().warn("SETTLE timed out without confirming v~0; returning to IDLE")
            self._stop_bag_recording()
            self.state = State.IDLE

    def _advance_after_settle(self) -> None:
        if self.path_index + 1 < len(self.generated_paths):
            self.path_index += 1
            self.step_index = 0
            self.state = State.ARMED
        else:
            self.state = State.DONE
            self._finalize_session()

    def _abort_to_idle(self, reason: str) -> None:
        self.get_logger().warn(f"Aborting to IDLE: {reason}")
        now = self.get_clock().now()
        self._publish_outputs(now)
        # Stop the bag so it finalizes cleanly. Per the DONE-state design,
        # resuming requires a fresh io.bag_path (or node restart) since
        # `ros2 bag record` refuses to write into an existing bag dir.
        self._stop_bag_recording()
        self.state = State.IDLE

    def _finalize_session(self) -> None:
        self._stop_bag_recording()
        report = self.coverage_tracker.report()
        report_path = self.config["io"]["bag_path"] + "_coverage_report.json"
        try:
            with open(report_path, "w") as f:
                json.dump(report, f, indent=2)
        except OSError as e:
            self.get_logger().error(f"failed to write coverage report to {report_path}: {e}")
        self.get_logger().info(
            f"Session DONE: {report['bins_met']}/{report['bins_total']} bins met "
            f"({report['pct_met']:.1f}%); coverage report at {report_path}")

    # ------------------------------------------------------------------
    # Output publishing
    # ------------------------------------------------------------------

    def _publish_outputs(self, now, cmd=None, path=None, t_intended=None) -> None:
        if t_intended is None:
            t_intended = now

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = t_intended.to_msg()
        drive_msg.drive.speed = cmd.v if cmd is not None else 0.0
        drive_msg.drive.steering_angle = cmd.delta if cmd is not None else 0.0
        self.safe_drive_pub.publish(drive_msg)

        meta = SysidMeta()
        meta.header.stamp = t_intended.to_msg()
        meta.phase = int(self.state)
        meta.rng_seed = self.rng_seed
        meta.t_cmd_intended = t_intended.to_msg()
        meta.t_cmd_published = now.to_msg()
        meta.loop_jitter_s = (now - t_intended).nanoseconds / 1e9
        meta.flag_stale_pose = self.pose_stale
        meta.step_index = self.step_index

        if cmd is not None and path is not None:
            meta.path_id = path.path_id
            meta.v_cmd = cmd.v
            meta.delta_cmd = cmd.delta
            meta.kappa_cmd = cmd.kappa
            v_idx, kappa_idx = self.coverage_tracker.bin_for(cmd.v, cmd.kappa)
            meta.coverage_bin_v = v_idx
            meta.coverage_bin_kappa = kappa_idx
            meta.coverage_bin_id = self.coverage_tracker.bin_id(v_idx, kappa_idx)
            meta.lat_accel_est = cmd.lat_accel
            meta.flag_sync_chirp = cmd.flag_sync_chirp
            meta.flag_high_lat_accel = cmd.flag_high_lat_accel
            meta.dt = cmd.dt
        else:
            meta.path_id = (self.generated_paths[self.path_index].path_id
                            if self.generated_paths else 0)
            meta.coverage_bin_v = -1
            meta.coverage_bin_kappa = -1
            meta.coverage_bin_id = -1
            meta.dt = self.control_dt

        self.meta_pub.publish(meta)


def main(args=None):
    rclpy.init(args=args)
    node = SysidDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_bag_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
