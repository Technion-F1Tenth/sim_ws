import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# Minimum closing speed (m/s) considered for TTC; guards against
# division-by-zero / float noise for near-tangential beams.
_CLOSING_SPEED_EPS = 1e-3


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Topic parameters
        self.declare_parameter('safe_drive_topic', '/safe_drive')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')

        # Behavior toggle parameters
        self.declare_parameter('enable_clipping', True)
        self.declare_parameter('enable_ttc', False)

        # Tuning parameters
        self.declare_parameter('ttc_threshold', 1.0)          # seconds
        self.declare_parameter('max_speed', 3.0)              # m/s
        self.declare_parameter('max_steering_angle', 0.4189)  # rad (~24 deg)
        self.declare_parameter('brake_speed', 0.0)            # m/s
        self.declare_parameter('aeb_hold_duration', 0.5)      # seconds

        safe_drive_topic = self.get_parameter(
            'safe_drive_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        # State
        self.last_odom_v = None
        self.last_steering_angle = 0.0
        self.aeb_active = False
        self.aeb_last_trigger_time = None

        # Publisher: the sole writer to drive_topic
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # Subscribers
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.safe_drive_sub = self.create_subscription(
            AckermannDriveStamped, safe_drive_topic, self.safe_drive_callback, 10
        )

        self.get_logger().info(
            f'SafetyNode initialized: {safe_drive_topic} -> {drive_topic} '
            f'(scan: {scan_topic}, odom: {odom_topic})'
        )

    def odom_callback(self, msg: Odometry):
        """Track the latest forward speed for TTC computation."""
        self.last_odom_v = msg.twist.twist.linear.x

    def scan_callback(self, scan: LaserScan):
        """Compute TTC-based AEB and latch/release the brake state."""
        enable_ttc = self.get_parameter('enable_ttc').get_parameter_value().bool_value
        if not enable_ttc:
            self.aeb_active = False
            return

        ttc_threshold = self.get_parameter('ttc_threshold').get_parameter_value().double_value
        aeb_hold_duration = self.get_parameter(
            'aeb_hold_duration').get_parameter_value().double_value

        triggered = False
        v = self.last_odom_v
        if v is not None:
            angle = scan.angle_min
            for r in scan.ranges:
                valid = (not math.isnan(r) and not math.isinf(r)
                         and scan.range_min <= r <= scan.range_max)
                if valid:
                    closing_speed = v * math.cos(angle)
                    if closing_speed > _CLOSING_SPEED_EPS:
                        ttc = r / closing_speed
                        if ttc < ttc_threshold:
                            triggered = True
                            break
                angle += scan.angle_increment

        now = self.get_clock().now()
        if triggered:
            self.aeb_last_trigger_time = now
            self.aeb_active = True
        elif self.aeb_active and self.aeb_last_trigger_time is not None:
            held_for = (now - self.aeb_last_trigger_time).nanoseconds / 1e9
            if held_for >= aeb_hold_duration:
                self.aeb_active = False

        if self.aeb_active:
            self.get_logger().warn('AEB triggered: braking', throttle_duration_sec=0.5)
            self._publish_brake()

    def safe_drive_callback(self, msg: AckermannDriveStamped):
        """Apply clipping and AEB override, then publish the arbitrated command."""
        enable_clipping = self.get_parameter('enable_clipping').get_parameter_value().bool_value
        enable_ttc = self.get_parameter('enable_ttc').get_parameter_value().bool_value

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = msg.drive.speed
        out.drive.steering_angle = msg.drive.steering_angle

        if enable_clipping:
            max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
            max_steer = self.get_parameter('max_steering_angle').get_parameter_value().double_value
            out.drive.speed = max(-max_speed, min(max_speed, out.drive.speed))
            out.drive.steering_angle = max(-max_steer, min(max_steer, out.drive.steering_angle))

        self.last_steering_angle = out.drive.steering_angle

        if enable_ttc and self.aeb_active:
            brake_speed = self.get_parameter('brake_speed').get_parameter_value().double_value
            out.drive.speed = brake_speed
            if enable_clipping:
                max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
                out.drive.speed = max(-max_speed, min(max_speed, out.drive.speed))

        self.drive_pub.publish(out)

    def _publish_brake(self):
        """Directly publish a brake command, independent of /safe_drive."""
        enable_clipping = self.get_parameter('enable_clipping').get_parameter_value().bool_value
        brake_speed = self.get_parameter('brake_speed').get_parameter_value().double_value
        steering = self.last_steering_angle

        if enable_clipping:
            max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
            max_steer = self.get_parameter('max_steering_angle').get_parameter_value().double_value
            brake_speed = max(-max_speed, min(max_speed, brake_speed))
            steering = max(-max_steer, min(max_steer, steering))

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = brake_speed
        out.drive.steering_angle = steering
        self.drive_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
