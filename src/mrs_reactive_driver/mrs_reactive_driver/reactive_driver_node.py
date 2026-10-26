import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import SetBool
from std_msgs.msg import String
from enum import Enum


class DriverState(Enum):
    INACTIVE = "inactive"
    ACTIVE = "active"


class ReactiveDriverNode(Node):
    def __init__(self):
        super().__init__('reactive_driver_node')

        # Declare topic parameters with default values
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/safe_drive')
        self.declare_parameter('state_topic', '/reactive_driver/state')
        self.declare_parameter('activate_service', '/reactive_driver/activate')

        # Get topic parameter values
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        activate_service_name = self.get_parameter('activate_service').get_parameter_value().string_value

        # Driver parameters
        self.min_distance = 1.0     # meters: a ray > min_distance is considered "free"
        self.cruise_velocity = 0.5  # m/s
        self.max_steer = 0.5        # rad: clamp steering to something reasonable

        # State management
        self.state = DriverState.ACTIVE
        
        # Publishers and subscribers
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.state_publisher = self.create_publisher(String, state_topic, 10)
        
        # Use sensor-data QoS so LaserScan actually arrives
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        self.subscription = self.create_subscription(
            LaserScan, scan_topic, self.laser_callback, qos
        )
        
        # Services for state control
        self.activate_service = self.create_service(
            SetBool, activate_service_name, self.activate_callback
        )
        
        # Timer to publish state periodically
        self.state_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('ReactiveDriverNode initialized - active by default, use /reactive_driver/activate service to control')
        
    def activate_callback(self, request, response):
        """Service callback to activate/deactivate the driver."""
        if request.data:
            # Activate
            self.state = DriverState.ACTIVE
            response.success = True
            response.message = "Reactive driver activated - ready to drive!"
            self.get_logger().info(response.message)
        else:
            # Deactivate - stop the car first
            self._publish_stop()
            self.state = DriverState.INACTIVE
            response.success = True
            response.message = "Reactive driver deactivated - car stopped"
            self.get_logger().info(response.message)
        
        return response

    def publish_state(self):
        """Publish current state."""
        msg = String()
        msg.data = self.state.value
        self.state_publisher.publish(msg)

    def laser_callback(self, scan: LaserScan):
        """Process laser scan data - only when node is active."""
        # Only process if we're in the active state
        if self.state != DriverState.ACTIVE:
            return
        
        # 1) Preprocess ranges (very basic: clamp to sensor limits, replace bad values with range_max)
        rmin = max(scan.range_min, 0.0)
        rmax = scan.range_max
        ranges = []
        for r in scan.ranges:
            if r is None or math.isnan(r) or math.isinf(r):
                ranges.append(rmax)
            else:
                ranges.append(min(max(r, rmin), rmax))

        n = len(ranges)
        if n == 0 or scan.angle_increment == 0.0:
            self._publish(0.0, 0.0, note='empty scan')
            return

        # 2) Find the largest contiguous "gap" where range > min_distance
        max_gap_start, max_gap_end = -1, -2
        in_gap = False
        gap_start = 0

        for i, d in enumerate(ranges):
            if d > self.min_distance:
                if not in_gap:
                    in_gap = True
                    gap_start = i
            else:
                if in_gap:
                    in_gap = False
                    gap_end = i - 1
                    if (gap_end - gap_start) > (max_gap_end - max_gap_start):
                        max_gap_start, max_gap_end = gap_start, gap_end

        # Close a gap that reaches the end of the scan
        if in_gap:
            gap_end = n - 1
            if (gap_end - gap_start) > (max_gap_end - max_gap_start):
                max_gap_start, max_gap_end = gap_start, gap_end

        # 3) If no gap found, publish a STOP (important!)
        if max_gap_end < max_gap_start:
            self.get_logger().info('No valid gap found, stopping.')
            self._publish(0.0, 0.0, note='no gap')
            return

        # 4) Aim at the center of the best gap
        gap_center = (max_gap_start + max_gap_end) // 2

        # Find index corresponding to 0 rad (forward) robustly (don't assume middle is zero)
        idx_forward = int(round((0.0 - scan.angle_min) / scan.angle_increment))
        idx_forward = max(0, min(n - 1, idx_forward))

        # steering = angle from forward to the gap center
        steering_angle = (gap_center - idx_forward) * scan.angle_increment
        # Clamp steering to avoid crazy commands
        steering_angle = max(-self.max_steer, min(self.max_steer, steering_angle))

        velocity = self.cruise_velocity

        self.get_logger().info(
            f'gap [{max_gap_start},{max_gap_end}] -> center {gap_center}, '
            f'steer {steering_angle:.2f} rad, v {velocity:.2f} m/s'
        )
        self._publish(steering_angle, velocity)

    def _publish(self, steering: float, speed: float, note: str = ''):
        """Publish drive command."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.publisher.publish(msg)
        if note:
            self.get_logger().debug(f'publish: {note}')

    def _publish_stop(self):
        """Send a stop command to the car."""
        self._publish(0.0, 0.0, note='stop command')


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()