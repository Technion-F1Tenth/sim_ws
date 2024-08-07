from rclpy.node import Node
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped

class TestNode(Node):
    ego_drive_topic = '/drive'
    opp_drive_topic = '/opp_drive'

    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Test node created')

        self.ego_drive_pub = self.create_publisher(AckermannDriveStamped, self.ego_drive_topic, 10)
        self.opp_drive_pub = self.create_publisher(AckermannDriveStamped, self.opp_drive_topic, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 1.0
        msg.drive.steering_angle = 0.0
        self.get_logger().info('Publishing: {}'.format(msg))
        self.ego_drive_pub.publish(msg)
        self.opp_drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()