"""
DEV-ONLY: republish nav_msgs/Odometry as PoseStamped (+ optional TwistStamped).

The mocap integration contract (§8) expects pose (and optionally twist) on
dedicated topics. In sim, `/ego_racecar/odom` already carries both; this
node just republishes the relevant fields under that contract's topic
names so `sysid_data_collector` can be developed/tested without mocap.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class OdomToPoseAdapter(Node):

    def __init__(self):
        super().__init__('odom_to_pose_adapter')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('pose_topic', '/sysid/pose')
        self.declare_parameter('twist_topic', '/sysid/twist')
        self.declare_parameter('publish_twist', True)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        self.publish_twist = self.get_parameter('publish_twist').get_parameter_value().bool_value

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.twist_pub = (
            self.create_publisher(TwistStamped, twist_topic, 10) if self.publish_twist else None
        )

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        self.get_logger().info(
            f'OdomToPoseAdapter: {odom_topic} -> {pose_topic}'
            + (f', {twist_topic}' if self.publish_twist else '')
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

        if self.publish_twist:
            twist = TwistStamped()
            twist.header = msg.header
            twist.twist = msg.twist.twist
            self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
