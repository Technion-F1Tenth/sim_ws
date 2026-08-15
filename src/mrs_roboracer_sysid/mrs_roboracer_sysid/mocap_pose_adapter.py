"""
Republish one named mocap4r2_msgs/RigidBodies entry as geometry_msgs/PoseStamped.

The mocap4r2_optitrack_driver publishes all tracked rigid bodies in a single
RigidBodies message per frame (shared header, no per-body twist). This node
picks the body matching `rigid_body_name` and republishes it under the
mocap integration contract's pose topic so `sysid_data_collector` can
consume it like any other PoseStamped source. No twist is synthesized here;
pose-derivative velocity (already used by `velocity_estimators.py`) is the
supported velocity source for mocap runs.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies


class MocapPoseAdapter(Node):

    def __init__(self):
        super().__init__('mocap_pose_adapter')

        self.declare_parameter('rigid_bodies_topic', '/rigid_bodies')
        self.declare_parameter('rigid_body_name', '')
        self.declare_parameter('pose_topic', '/mocap/pose')

        rigid_bodies_topic = self.get_parameter(
            'rigid_bodies_topic').get_parameter_value().string_value
        self.rigid_body_name = self.get_parameter(
            'rigid_body_name').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        if not self.rigid_body_name:
            raise ValueError('mocap_pose_adapter requires a non-empty rigid_body_name parameter')

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.rigid_bodies_sub = self.create_subscription(
            RigidBodies, rigid_bodies_topic, self._rigid_bodies_callback, 10)

        self.get_logger().info(
            f'MocapPoseAdapter: {rigid_bodies_topic} (rigid_body_name='
            f'"{self.rigid_body_name}") -> {pose_topic}'
        )

    def _rigid_bodies_callback(self, msg: RigidBodies) -> None:
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == self.rigid_body_name:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = rb.pose
                self.pose_pub.publish(pose)
                return

        self.get_logger().warn(
            f'rigid body "{self.rigid_body_name}" not present in this RigidBodies frame'
            ' (out of view / not tracked)',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MocapPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
