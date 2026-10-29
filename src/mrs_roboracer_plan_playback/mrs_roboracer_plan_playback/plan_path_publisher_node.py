"""Publishes a plan CSV's (x, y, yaw) columns as a single transient-local
nav_msgs/Path, for pure_pursuit_node's `incoming_path_topic` (or anything
else) to track.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from mrs_roboracer_plan_playback.plan_csv import load_plan


def _yaw_to_quat_z_w(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PlanPathPublisher(Node):

    def __init__(self):
        super().__init__('plan_path_publisher')

        self.declare_parameter('plan_csv', '')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('frame_id', 'map')

        plan_csv = self.get_parameter('plan_csv').get_parameter_value().string_value
        if not plan_csv:
            raise RuntimeError('plan_path_publisher requires plan_csv:=<path to a plan .csv>')
        plan = load_plan(plan_csv)

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for row in plan.rows:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = row.x
            pose.pose.position.y = row.y
            qz, qw = _yaw_to_quat_z_w(row.yaw)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        # Transient-local, matching pure_pursuit_node's path subscriber, so it
        # still gets this one-shot publish even if it starts up a moment later.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(Path, path_topic, qos)
        self._path_pub.publish(path)
        self.get_logger().info(
            f'Loaded {plan_csv}: published {len(path.poses)} waypoints on {path_topic}')


def main(args=None):
    rclpy.init(args=args)
    node = PlanPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
