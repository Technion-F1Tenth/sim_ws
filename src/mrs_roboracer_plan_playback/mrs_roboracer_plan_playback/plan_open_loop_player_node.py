"""Open-loop playback of a plan CSV's recorded actions.

Replays the plan's speed_setpoint/steer_setpoint columns directly as
AckermannDriveStamped commands at the plan's own dt -- no odometry feedback
at all. The plan can come from any source (a planner, a logged run, hand-
authored setpoints) as long as it's in plan_csv's format. safety_node's
clipping/TTC-AEB (see the cmd-mux contract) is the only thing between this
and the vehicle. Once the recorded commands run out, this node holds a
zero-speed brake command on a timer indefinitely, so a finished or killed
run never leaves a stale nonzero command latched on /safe_drive.
"""

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

from mrs_roboracer_plan_playback.plan_csv import load_plan


class PlanOpenLoopPlayer(Node):

    def __init__(self):
        super().__init__('plan_open_loop_player')

        self.declare_parameter('plan_csv', '')
        self.declare_parameter('drive_topic', '/safe_drive')

        plan_csv = self.get_parameter('plan_csv').get_parameter_value().string_value
        if not plan_csv:
            raise RuntimeError('plan_open_loop_player requires plan_csv:=<path to a plan .csv>')
        self._plan = load_plan(plan_csv)
        self._index = 0

        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self._drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.get_logger().info(
            f'Loaded {plan_csv}: {len(self._plan.rows)} open-loop steps @ '
            f'dt={self._plan.dt:.3f}s -> {drive_topic}')

        self._timer = self.create_timer(self._plan.dt, self._tick)

    def _publish(self, speed: float, steer: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self._drive_pub.publish(msg)

    def _tick(self):
        if self._index < len(self._plan.rows):
            row = self._plan.rows[self._index]
            self._publish(row.speed_setpoint, row.steer_setpoint)
            self._index += 1
            if self._index == len(self._plan.rows):
                self.get_logger().info('Plan exhausted; holding brake command.')
            return
        self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PlanOpenLoopPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
