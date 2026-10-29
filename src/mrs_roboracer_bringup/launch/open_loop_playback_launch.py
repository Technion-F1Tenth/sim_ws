"""
Replay a recorded plan's actions open-loop: platform + plan_open_loop_player
+ safety_node.

    ros2 launch mrs_roboracer_bringup open_loop_playback_launch.py mode:=sim|mocap \
        plan_csv:=<path to a plan .csv>

plan_open_loop_player replays the plan's speed_setpoint/steer_setpoint
columns directly as AckermannDriveStamped commands at the plan's own dt --
no odometry feedback. The plan can come from any source (a planner's output,
a logged run, hand-authored setpoints) as long as it's in plan_csv's format
(see mrs_roboracer_plan_playback/plan_csv.py); it's not specific to any one
planner. safety_node's clipping/TTC-AEB is the only thing between this and
the vehicle; it still needs `odom`/`scan` from the platform, which is why
this launch still takes `mode`.

No `onboard` mode, since a recorded plan's coordinates generally assume
whatever map frame it was recorded in (e.g. the arena/mocap frame), not an
independently-localized map.
Commands flow plan_open_loop_player -> /safe_drive -> safety_node -> /drive.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mrs_roboracer_bringup.modes import (include_platform, MODE_DESCRIPTION, resolve_topics,
                                         topic_arguments)
from mrs_roboracer_bringup.safety import safety_arguments, safety_node


def _nodes(context):
    _, topics = resolve_topics(context, allowed=('sim', 'mocap'))
    player = Node(
        package='mrs_roboracer_plan_playback',
        executable='plan_open_loop_player',
        name='plan_open_loop_player',
        output='screen',
        parameters=[{
            'plan_csv': LaunchConfiguration('plan_csv'),
            'drive_topic': LaunchConfiguration('safe_drive_topic'),
        }],
    )
    return [safety_node(topics), player]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description=MODE_DESCRIPTION + " ('sim' or 'mocap')"),
        *topic_arguments('scan', 'odom'),
        *safety_arguments(max_speed='3.0'),
        DeclareLaunchArgument(
            'plan_csv', description='Plan CSV (see plan_csv.py) to replay open-loop.'),
        OpaqueFunction(function=_nodes),
        include_platform(localization='false'),
    ])
