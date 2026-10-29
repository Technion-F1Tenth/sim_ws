"""
Track an OL_MCTS plan with pure pursuit: platform + plan_path_publisher +
pure_pursuit_node + safety_node.

    ros2 launch mrs_roboracer_bringup mcts_pure_pursuit_launch.py mode:=sim|mocap \
        plan_csv:=<path to a kmcts-*.csv>

plan_path_publisher loads the plan's (x, y, yaw) columns once at startup and
publishes them as a latched nav_msgs/Path on `path_topic` (default
/planned_path); pure_pursuit_node tracks it, closed-loop, through
odom/`topics['pose']`. Default plan_csv is the best-performing run (0
collisions / 10000 rollouts, 99.49% goal rate) from the 2026-10-26
lab-longway-noblock tuning sweep.

No `onboard` mode: the plan's coordinates are in the arena/mocap map frame
(see that scenario's scenario.toml), which only `sim` (arena.yaml is already
f1tenth_gym_ros's default map/sim.yaml) and `mocap` (mrs_mocap_mapping
/mocap_odom, same frame) place the car in.
Commands flow pure_pursuit_node -> /safe_drive -> safety_node -> /drive.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mrs_roboracer_bringup.modes import (include_platform, MODE_DESCRIPTION, resolve_topics,
                                         topic_arguments)
from mrs_roboracer_bringup.safety import safety_arguments, safety_node

_DEFAULT_PLAN = ('/home/mrslab3/mcts/MPUnderUncertainty/OL_MCTS/tuning-runs/'
                  'lab-longway-noblock-2026-10-26/d70/plans/kmcts-d70-t120-s1008.csv')


def _nodes(context):
    _, topics = resolve_topics(context, allowed=('sim', 'mocap'))
    path_publisher = Node(
        package='mrs_roboracer_plan_playback',
        executable='plan_path_publisher',
        name='plan_path_publisher',
        output='screen',
        parameters=[{
            'plan_csv': LaunchConfiguration('plan_csv'),
            'path_topic': LaunchConfiguration('path_topic'),
            'frame_id': 'map',
        }],
    )
    pure_pursuit_node = Node(
        package='mrs_roboracer_pp_cpp',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'odom_topic': topics['pose'],
            'drive_topic': LaunchConfiguration('safe_drive_topic'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'speed': LaunchConfiguration('speed'),
            'max_steer': LaunchConfiguration('max_steering_angle'),
            'init_path': False,
            'incoming_path_topic': LaunchConfiguration('path_topic'),
            'stop_at_end': True,
            'stop_distance': LaunchConfiguration('stop_distance'),
        }],
    )
    return [safety_node(topics), path_publisher, pure_pursuit_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description=MODE_DESCRIPTION + " ('sim' or 'mocap')"),
        *topic_arguments('scan', 'odom', 'pose'),
        *safety_arguments(max_speed='3.0'),
        DeclareLaunchArgument(
            'plan_csv', default_value=_DEFAULT_PLAN,
            description='OL_MCTS plan CSV (kmcts-*.csv) to track.'),
        DeclareLaunchArgument(
            'path_topic', default_value='/planned_path',
            description='nav_msgs/Path from plan_path_publisher to pure_pursuit_node.'),
        DeclareLaunchArgument(
            'wheelbase', default_value='0.28', description='Vehicle wheelbase (m).'),
        DeclareLaunchArgument(
            'lookahead_distance', default_value='1.0',
            description='Pure pursuit lookahead distance (m).'),
        DeclareLaunchArgument(
            'speed', default_value='1.4',
            description='Constant commanded speed (m/s); the plan averages ~1.4 m/s.'),
        DeclareLaunchArgument(
            'stop_distance', default_value='0.20',
            description='Distance (m) to the final waypoint considered "reached".'),
        OpaqueFunction(function=_nodes),
        include_platform(localization='false'),
    ])
