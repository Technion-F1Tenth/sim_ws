"""
Pure pursuit (C++): platform + pure_pursuit_node + safety_node.

    ros2 launch mrs_roboracer_bringup pure_pursuit_safety_launch.py mode:=sim|onboard|mocap \
        raceline_path:=<csv> init_path:=true

Tracks a raceline/centerline CSV loaded at startup (`raceline_path`,
`init_path:=true`), or a nav_msgs/Path received on `incoming_path_topic`
(default /planned_path, for a future global planner to publish into).

The controller needs a map-frame pose, so onboard mode requires
`localization:=true` and `map:=<map .yaml>`.
Commands flow pure_pursuit_node -> /safe_drive -> safety_node -> /drive.
Pause/resume via std_srvs/Trigger on `pause_service`/`resume_service`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mrs_roboracer_bringup.modes import (include_platform, MODE_DESCRIPTION, resolve_topics,
                                         topic_arguments)
from mrs_roboracer_bringup.safety import safety_arguments, safety_node


def _nodes(context):
    _, topics = resolve_topics(context)
    pure_pursuit_node = Node(
        package='mrs_roboracer_pp_cpp',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'odom_topic': topics['pose'],
            'drive_topic': LaunchConfiguration('safe_drive_topic'),
            'pause_topic': LaunchConfiguration('pause_service'),
            'resume_topic': LaunchConfiguration('resume_service'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'speed': LaunchConfiguration('speed'),
            'max_steer': LaunchConfiguration('max_steering_angle'),
            'raceline_path': LaunchConfiguration('raceline_path'),
            'init_path': LaunchConfiguration('init_path'),
            'incoming_path_topic': LaunchConfiguration('incoming_path_topic'),
            'stop_at_end': LaunchConfiguration('stop_at_end'),
            'stop_distance': LaunchConfiguration('stop_distance'),
        }],
    )
    return [safety_node(topics), pure_pursuit_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description=MODE_DESCRIPTION),
        *topic_arguments('scan', 'odom', 'pose'),
        *safety_arguments(max_speed='3.0'),
        DeclareLaunchArgument(
            'pause_service', default_value='/pure_pursuit/pause',
            description='std_srvs/Trigger: zero the drive command and pause tracking.'),
        DeclareLaunchArgument(
            'resume_service', default_value='/pure_pursuit/resume',
            description='std_srvs/Trigger: resume tracking.'),
        DeclareLaunchArgument(
            'wheelbase', default_value='0.28',
            description='Vehicle wheelbase (m), used in the curvature-to-steering model.'),
        DeclareLaunchArgument(
            'lookahead_distance', default_value='1.0',
            description='Pure pursuit lookahead distance (m).'),
        DeclareLaunchArgument(
            'speed', default_value='1.0',
            description='Constant commanded speed (m/s) while tracking.'),
        DeclareLaunchArgument(
            'raceline_path', default_value='',
            description='CSV (s;x;y;... or similar) loaded at startup if init_path:=true. '
                        'Required when init_path:=true.'),
        DeclareLaunchArgument(
            'init_path', default_value='false',
            description='Load raceline_path at startup instead of waiting for '
                        'incoming_path_topic.'),
        DeclareLaunchArgument(
            'incoming_path_topic', default_value='/planned_path',
            description='nav_msgs/Path topic that (re)loads the tracked path at runtime.'),
        DeclareLaunchArgument(
            'stop_at_end', default_value='true',
            description='Brake and pause once within stop_distance of the final path point.'),
        DeclareLaunchArgument(
            'stop_distance', default_value='0.20',
            description='Distance (m) to the final path point considered "reached".'),
        OpaqueFunction(function=_nodes),
        include_platform(localization='true'),
    ])
