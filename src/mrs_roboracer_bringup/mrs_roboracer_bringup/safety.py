"""safety_node launch arguments and node, shared by every driving launch (cmd-mux contract)."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def safety_arguments(max_speed='3.0'):
    """Launch arguments for safety_node behavior; `max_speed` default is per controller."""
    return [
        DeclareLaunchArgument(
            'safe_drive_topic', default_value='/safe_drive',
            description='Unarbitrated drive command topic published by the controller.'),
        DeclareLaunchArgument(
            'enable_clipping', default_value='true',
            description='Enable clamping of speed/steering to vehicle limits.'),
        DeclareLaunchArgument(
            'enable_ttc', default_value='true',
            description='Enable TTC-based AEB override.'),
        DeclareLaunchArgument(
            'max_speed', default_value=max_speed,
            description='Max |speed| (m/s) when clipping is enabled.'),
        DeclareLaunchArgument(
            'max_steering_angle', default_value='0.4189',
            description='Max |steering_angle| (rad) when clipping is enabled.'),
        DeclareLaunchArgument(
            'ttc_threshold', default_value='1.0',
            description='TTC threshold (s) below which AEB triggers.'),
        DeclareLaunchArgument(
            'brake_speed', default_value='0.0',
            description='Speed (m/s) commanded while AEB is active.'),
        DeclareLaunchArgument(
            'aeb_hold_duration', default_value='0.5',
            description='Minimum time (s) AEB stays latched once triggered.'),
    ]


def safety_node(topics):
    """safety_node: /safe_drive -> clip + AEB -> topics['drive'], the sole writer of /drive."""
    return Node(
        package='mrs_roboracer_safety',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[{
            'safe_drive_topic': LaunchConfiguration('safe_drive_topic'),
            'drive_topic': topics['drive'],
            'scan_topic': topics['scan'],
            'odom_topic': topics['odom'],
            'enable_clipping': LaunchConfiguration('enable_clipping'),
            'enable_ttc': LaunchConfiguration('enable_ttc'),
            'max_speed': LaunchConfiguration('max_speed'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'ttc_threshold': LaunchConfiguration('ttc_threshold'),
            'brake_speed': LaunchConfiguration('brake_speed'),
            'aeb_hold_duration': LaunchConfiguration('aeb_hold_duration'),
        }],
    )
