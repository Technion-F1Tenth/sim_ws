"""
Reactive (gap-following) driver: platform + reactive_driver_node + safety_node.

    ros2 launch mrs_roboracer_bringup reactive_driver_safety_launch.py mode:=sim|onboard|mocap

The driver only needs /scan, so onboard mode runs without localization.
Enable it through `/reactive_driver/activate` (std_srvs/SetBool).
Commands flow reactive_driver_node -> /safe_drive -> safety_node -> /drive.
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
    reactive_driver_node = Node(
        package='mrs_reactive_driver',
        executable='reactive_driver_node',
        name='reactive_driver_node',
        output='screen',
        parameters=[{
            'scan_topic': topics['scan'],
            'drive_topic': LaunchConfiguration('safe_drive_topic'),
            'state_topic': LaunchConfiguration('state_topic'),
            'activate_service': LaunchConfiguration('activate_service'),
        }],
    )
    return [safety_node(topics), reactive_driver_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description=MODE_DESCRIPTION),
        *topic_arguments('scan', 'odom'),
        *safety_arguments(max_speed='3.0'),
        DeclareLaunchArgument(
            'activate_service', default_value='/reactive_driver/activate',
            description='Service used to activate/deactivate the reactive driver.'),
        DeclareLaunchArgument(
            'state_topic', default_value='/reactive_driver/state',
            description='Topic on which the reactive driver publishes its state.'),
        OpaqueFunction(function=_nodes),
        include_platform(localization='false'),
    ])
