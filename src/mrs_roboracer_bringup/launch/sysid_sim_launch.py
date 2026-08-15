"""
Sim dev launch: odom->pose/twist adapter + sysid_data_collector + safety_node.

Does not launch the f1tenth_gym_ros bridge itself -- launch that separately,
then bring this up. Commands flow sysid_data_collector -> /safe_drive ->
safety_node -> /drive -> sim, per the cmd-mux contract (the sysid node never
publishes /drive directly).

`ros2 bag record` is node-managed (started/stopped by sysid_data_collector
itself), so there is no recording launch action here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('mrs_roboracer_sysid')
    default_params_file = os.path.join(pkg_share, 'config', 'sysid_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Path to sysid_params.yaml.')
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic', default_value='/sysid/pose',
        description='Ground-truth pose topic consumed by sysid_data_collector.')
    twist_topic_arg = DeclareLaunchArgument(
        'twist_topic', default_value='/sysid/twist',
        description='Ground-truth twist topic (set "" to disable).')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/ego_racecar/odom',
        description='Sim odometry topic; source for the pose/twist adapter.')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path', default_value='/home/mrslab3/sysid_bags/session',
        description='Output path for the node-managed rosbag2 recording '
                    '(ros2 bag record refuses to write into an existing dir).')

    odom_to_pose_adapter_node = Node(
        package='mrs_roboracer_sysid',
        executable='odom_to_pose_adapter_node',
        name='odom_to_pose_adapter',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'odom_topic': LaunchConfiguration('odom_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'twist_topic': LaunchConfiguration('twist_topic'),
        }],
    )

    sysid_data_collector_node = Node(
        package='mrs_roboracer_sysid',
        executable='sysid_data_collector',
        name='sysid_data_collector',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'io.pose_topic': LaunchConfiguration('pose_topic'),
            'io.twist_topic': LaunchConfiguration('twist_topic'),
            'io.odom_topic': LaunchConfiguration('odom_topic'),
            'io.bag_path': LaunchConfiguration('bag_path'),
        }],
    )

    safety_node = Node(
        package='mrs_roboracer_safety',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[{
            'safe_drive_topic': '/safe_drive',
            'drive_topic': '/drive',
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
    )

    for action in (params_file_arg, pose_topic_arg, twist_topic_arg, odom_topic_arg,
                   bag_path_arg):
        ld.add_action(action)

    ld.add_action(odom_to_pose_adapter_node)
    ld.add_action(sysid_data_collector_node)
    ld.add_action(safety_node)

    return ld
