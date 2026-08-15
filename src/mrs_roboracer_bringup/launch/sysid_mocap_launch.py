"""
Mocap launch: mocap_pose_adapter + sysid_data_collector + safety_node.

`mocap4r2_optitrack_driver` (run separately; must be lifecycle-activated
first, see README §6) publishes all tracked rigid bodies on `rigid_bodies`
(mocap4r2_msgs/RigidBodies) with no per-body PoseStamped and no twist.
`mocap_pose_adapter_node` picks the rigid body named `rigid_body_name` and
republishes it as geometry_msgs/PoseStamped on `pose_topic`. There is no
mocap twist source -- `twist_topic` defaults empty; velocity comes from
pose-derivative (`velocity_estimators.py`), same as the offline pipeline.

Commands flow sysid_data_collector -> /safe_drive -> safety_node -> /drive,
per the cmd-mux contract. `ros2 bag record` is node-managed.
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
        'pose_topic', default_value='/mocap/pose',
        description='PoseStamped topic published by mocap_pose_adapter_node.')
    twist_topic_arg = DeclareLaunchArgument(
        'twist_topic', default_value='',
        description='No mocap twist source exists; left empty (set to enable, if added later).')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='',
        description='Onboard odometry topic, if available (set "" to disable).')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path', default_value='/home/mrslab3/sysid_bags/session',
        description='Output path for the node-managed rosbag2 recording '
                    '(ros2 bag record refuses to write into an existing dir).')
    rigid_bodies_topic_arg = DeclareLaunchArgument(
        'rigid_bodies_topic', default_value='/rigid_bodies',
        description='mocap4r2_msgs/RigidBodies topic published by mocap4r2_optitrack_driver.')
    rigid_body_name_arg = DeclareLaunchArgument(
        'rigid_body_name',
        description='Rigid body name (as configured in Motive) for this car. Required.')

    mocap_pose_adapter_node = Node(
        package='mrs_roboracer_sysid',
        executable='mocap_pose_adapter_node',
        name='mocap_pose_adapter',
        output='screen',
        parameters=[{
            'rigid_bodies_topic': LaunchConfiguration('rigid_bodies_topic'),
            'rigid_body_name': LaunchConfiguration('rigid_body_name'),
            'pose_topic': LaunchConfiguration('pose_topic'),
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
        }],
    )

    for action in (params_file_arg, pose_topic_arg, twist_topic_arg, odom_topic_arg,
                   bag_path_arg, rigid_bodies_topic_arg, rigid_body_name_arg):
        ld.add_action(action)

    ld.add_action(mocap_pose_adapter_node)
    ld.add_action(sysid_data_collector_node)
    ld.add_action(safety_node)

    return ld
