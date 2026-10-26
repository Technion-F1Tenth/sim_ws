"""
RL policy (MinGRU): platform + inference_node + safety_node.

    ros2 launch mrs_roboracer_bringup inference_launch.py mode:=sim|onboard|mocap

The policy only needs /scan and the odom speed, so onboard mode runs without
localization. Observation/action contract: mrs_roboracer_inference/model/MODEL_INFO.md.
Commands flow inference_node -> /safe_drive -> safety_node -> /drive.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mrs_roboracer_bringup.modes import (include_platform, MODE_DESCRIPTION, resolve_topics,
                                         topic_arguments)
from mrs_roboracer_bringup.safety import safety_arguments, safety_node


def _nodes(context):
    _, topics = resolve_topics(context)
    inference_node = Node(
        package='mrs_roboracer_inference',
        executable='inference_node',
        name='inference_node',
        output='screen',
        parameters=[{
            'scan_topic': topics['scan'],
            'odom_topic': topics['odom'],
            'drive_topic': LaunchConfiguration('safe_drive_topic'),
            'checkpoint_path': LaunchConfiguration('checkpoint_path'),
            'n_lidar_beams': LaunchConfiguration('n_lidar_beams'),
            'lidar_stack_n': LaunchConfiguration('lidar_stack_n'),
            'fov_deg': LaunchConfiguration('fov_deg'),
            'max_range': LaunchConfiguration('max_range'),
            'v_max': LaunchConfiguration('v_max'),
            'max_steer_angle': LaunchConfiguration('max_steering_angle'),
            'max_steer_vel': LaunchConfiguration('max_steer_vel'),
            'max_accel': LaunchConfiguration('max_accel'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'deterministic': LaunchConfiguration('deterministic'),
        }],
    )
    return [safety_node(topics), inference_node]


def generate_launch_description():
    default_checkpoint_path = os.path.join(
        get_package_share_directory('mrs_roboracer_inference'), 'model', 'policy.pt')

    # Inference / policy parameters
    checkpoint_path_arg = DeclareLaunchArgument(
        'checkpoint_path', default_value=default_checkpoint_path,
        description='Path to the policy checkpoint (.pt). Defaults to the '
                    'package-installed model/policy.pt.')
    n_lidar_beams_arg = DeclareLaunchArgument(
        'n_lidar_beams', default_value='64',
        description='Beams per stacked LIDAR frame the policy expects.')
    lidar_stack_n_arg = DeclareLaunchArgument(
        'lidar_stack_n', default_value='4',
        description='Number of stacked historical LIDAR frames in the observation.')
    fov_deg_arg = DeclareLaunchArgument(
        'fov_deg', default_value='270.0',
        description='Total body-frame FOV (deg) the resampled beams span.')
    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='30.0',
        description='LIDAR normalization range (m).')
    v_max_arg = DeclareLaunchArgument(
        'v_max', default_value='10.0',
        description="Policy's training-time velocity normalization constant "
                    '(not a real speed cap -- do not change unless retraining).')
    max_steer_vel_arg = DeclareLaunchArgument(
        'max_steer_vel', default_value='3.2',
        description='Clamp (rad/s) on the raw policy steering-rate output.')
    max_accel_arg = DeclareLaunchArgument(
        'max_accel', default_value='9.51',
        description='Clamp (m/s^2) on the raw policy acceleration output.')
    control_rate_hz_arg = DeclareLaunchArgument(
        'control_rate_hz', default_value='20.0',
        description='Policy control loop rate (Hz) -- must match the training dt.')
    deterministic_arg = DeclareLaunchArgument(
        'deterministic', default_value='true',
        description='Use the policy action mean (recommended for deployment) instead of '
                    'sampling.')

    # max_speed is raised above the default 3.0 m/s used by the reactive driver
    # launch: this policy's trained operating range averages ~7.6 m/s, so the
    # stock default would clip it hard.
    return LaunchDescription([
        DeclareLaunchArgument('mode', description=MODE_DESCRIPTION),
        *topic_arguments('scan', 'odom'),
        *safety_arguments(max_speed='8.5'),
        checkpoint_path_arg,
        n_lidar_beams_arg,
        lidar_stack_n_arg,
        fov_deg_arg,
        max_range_arg,
        v_max_arg,
        max_steer_vel_arg,
        max_accel_arg,
        control_rate_hz_arg,
        deterministic_arg,
        OpaqueFunction(function=_nodes),
        include_platform(localization='false'),
    ])
