import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    default_checkpoint_path = os.path.join(
        get_package_share_directory('mrs_roboracer_inference'), 'model', 'policy.pt')

    # Topic names
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='LaserScan topic consumed by the inference and safety nodes.')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/ego_racecar/odom',
        description='Odometry topic used by both the inference node (velocity input) '
                     'and the safety node (TTC).')
    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic', default_value='/drive',
        description='Final arbitrated drive command topic (safety node owns this).')
    safe_drive_topic_arg = DeclareLaunchArgument(
        'safe_drive_topic', default_value='/safe_drive',
        description='Unarbitrated drive command topic published by the inference node.')

    # Safety behavior. max_speed is raised above the default 3.0 m/s used by the
    # reactive driver launch -- this policy's trained operating range averages
    # ~7.6 m/s, so the stock default would clip it hard.
    enable_clipping_arg = DeclareLaunchArgument(
        'enable_clipping', default_value='true',
        description='Enable clamping of speed/steering to vehicle limits.')
    enable_ttc_arg = DeclareLaunchArgument(
        'enable_ttc', default_value='true',
        description='Enable TTC-based AEB override.')
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='8.5',
        description='Max |speed| (m/s) when clipping is enabled.')
    max_steering_angle_arg = DeclareLaunchArgument(
        'max_steering_angle', default_value='0.4189',
        description='Max |steering_angle| (rad) when clipping is enabled.')
    ttc_threshold_arg = DeclareLaunchArgument(
        'ttc_threshold', default_value='1.0',
        description='TTC threshold (s) below which AEB triggers.')
    brake_speed_arg = DeclareLaunchArgument(
        'brake_speed', default_value='0.0',
        description='Speed (m/s) commanded while AEB is active.')
    aeb_hold_duration_arg = DeclareLaunchArgument(
        'aeb_hold_duration', default_value='0.5',
        description='Minimum time (s) AEB stays latched once triggered.')

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

    safety_node = Node(
        package='mrs_roboracer_safety',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[{
            'safe_drive_topic': LaunchConfiguration('safe_drive_topic'),
            'drive_topic': LaunchConfiguration('drive_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'enable_clipping': LaunchConfiguration('enable_clipping'),
            'enable_ttc': LaunchConfiguration('enable_ttc'),
            'max_speed': LaunchConfiguration('max_speed'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'ttc_threshold': LaunchConfiguration('ttc_threshold'),
            'brake_speed': LaunchConfiguration('brake_speed'),
            'aeb_hold_duration': LaunchConfiguration('aeb_hold_duration'),
        }],
    )

    inference_node = Node(
        package='mrs_roboracer_inference',
        executable='inference_node',
        name='inference_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
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

    for action in (
        scan_topic_arg, odom_topic_arg, drive_topic_arg, safe_drive_topic_arg,
        enable_clipping_arg, enable_ttc_arg, max_speed_arg,
        max_steering_angle_arg, ttc_threshold_arg, brake_speed_arg,
        aeb_hold_duration_arg, checkpoint_path_arg, n_lidar_beams_arg,
        lidar_stack_n_arg, fov_deg_arg, max_range_arg, v_max_arg,
        max_steer_vel_arg, max_accel_arg, control_rate_hz_arg, deterministic_arg,
    ):
        ld.add_action(action)

    ld.add_action(safety_node)
    ld.add_action(inference_node)

    return ld
