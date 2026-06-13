from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Topic names
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='LaserScan topic consumed by the reactive driver and safety node.')
    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic', default_value='/drive',
        description='Final arbitrated drive command topic (safety node owns this).')
    safe_drive_topic_arg = DeclareLaunchArgument(
        'safe_drive_topic', default_value='/safe_drive',
        description='Unarbitrated drive command topic published by controllers.')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/ego_racecar/odom',
        description='Odometry topic used by the safety node for TTC.')

    # Safety behavior
    enable_clipping_arg = DeclareLaunchArgument(
        'enable_clipping', default_value='true',
        description='Enable clamping of speed/steering to vehicle limits.')
    enable_ttc_arg = DeclareLaunchArgument(
        'enable_ttc', default_value='true',
        description='Enable TTC-based AEB override.')
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='3.0',
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

    # Reactive driver state/service topics
    activate_service_arg = DeclareLaunchArgument(
        'activate_service', default_value='/reactive_driver/activate',
        description='Service used to activate/deactivate the reactive driver.')
    state_topic_arg = DeclareLaunchArgument(
        'state_topic', default_value='/reactive_driver/state',
        description='Topic on which the reactive driver publishes its state.')

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

    reactive_driver_node = Node(
        package='mrs_reactive_driver',
        executable='reactive_driver_node',
        name='reactive_driver_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'drive_topic': LaunchConfiguration('safe_drive_topic'),
            'state_topic': LaunchConfiguration('state_topic'),
            'activate_service': LaunchConfiguration('activate_service'),
        }],
    )

    for action in (
        scan_topic_arg, drive_topic_arg, safe_drive_topic_arg, odom_topic_arg,
        enable_clipping_arg, enable_ttc_arg, max_speed_arg,
        max_steering_angle_arg, ttc_threshold_arg, brake_speed_arg,
        aeb_hold_duration_arg, activate_service_arg, state_topic_arg,
    ):
        ld.add_action(action)

    ld.add_action(safety_node)
    ld.add_action(reactive_driver_node)

    return ld
