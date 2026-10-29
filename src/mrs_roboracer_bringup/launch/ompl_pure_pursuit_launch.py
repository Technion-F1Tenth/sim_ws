"""
Plan with OMPL, track with pure pursuit.

platform + planning_node + pure_pursuit_node + safety_node:

    ros2 launch mrs_roboracer_bringup ompl_pure_pursuit_launch.py mode:=sim|onboard|mocap \
        [map:=<map .yaml> (onboard)] [planner_type:=dubins|rrt|rcs]

planning_node (mrs_roboracer_ompl) reads the latched map on `topics['map']`
and the car's map-frame pose on `topics['pose']`. Each goal on
`goal_pose_topic` (Foxglove/RViz "2D Goal Pose") triggers one plan, published
as a latched nav_msgs/Path on `path_topic`. The planner only plans: tracking
is pure_pursuit_node's job (mrs_roboracer_pp_cpp), which reloads the path
whenever a new one arrives.

The planner gets the safety node's `max_speed`/`max_steering_angle` as its
vehicle limits, so plans stay inside what safety_node lets through. Onboard
mode starts localization (map_server + particle_filter) and needs `map:=`;
mocap mode needs mocap_map_launch.py running on the pit PC for /mocap_map.
In sim, rviz2 starts with rviz/ompl_sim.rviz (map, car, scan, path, goal);
`rviz:=false` skips it.
Commands flow pure_pursuit_node -> /safe_drive -> safety_node -> /drive.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mrs_roboracer_bringup.modes import (include_platform, MODE_DESCRIPTION, resolve_topics,
                                         topic_arguments)
from mrs_roboracer_bringup.safety import safety_arguments, safety_node


def _nodes(context):
    mode, topics = resolve_topics(context)
    planning_node = Node(
        package='mrs_roboracer_ompl',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{
            'odom_topic': topics['pose'],
            'map_topic': topics['map'],
            'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
            'planned_path_topic': LaunchConfiguration('path_topic'),
            'planner_type': LaunchConfiguration('planner_type'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'planning_timeout_sec': LaunchConfiguration('planning_timeout_sec'),
            'min_turning_radius': LaunchConfiguration('min_turning_radius'),
            'path_resolution': LaunchConfiguration('path_resolution'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'wheelbase': LaunchConfiguration('planner_wheelbase'),
            'max_velocity': LaunchConfiguration('max_speed'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'max_steering_rate': LaunchConfiguration('max_steering_rate'),
            'rcs_sweep.enabled': LaunchConfiguration('rcs_sweep'),
            'rcs_sweep.config_path': LaunchConfiguration('rcs_sweep_config'),
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
            'pause_topic': LaunchConfiguration('pause_service'),
            'resume_topic': LaunchConfiguration('resume_service'),
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
    actions = [safety_node(topics), planning_node, pure_pursuit_node]
    if mode == 'sim' and LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description=MODE_DESCRIPTION),
        *topic_arguments('scan', 'odom', 'pose', 'map'),
        *safety_arguments(max_speed='3.0'),
        DeclareLaunchArgument(
            'goal_pose_topic', default_value='/goal_pose',
            description='geometry_msgs/PoseStamped goal; each one triggers a new plan.'),
        DeclareLaunchArgument(
            'path_topic', default_value='/planned_path',
            description='nav_msgs/Path from planning_node to pure_pursuit_node.'),
        # Planner
        DeclareLaunchArgument(
            'planner_type', default_value='dubins',
            description="'dubins' (RRT* on Dubins curves: forward-only, curvature-limited), "
                        "'rrt' (kinodynamic RRT), or 'rcs' (only if built with RCS)."),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.25',
            description="Goal tolerance in the planner's state-space distance."),
        DeclareLaunchArgument(
            'planning_timeout_sec', default_value='1.0',
            description='Planner time budget per goal (s); dubins (RRT*) optimizes for all of it.'),
        DeclareLaunchArgument(
            'min_turning_radius', default_value='1.0',
            description='Dubins turning radius (m). Car limit is ~0.75 m '
                        '(wheelbase / tan(max_steering_angle)); margin helps pure pursuit.'),
        DeclareLaunchArgument(
            'path_resolution', default_value='0.1',
            description='Spacing (m) of the published path poses (dubins).'),
        DeclareLaunchArgument(
            'robot_radius', default_value='0.3',
            description='Obstacle inflation radius (m).'),
        DeclareLaunchArgument(
            'planner_wheelbase', default_value='0.2032',
            description="Wheelbase (m) in the planner's kinematic model."),
        DeclareLaunchArgument(
            'max_acceleration', default_value='0.5',
            description='Planner acceleration bound (m/s^2).'),
        DeclareLaunchArgument(
            'max_steering_rate', default_value='0.035',
            description='Planner steering-rate bound (rad per control step).'),
        DeclareLaunchArgument(
            'rcs_sweep', default_value='false',
            description='Run the RCSStar parameter sweep on the next goal (planner_type:=rcs).'),
        DeclareLaunchArgument(
            'rcs_sweep_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mrs_roboracer_ompl'), 'config', 'rcs_sweep.yaml']),
            description='RCSStar sweep grid (YAML).'),
        # Pure pursuit
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
            'stop_distance', default_value='0.20',
            description='Distance (m) to the final path point considered "reached".'),
        # Visualization (sim only; on the car use Foxglove from the pit PC)
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start rviz2 in mode:=sim (2D Goal Pose tool publishes /goal_pose).'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mrs_roboracer_bringup'), 'rviz', 'ompl_sim.rviz']),
            description='rviz2 config file.'),
        OpaqueFunction(function=_nodes),
        include_platform(localization='true'),
    ])
