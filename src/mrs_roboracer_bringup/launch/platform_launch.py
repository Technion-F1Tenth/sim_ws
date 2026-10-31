"""
Platform launch: whatever provides /scan, odom, pose and consumes /drive.

Included by every driving launch with its `mode`; runnable on its own too
(e.g. `mode:=onboard` for joystick teleop only). Topic names per mode are in
`mrs_roboracer_bringup/modes.py`.

- sim:     f1tenth_gym_ros `gym_bridge_launch.py` (map etc. from its sim.yaml).
- onboard: f1tenth_stack `bringup_launch.py` (VESC, lidar, joystick,
           ackermann_mux; /drive is the mux's navigation input). With
           `localization:=true`, also map_server + particle_filter on `map`.
- mocap:   f1tenth_stack as above. The pose comes from /mocap_odom, streamed
           over the network from `mocap_map_launch.py` running on the pit PC
           (same ROS_DOMAIN_ID). This launch only ever receives that pose; it
           never starts the OptiTrack driver or mocap_map_node itself -- see
           the car/pit rule in `mrs_roboracer_bringup/README.md`.

`launch_hardware:=false` skips f1tenth_stack if it already runs on the car.
Packages are looked up only for the selected mode, so the sim machine does not
need f1tenth_stack and the car does not need f1tenth_gym_ros.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from mrs_roboracer_bringup.modes import check_mode, MODE_DESCRIPTION


def _include(package, launch_file, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=launch_arguments.items(),
    )


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _platform(context):
    mode = check_mode(LaunchConfiguration('mode').perform(context))
    actions = []

    if mode == 'sim':
        actions.append(_include('f1tenth_gym_ros', 'gym_bridge_launch.py'))
        return actions

    if _is_true(context, 'launch_hardware'):
        actions.append(_include('f1tenth_stack', 'bringup_launch.py'))

    if mode == 'onboard' and _is_true(context, 'localization'):
        map_yaml = LaunchConfiguration('map').perform(context)
        if not map_yaml:
            raise RuntimeError(
                'this controller needs a pose: onboard mode requires map:=<map .yaml> '
                'for particle_filter (e.g. a mocap_map_node export in ~/mocap_maps/)')
        actions.append(_include('mrs_roboracer_bringup', 'localization_launch.py',
                                map=map_yaml))

    if mode == 'mocap':
        actions.append(LogInfo(msg=(
            'mode:=mocap expects /mocap_odom from the pit PC: '
            'ros2 launch mrs_roboracer_bringup mocap_map_launch.py '
            'car_rigid_body_name:=<streaming ID>')))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', description=MODE_DESCRIPTION),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='onboard only: run map_server + particle_filter for a map-frame pose.'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='onboard + localization: map .yaml for map_server/particle_filter.'),
        DeclareLaunchArgument(
            'launch_hardware', default_value='true',
            description='onboard/mocap: start f1tenth_stack bringup (VESC, lidar, joy, mux).'),
        OpaqueFunction(function=_platform),
    ])
