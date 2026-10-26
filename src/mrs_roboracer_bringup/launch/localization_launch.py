"""
Onboard localization: nav2 map_server + f1tenth particle_filter on a given map.

Included by `platform_launch.py` for `mode:=onboard localization:=true`.
Upstream's `particle_filter/launch/localize_launch.py` is not used because it
hardcodes the map to one in its own share directory and sets
use_sim_time:=true.

particle_filter fetches the map once from the `/map_server/map` service,
subscribes to /scan, /odom and /initialpose (set the start pose with the
Foxglove/RViz pose-estimate tool), and publishes /pf/pose/odom. Note that it
broadcasts a `map -> laser` TF, which gives `laser` a second parent next to
f1tenth_stack's static `base_link -> laser`.

Requires the `particle_filter` package and range_libc (not in this workspace,
see https://github.com/f1tenth/particle_filter).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_pf_config = os.path.join(
        get_package_share_directory('mrs_roboracer_bringup'), 'config', 'particle_filter.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Map .yaml (nav2_map_server format). Required.'),
        DeclareLaunchArgument(
            'pf_config', default_value=default_pf_config,
            description='particle_filter parameters.'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'topic_name': 'map',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        Node(
            package='particle_filter',
            executable='particle_filter',
            name='particle_filter',
            output='screen',
            parameters=[LaunchConfiguration('pf_config')],
        ),
    ])
