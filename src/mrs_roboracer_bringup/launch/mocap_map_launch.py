"""
Mocap map launch: mocap_map_node (OptiTrack markers -> web editor -> OccupancyGrid).

`mocap4r2_optitrack_driver` (run separately; must be lifecycle-activated
first, see mrs_roboracer_sysid README §6) publishes `markers`
(mocap4r2_msgs/Markers) and `rigid_bodies` (mocap4r2_msgs/RigidBodies).
`mocap_map_node` serves the editor on http://<host>:<web_port>/, publishes
the map on /mocap_map (nav_msgs/OccupancyGrid, transient_local) plus
/mocap_map/obstacles (MarkerArray), the car as /mocap_odom (nav_msgs/Odometry),
the scan relayed as /mocap_scan, and the TF subtree
map -> mocap_base_link -> mocap_laser (parallel to the car's odom -> base_link).

No controller or safety node is started: this launch only builds/serves the map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('mrs_mocap_mapping')
    default_params_file = os.path.join(pkg_share, 'config', 'mocap_map_params.yaml')

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Path to mocap_map_params.yaml.'),
        DeclareLaunchArgument(
            'car_rigid_body_name',
            description='Rigid body name (as configured in Motive) for the car. Required.'),
        DeclareLaunchArgument(
            'up_axis', default_value='z',
            description="Up axis of the mocap stream: 'z' or 'y' (Motive default is Y-up)."),
        DeclareLaunchArgument(
            'markers_topic', default_value='/markers',
            description='mocap4r2_msgs/Markers topic published by mocap4r2_optitrack_driver.'),
        DeclareLaunchArgument(
            'rigid_bodies_topic', default_value='/rigid_bodies',
            description='mocap4r2_msgs/RigidBodies topic published by mocap4r2_optitrack_driver.'),
        DeclareLaunchArgument(
            'map_file', default_value='~/mocap_maps/arena.json',
            description='Obstacle/grid JSON (loaded if it exists; Save writes it + .pgm/.yaml).'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Car LaserScan relayed as /mocap_scan in the mocap_laser frame.'),
        DeclareLaunchArgument(
            'web_port', default_value='8080',
            description='Port of the map editor web UI.'),
    ]

    mocap_map_node = Node(
        package='mrs_mocap_mapping',
        executable='mocap_map_node',
        name='mocap_map',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'car_rigid_body_name': ParameterValue(LaunchConfiguration('car_rigid_body_name'),
                                                  value_type=str),
            'up_axis': LaunchConfiguration('up_axis'),
            'markers_topic': LaunchConfiguration('markers_topic'),
            'rigid_bodies_topic': LaunchConfiguration('rigid_bodies_topic'),
            'map_file': LaunchConfiguration('map_file'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'web.port': LaunchConfiguration('web_port'),
        }],
    )

    for action in args:
        ld.add_action(action)
    ld.add_action(mocap_map_node)

    return ld
