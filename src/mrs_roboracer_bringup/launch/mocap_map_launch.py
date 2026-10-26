"""
Mocap map launch: mocap_map_node (OptiTrack markers -> web editor -> OccupancyGrid).

Starts the OptiTrack driver too (`optitrack_launch.py`, configured and
activated automatically; `launch_optitrack:=false` to skip it), which
publishes `markers` (mocap4r2_msgs/Markers) and `rigid_bodies`
(mocap4r2_msgs/RigidBodies).
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('mrs_mocap_mapping')
    default_params_file = os.path.join(pkg_share, 'config', 'mocap_map_params.yaml')

    driver_share = get_package_share_directory('mocap4r2_optitrack_driver')
    default_driver_config = os.path.join(
        driver_share, 'config', 'mocap4r2_optitrack_driver_params.yaml')

    launch_optitrack_arg = DeclareLaunchArgument(
        'launch_optitrack', default_value='true',
        description='Start the OptiTrack driver (auto-activated) via optitrack_launch.py. '
                    'Set false if it already runs elsewhere on this ROS_DOMAIN_ID.')
    driver_config_arg = DeclareLaunchArgument(
        'driver_config', default_value=default_driver_config,
        description='mocap4r2_optitrack_driver params (server/local address, multicast).')

    optitrack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mrs_roboracer_bringup'),
            'launch', 'optitrack_launch.py')),
        launch_arguments={'driver_config': LaunchConfiguration('driver_config')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_optitrack')),
    )

    args = [
        launch_optitrack_arg,
        driver_config_arg,
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Path to mocap_map_params.yaml.'),
        DeclareLaunchArgument(
            'car_rigid_body_name',
            description='Motive Streaming ID of the car rigid body, e.g. 4. The driver '
                        'publishes the numeric ID, not the asset name. Required.'),
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
    ld.add_action(optitrack)
    ld.add_action(mocap_map_node)

    return ld
