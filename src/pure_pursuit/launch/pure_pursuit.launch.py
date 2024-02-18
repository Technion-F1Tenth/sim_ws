from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pure_pursuit_node_config =[
            {'laser_topic': '/scan'},
            {'odom_topic': '/ego_racecar/odom'},
            {'map_topic': '/occupancy_grid'},
            {'drive_topic': '/reactive_drive'},
            {'waypoint_viz_topic': '/waypoint_array'},
            {'goalpoint_viz_topic': '/waypoint_array'},
            {'file_name': 'points_map_in_the_lab_cleaned.csv'},
            {'K_P': 0.2},
            {'lookahead_distance': 0.45},
            {'base_velocity': 1.0},
        ]
    print('Starting Pure Pursuit node with the following parameters')
    for element in pure_pursuit_node_config:
        for i in element:
            print('\t',i,':', element[i])


    behavior_node_config = [
        {'file_name': 'points_map_in_the_lab_cleaned.csv'},
        {'reactive_node_drive_topic': '/reactive_drive'},
        {'pure_pursuit_node_drive_topic':'/pure_pursuit_drive'},
        {'drive_topic': '/drive'},
        {'command_topic': '/behavior_command'},
        {'goal_topic': '/goal_pose'},
        {'occupancy_grid_topic': '/occupancy_grid'},
        {'poses_topic': '/new_path_pose'},
        {'obstacles_marker_topic': '/obstacles_marker'},
        {'odometry_topic':'/ego_racecar/odom'},
        {'offset_points': 3}
    ]

    occupancy_grid_node_config = [
        {"map_width": 30},
        {"map_height": 30},
        {"resolution": 0.1},
        {"laser_frame": "ego_racecar/laser"},
        {"laser_topic": "/scan"},
        {"odom_topic": "/ego_racecar/odom"},
        {"map_topic": "/occupancy_grid"},  
    ]

    occupancy_grid_node = Node(
        package="occupancy_grid",
        executable="occupancy_grid_node",
        name="occupancy_grid_node",
        parameters=occupancy_grid_node_config
    )

    behavior_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        parameters=behavior_node_config
    )
    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        parameters=pure_pursuit_node_config
    )

    ld = LaunchDescription()
    
    ld.add_action(pure_pursuit)
    ld.add_action(occupancy_grid_node)
    ld.add_action(behavior_node)
    return ld