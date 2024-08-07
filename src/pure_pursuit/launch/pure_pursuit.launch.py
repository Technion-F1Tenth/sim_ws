from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pure_pursuit_node_config =[
                {"laser_topic": "/ego_racecar/scan"},
                {"odom_topic": "/ego_racecar/odom"},
                {"map_topic": "/occupancy_grid"},
                {"drive_topic": "/drive"},
                {"waypoint_viz_topic": "/waypoint_array"},
                {"goalpoint_viz_topic": "/waypoint_array"},
                {"file_name": "Spielberg_raceline.csv"},
                {"K_P": 0.3},
                {"K_D": 0.01},
                {"lookahead_distance": 0.1},
                {"base_velocity": 7.0},
                {"new_path_topic": "/new_path_pose"},
                {"behavior_topic":"/behavior_state"},
                {"max_acceleration": 2.5},
                {"minimum_turning_radius": 0.861},
                {"waypoint_viz_topic": "/waypoint_viz"},
            ]
    
    
    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        parameters=pure_pursuit_node_config
    )

    ld = LaunchDescription()
    
    ld.add_action(pure_pursuit)
    return ld