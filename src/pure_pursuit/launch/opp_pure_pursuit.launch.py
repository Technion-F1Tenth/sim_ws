from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_pp =[
            {"laser_topic": "/opp_scan"},
            {"odom_topic": "/opp_racecar/odom"},
            {"map_topic": "/occupancy_grid"},
            {"drive_topic": "/opp_drive"},
            {"waypoint_viz_topic": "/waypoint_array"},
            {"goalpoint_viz_topic": "/waypoint_array"},
            {"file_name": "points_map_in_the_lab_cleaned.csv"},
            {"K_P": 0.2},
            {"lookahead_distance": 0.45},
            {"base_velocity": 1.0},
    ]
    print("Starting Pure Pursuit node with the following parameters")
    for element in params_pp:
        for i in element:
            print("\t",i,":", element[i])
    ld = LaunchDescription()
    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        parameters=params_pp
    )
    
    ld.add_action(pure_pursuit)
    return ld