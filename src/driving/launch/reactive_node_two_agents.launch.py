from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import math  # Import math module for math.pi

def generate_launch_description():
    
    ld = LaunchDescription()
    
    # Define parameters as a dictionary
    config_ego = {
        'bubble_radius': 0.4,
        'laser_topic': '/scan',
        'drive_topic': '/drive',
        'slow_mode': True,
        'max_speed': 7.0,
        'min_speed': 1.2,
        'max_range': 5.0,
        'car_width': 0.3,
        "state_topic": "/behavior_state",  # Duplicate removed
        "velocity_topic": "/follow_the_gap_velocity",
        "threshold_topic": "/follow_the_gap_threshold",
        "safety_factor": 2.50,
        "max_lidar_range": 30.0,
        "disparity_threshold": 0.05,
        "marker_topic": "/chosen_point",
        "angle_clip": 3/4 * math.pi,  # Use math.pi for PI
        "param_update": "/reactive/param_update",
    }

    config_opp = {
        'bubble_radius': 0.4,
        'laser_topic': '/opp_scan',
        'drive_topic': '/opp_drive',
        'slow_mode': True,
        'max_speed': 7.0,
        'min_speed': 1.2,
        'max_range': 5.0,
        'car_width': 0.3,
        "state_topic": "/behavior_state",  # Duplicate removed
        "velocity_topic": "/follow_the_gap_velocity",
        "threshold_topic": "/follow_the_gap_threshold",
        "safety_factor": 2.0,
        "max_lidar_range": 30.0,
        "disparity_threshold": 0.05,
        "marker_topic": "/chosen_point",
        "angle_clip": 3/4 * math.pi,  # Use math.pi for PI
        "param_update": "/reactive/param_update",
    }
    
    reactive_node_ego = Node(
        package='driving',
        executable='reactive_node',
        name='reactive_node',
        parameters=[config_ego]  # Pass the parameters dictionary
    )
    reactive_node_opp = Node(
        package='driving',
        executable='reactive_node',
        name='reactive_node',
        parameters=[config_opp]  # Pass the parameters dictionary
    )
    
    ld.add_action(reactive_node_ego)
    ld.add_action(reactive_node_opp)
    # ld.add_action(behavior_node)  # This line references an undefined variable

    return ld