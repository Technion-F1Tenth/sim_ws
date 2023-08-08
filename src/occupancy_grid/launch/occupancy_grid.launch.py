from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('occupancy_grid'),
        'config',
        'params.yaml'
        )
    
    occupancy_grid_node = Node(
        package="occupancy_grid",
        executable="occupancy_grid_node",
        name='occupancy_grid_node',
        parameters=[config]
    )
    # Print occupancy grid node parameters using ROS2 info log





    ld.add_action(occupancy_grid_node)
    return ld