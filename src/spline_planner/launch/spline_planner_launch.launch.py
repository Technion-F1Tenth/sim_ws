from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
def generate_launch_description():
    ld = LaunchDescription()
    planner_config = os.path.join(
        get_package_share_directory('spline_planner'),
        'config',
        'params.yaml'
        )
    
    spline_planner_node = Node(
        package="spline_planner",
        executable="planner_node",
        name='planner_node',
        parameters=[planner_config]
    )

    pure_pursuit_config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'params.yaml'
        )
    
    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit",
        name='pure_pursuit_node',
        parameters=[pure_pursuit_config]
    )
    occupancy_grid_config = os.path.join(
        get_package_share_directory('occupancy_grid'),
        'config',
        'params.yaml'
        )
    
    occupancy_grid_node = Node(
        package="occupancy_grid",
        executable="occupancy_grid_node",
        name='occupancy_grid_node',
        parameters=[occupancy_grid_config]
    )
    # Print occupancy grid node parameters using ROS2 info log
    # behavior node
    behavior_node = Node(
        package="behavior",
        executable="behavior_node",
        name='behavior_node',
    )
    
    ld.add_action(spline_planner_node)
    ld.add_action(pure_pursuit_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(behavior_node)



    return ld