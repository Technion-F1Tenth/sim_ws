from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 
def generate_launch_description():
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('driving'),
        'config',
        'params.yaml'
        )
    reactive_node = Node(
        package='driving',
        executable='reactive_node',
        name='reactive_node',
        parameters=[config]
    )

    behavior_node = Node(
        package='behavior',
        executable='behavior_node',
        name='behavior_node',
    )
    

    ld.add_action(reactive_node)
    ld.add_action(behavior_node)

    return ld