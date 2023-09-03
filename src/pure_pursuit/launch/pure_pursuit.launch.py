from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = [
            {'speed':0.4},
            {'K_P':0.3},
            {'lookahead_distance':0.3},
            {'file_name':'tame_courtyard.csv'}
    ]
    print("Starting Pure Pursuit node with the following parameters")
    for element in params:
        for i in element:
            print("\t",i,":", element[i])
    ld = LaunchDescription()
    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        parameters=params
    )
    ld.add_action(pure_pursuit)
    return ld