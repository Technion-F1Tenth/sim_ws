from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrs_roboracer_pp_cpp',                # ASSUMPTION: package provides the node
            executable='pure_pursuit_node',         # ASSUMPTION: entrypoint/executable name
            name='mrs_roboracer_pure_pursuit',
            output='screen',
            emulate_tty=True,
            parameters=[
                # Topics 
                {'odom_topic': '/pf/pose/odom'},
                {'drive_topic': '/drive'},
                # Vehicle parameters
                {'wheelbase': 0.28},

                # Control parameters
                {'lookahead_distance': 0.8},
                {'speed': 1.0},
                {'max_steer': 0.7853981633974483},  # 45 deg in radians

                # Path parameters & loading
                {'raceline_path': '/home/andy/mrs_ws/IMS_centerline_trajectory.csv'},
                {'init_path': False},
            ],
        ),
    ])
