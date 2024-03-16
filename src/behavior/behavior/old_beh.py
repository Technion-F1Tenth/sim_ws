#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from os.path import expanduser
import copy

BRAKE_MESSAGE = AckermannDriveStamped()
BRAKE_MESSAGE.drive.speed = 0.0

home = expanduser('~')

class Behavior(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('file_name', 'Spielberg_clean.csv'),
                ('reactive_node_drive_topic', '/reactive_drive'),
                ('pure_pursuit_node_drive_topic', '/pure_pursuit_drive'),
                ('drive_topic', '/drive'),
                ('command_topic', '/behavior_command'),
                ('goal_topic', '/goal_pose'),
                ('occupancy_grid_topic', '/occupancy_grid'),
                ('poses_topic', '/new_path_pose'),
                ('obstacles_marker_topic', '/obstacles_marker'),
                ('odometry_topic', '/ego_racecar/odom'),
                ('offset_points', 3)

            ]
        )

        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        reactive_node_drive_topic = self.get_parameter('reactive_node_drive_topic').get_parameter_value().string_value
        pure_pursuit_node_drive_topic = self.get_parameter('pure_pursuit_node_drive_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.occupancy_grid_topic= self.get_parameter('occupancy_grid_topic').get_parameter_value().string_value
        self.poses_topic = self.get_parameter('poses_topic').get_parameter_value().string_value
        self.obstacles_marker_topic = self.get_parameter('obstacles_marker_topic').get_parameter_value().string_value

        self.offset_points = self.get_parameter('offset_points').get_parameter_value().integer_value
        self.occupancy_grid = None
        self.poses = None
        self.odometry = None
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.odometry_sub = self.create_subscription(Odometry, self.odometry_topic, self.odometry_callback, 10)
        self.create_subscription(AckermannDriveStamped, reactive_node_drive_topic, self.drive_callback,10)
        self.create_subscription(AckermannDriveStamped, pure_pursuit_node_drive_topic, self.drive_callback,10)
        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.create_subscription(OccupancyGrid, self.occupancy_grid_topic, self.occupancy_grid_callback, 10)
        self.create_subscription(PoseArray, self.poses_topic, self.poses_callback, 10)
        self.obstacle_marker_pub = self.create_publisher(MarkerArray, self.obstacles_marker_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.drive_message = BRAKE_MESSAGE
        
        self.state = "PURE_PURSUIT"
        self.curr_goal = None

        self.init_poses()
        self.base_poses = copy.deepcopy(self.poses)
        self.get_logger().info('Behavior Initialized')
        # TODO: Publish to drive
    def odometry_callback(self, message):
        self.odometry = message
    def init_poses(self):
        base_waypoints = np.genfromtxt('/sim_ws/wps/'+str(self.get_parameter('file_name').get_parameter_value().string_value), delimiter=',')
        # Get the poses from the base waypoints
        self.poses = []
        for waypoint in base_waypoints:
            pose = Pose()
            pose.position.x = waypoint[0]
            pose.position.y = waypoint[1]
            pose.position.z = 0.0
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            self.poses.append(pose)
        self.get_logger().info('Poses Initialized')
        self.get_logger().info(str(len(self.poses))+"self.poses")

    

    def poses_callback(self, message):
        self.get_logger().info('New Poses Received')

        self.poses = message.poses

    def drive_callback(self, message):
        if self.state == "REACTIVE":
            self.drive_message = message
        if self.state == "BRAKE":
            self.drive_message = BRAKE_MESSAGE
        if self.state == "PURE_PURSUIT":
            self.drive_message = message
        self.drive_pub.publish(self.drive_message)

    def command_callback(self, message):
        command = message.data
        print(command)
        if command=="REACTIVE":
            self.state="REACTIVE"
        elif command=="BRAKE":
            self.state="BRAKE"
            self.drive_message = BRAKE_MESSAGE
        elif command=="PURE_PURSUIT":
            self.state="PURE_PURSUIT"
        elif command=="RESET":
            self.state="PURE_PURSUIT"
            self.poses = copy.deepcopy(self.base_poses)
            self.get_logger().info('Reset Poses')
            self.curr_goal = None

    
    def occupancy_grid_callback(self, message):
        if self.poses is not None and self.odometry is not None:
            self.occupancy_grid = message.data
            self.occupancy_grid = np.array(self.occupancy_grid).reshape((message.info.height, message.info.width))
            try: 
                transform = self.tfBuffer.lookup_transform('ego_racecar/laser', 'map', rclpy.time.Time())
                # Transform poses position to local map 

                obstacles = MarkerArray()
                obstacles.markers = []
                
                self.get_logger().info(str(len(self.poses))+"self.poses")
                for index_pose, pose in enumerate(self.poses):
                    # elf.get_logger().info('Checking Obstacles')

                    # Use transform to get the pose in the map frame
                    pose_car_frame = tf2_geometry_msgs.do_transform_pose(pose, transform)
                    # Get the pose in the map frame
                    '''
                    # Obstacle in discretized coordinate frame.
                    y_grid = int(y/self.resolution) + self.height//2 # x coordinate in grid frame
                    x_grid = int(x/self.resolution) # y coordinate in grid frame
                    '''
                    
                    # Map pose to occupancy grid 
                    y_grid = int(pose_car_frame.position.y/message.info.resolution) + message.info.height//2
                    x_grid = int(pose_car_frame.position.x/message.info.resolution) 
                    
                    # Check if the pose is in the map
                    if x_grid >= 0 and x_grid < message.info.width and y_grid >= 0 and y_grid < message.info.height:
                        # Check if it is occupied 

                        if self.occupancy_grid[y_grid][x_grid] == 100:
                            
                            # Create a marker for the obstacle
                            obstacle_marker = Marker()

                            obstacle_marker.header.frame_id = "map"
                            # Add a stamp time 
                            obstacle_marker.header.stamp = rclpy.time.Time().to_msg() 
                            # duration
                            obstacle_marker.lifetime = rclpy.time.Duration(seconds=1.0).to_msg()
                            obstacle_marker.id = index_pose
                            obstacle_marker.type = Marker.CUBE
                            obstacle_marker.action = Marker.ADD
                            obstacle_marker.pose.position.x = pose.position.x
                            obstacle_marker.pose.position.y = pose.position.y
                            obstacle_marker.pose.position.z = pose.position.z
                            obstacle_marker.pose.orientation.x = pose.orientation.x
                            obstacle_marker.pose.orientation.y = pose.orientation.y
                            obstacle_marker.pose.orientation.z = pose.orientation.z
                            obstacle_marker.pose.orientation.w = pose.orientation.w
                            obstacle_marker.scale.x = 0.1
                            obstacle_marker.scale.y = 0.1
                            obstacle_marker.scale.z = 0.1
                            obstacle_marker.color.a = 1.0
                            obstacle_marker.color.r = 0.0
                            obstacle_marker.color.g = 0.0
                            obstacle_marker.color.b = 1.0
                            obstacles.markers.append(obstacle_marker)
                            # TODO: PLAN AROUND THE OBSTACLE
                            # Calculate the distance from the obstacle 
                            # Calculate the angle from the obstacle
                            distance = math.sqrt((pose.position.x-self.odometry.pose.pose.position.x)**2 + (pose.position.y-self.odometry.pose.pose.position.y)**2)
                            angle = math.atan2(pose.position.y-self.odometry.pose.pose.position.y, pose.position.x-self.odometry.pose.pose.position.x)
                            self.get_logger().info('Obstacle Detected' + str(pose.position))
                            self.get_logger().info('Distance to Obstacle' + str(distance))
                            if distance < 2.0:
                                # self.state = "BRAKE"
                                if self.curr_goal is None:
                                    idx_to_plan = (index_pose + self.offset_points) % (len(self.poses) -1)
                                    point_to_plan = self.poses[idx_to_plan]
                                    self.get_logger().info('Planning to point' + str(point_to_plan.position))
                                    goal = PoseStamped()
                                    goal.header.frame_id = "map"
                                    goal.header.stamp = rclpy.time.Time().to_msg()
                                    goal.pose = point_to_plan
                                    self.goal_pub.publish(goal)
                                    self.curr_goal = goal
                                    return 
                                else:
                                    self.get_logger().info('Planning to point' + str(self.curr_goal.pose.position))
                                    self.goal_pub.publish(self.curr_goal)
                                    return
                if not len(obstacles.markers) > 0:
                    self.get_logger().info('No Obstacles Detected')

                self.obstacle_marker_pub.publish(obstacles)

        

                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.get_logger().warn('No transform found')
        else:
            self.get_logger().warn('No poses found')

        
        

    
    
        
        


def main(args=None):
    rclpy.init(args=args)
    print("Behavior Initialized")
    behavior_node = Behavior()
    rclpy.spin(behavior_node)

    behavior_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    '''
    Obstacle Detected4.717662480537063, -3.039897385376894

    '''