#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO CHECK: include needed ROS msg type headers and libraries
from os.path import expanduser
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point, PoseStamped, Pose, PointStamped, PoseArray
import math
import tf_transformations
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
import matplotlib.pyplot as plt
home = expanduser('~')

from .spline_planner import CubicSplinePlanner  


class PlannerNode(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """


    def __init__(self):
        super().__init__('planner_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('laser_frame', 'ego_racecar/base_link'),
                ('local_frame', 'occupancy_grid'),
                ('global_frame', 'map'),
                ('odom_topic', '/ego_racecar/odom'),
                ('map_topic', '/occupancy_grid'),
                ('goal_topic', '/goal_pose'),
                ('M', 20),
                ('N', 1),
                ('INNER_POINTS', 20),
                ('MAX_ITER', 10),
            ]
        )    

        self.map = None
        self.odom = None
        self.iterations = 0
        self.waypoints = None
        

        # tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)


        # TODO: create ROS subscribers and publishers
        self.map_subscriber = self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value, self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value, self.goal_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/new_goal_pose', 10)
        self.path_publisher = self.create_publisher(MarkerArray, '/new_path', 10)
        self.path_pose_publisher = self.create_publisher(PoseArray, '/new_path_pose', 10)
        

        

    def map_callback(self, map_msg):
        self.map = map_msg


    def odom_callback(self, odom_msg):
        self.odom = odom_msg




    def goal_callback(self, goal_msg):
        if self.map is None:
            print("No Map")
            return
        self.goal = goal_msg
        

        try: 
            t = self.tfBuffer.lookup_transform(self.get_parameter('laser_frame').value, self.get_parameter('global_frame').value, rclpy.time.Time())
            print("Transform: ", t)
            
            # Transform to laser frame
            new_goal_msg = PoseStamped()
            new_goal_msg.header.frame_id = self.get_parameter('laser_frame').value
            new_goal_msg.header.stamp = rclpy.time.Time().to_msg()
            new_goal_msg.pose = tf2_geometry_msgs.do_transform_pose(goal_msg.pose, t)
            
            print("Transformed Goal Pose: ", new_goal_msg)
            
            self.goal_publisher.publish(new_goal_msg)

            point = new_goal_msg.pose.position
            x = point.x
            y = point.y
            print("x: ", x, "y: ", y)
            
            np_map = np.array(self.map.data)
            np_map = np_map.reshape(self.map.info.height, self.map.info.width)
            
            x_index = int((x)/self.map.info.resolution) 
            y_index = int((y)/self.map.info.resolution + self.map.info.height/2)
            goal_in_map = (y_index, x_index)
            origin_in_map = (int(self.map.info.height/2), 0)
            print(type(np_map))
            self.planner = CubicSplinePlanner(np_map, origin_in_map, goal_in_map, self.get_parameter('M').value, self.get_parameter('N').value, self.get_parameter('INNER_POINTS').value)
            try:
                interpolated_splines, costs_splines, best_spline = self.planner.interpolate_splines()
            except:
                self.iterations+=1
                self.get_logger().warn("No Clean Splines")

                
                self.goal_callback(goal_msg) # TODO: See if this is smart?
                return
            if len(interpolated_splines) == 0 and self.iterations < self.get_parameter('MAX_ITER').value:
                self.iterations+=1
                self.get_logger().warn("No Clean Splines")

                self.goal_callback(goal_msg) # TODO: See if this is smart?
                return
            # plot splines
            # self.planner.plot_splines()
            # plt.show()
            self.iterations = 0 


            # Get the best spline and make each point a marker 
            

            transform_new = self.tfBuffer.lookup_transform(self.get_parameter('global_frame').value, self.get_parameter('laser_frame').value, rclpy.time.Time())
            # Obtain the tanget at each point and use that to get the orientation of the marker
            pose_array = PoseArray()
            pose_array.header.frame_id = 'map'
            for e in best_spline:
                pose = Pose()
                marker_point = Point()
                marker_point.x = (e[1]*self.map.info.resolution) 
                marker_point.y = (e[0]*self.map.info.resolution - self.map.info.height/2*self.map.info.resolution) 
                marker_point.z = 0.0
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.get_parameter('laser_frame').value
                point_stamped.point = marker_point
                transformed_point =  tf2_geometry_msgs.do_transform_point(point_stamped, transform_new).point
                pose.position.x = transformed_point.x
                pose.position.y = transformed_point.y
                pose.position.z = 0.0
                pose.position.z = 0.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0
                
                pose_array.poses.append(pose)
            self.path_pose_publisher.publish(pose_array)
            self.get_logger().info("publsihed new path")



            




            
            
            # 

            


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Transform Error")
            return



    




    
        
def main(args=None):
    rclpy.init(args=args)
    print("Spline Planner Initialized")
    pure_pursuit_node = PlannerNode()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()