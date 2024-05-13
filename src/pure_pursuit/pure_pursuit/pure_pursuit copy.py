#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO CHECK: include needed ROS msg type headers and libraries
from os.path import expanduser
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped, Pose, PointStamped, PoseArray
import math
import tf_transformations
from std_msgs.msg import String, Float64


home = expanduser('~')

KP = 0.5
SLOW_VEL = 0.6
class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        pose_topic = 'ego_racecar/odom'
        drive_topic = '/pure_pursuit_drive'
        command_topic = '/behavior_command'

        self.declare_parameters(
            namespace="",
            parameters=[
                ("laser_topic", "/scan"),
                ("odom_topic", "/ego_racecar/odom"),
                ("map_topic", "/occupancy_grid"),
                ("drive_topic", "/drive"),
                ("waypoint_viz_topic", "/waypoint_array"),
                ("goalpoint_viz_topic", "/waypoint_array"),
                ("file_name", "lab_today_waypoints.csv"),
                ("K_P", 0.6),
                ("K_D", -0.1),

                ("lookahead_distance", 0.4),
                ("speed", 1.0),
                ("new_path_topic", "/new_path_pose"),
                ("behavior_topic", "/behavior_state"),
                ("max_acceleration", 2.5),
                ("minimum_turning_radius", 0.861)
            ],
        )


        self.base_waypoints = np.genfromtxt('/sim_ws/wps/'+str(self.get_parameter('file_name').get_parameter_value().string_value), delimiter=',')

        self.waypoints = self.base_waypoints   

        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_array', 5)
        self.pose_sub = self.create_subscription(Odometry,pose_topic, self.pose_callback, 10)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_point', 5)
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.get_parameter('drive_topic').value, 10)
        self.path_pose_publisher = self.create_publisher(PoseArray, '/new_path_pose', 10)
        # TODO: create ROS subscribers and publishers
        self.behavior_pub = self.create_publisher(String, command_topic, 10)
        self.behavior_pub = self.create_publisher(Float64, )
        self.init_markers(self.waypoints)        
        self.prev_curvature = 0.0
   
    def init_markers(self, data):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        for e in data:
            pose = Pose()
            
            pose.position.x = e[0]
            pose.position.y = e[1]
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
            pose_array.poses.append(pose)
            self.path_pose_publisher.publish(pose_array)
        
    def brake(self):
        ackermann_message = AckermannDriveStamped()
        ackermann_message.drive.steering_angle = 0.0
        ackermann_message.drive.speed = 0.0
        self.drive_pub.publish(ackermann_message)


    def find_closest_waypoint_index(self, pose_msg):
        best_distance = math.inf
        best_index = 0

        for index,element in enumerate(self.waypoints):
            distance = self.calc_dist(pose_msg, element)
            if(distance<best_distance):
                best_distance = distance
                best_index=index
        return best_index

    def get_velocity(self, steering_angle):
        return float(self.get_parameter('speed').get_parameter_value().double_value)
    def calc_dist(self, pose_msg, element):
        return math.sqrt(math.pow((pose_msg.pose.pose.position.x-element[0]),2)+math.pow((pose_msg.pose.pose.position.y-element[1]),2))

    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture
        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                               pose_msg.pose.pose.orientation.y,
                               pose_msg.pose.pose.orientation.z,
                               pose_msg.pose.pose.orientation.w])
        euler = tf_transformations.euler_from_quaternion(quaternion)
        position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y,pose_msg.pose.pose.position.z]) 

        
        # TODO: transform goal point to vehicle frame of reference
        #find closest waypoint 
        # move further away from the waypoint until the distance is too much
        closest_index = self.find_closest_waypoint_index(pose_msg)
        idx = closest_index
        if idx == len(self.waypoints)-1:
            self.waypoints = self.base_waypoints
            command = String()
            command.data = "RESET"
            self.behavior_pub.publish(command)
            
        if (not math.isnan(idx))  and idx < len(self.waypoints):
            pt = self.waypoints[idx]
            dist = self.calc_dist(pose_msg, pt)
            while dist < self.lookahead_distance:
                idx+=1
                if(idx >= len(self.waypoints)):
                    idx=0
                pt=self.waypoints[idx]
                dist = self.calc_dist(pose_msg, pt)


            
            self.publish_waypoint(pt)

            
            # TODO: calculate curvature/steering angle
            
            lookahead_angle = math.atan2(pt[1] - position[1], pt[0] - position[0])
            #print(lookahead_angle)
            heading = tf_transformations.euler_from_quaternion(quaternion)[2]
            del_y = dist * math.sin(lookahead_angle - heading)
            curvature = 2.0*del_y/(math.pow(dist, 2))

            steering_angle = float(self.get_parameter('K_P').get_parameter_value().double_value)*curvature + float(self.get_parameter('K_D').get_parameter_value().double_value) * (curvature-self.prev_curvature) 
            self.prev_curvature = curvature
            ackermann_message = AckermannDriveStamped()
            ackermann_message.drive.steering_angle = steering_angle
            ackermann_message.drive.speed = self.get_velocity(steering_angle)
            self.drive_pub.publish(ackermann_message)
            print(ackermann_message)
        else:
            # print("No waypoints found")
            self.waypoints = self.base_waypoints
            command = String()
            command.data = "RESET"
            self.behavior_pub.publish(command)
    def publish_waypoint(self,pt):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.id = 0
        marker.lifetime = rclpy.duration.Duration(seconds=10).to_msg()
        marker.pose.position.x = pt[0]
        marker.pose.position.y = pt[1]
        marker.pose.position.z = 0.00
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.header.frame_id = 'map'
        marker.color.r = 2.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.goal_marker_pub.publish(marker)
        
def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()