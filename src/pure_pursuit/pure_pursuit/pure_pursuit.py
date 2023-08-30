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
import math
import tf_transformations


home = expanduser('~')

KP = 0.5
SLOW_VEL = 0.6
class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """


    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        pose_topic = 'ego_racecar/odom'
        drive_topic = '/drive'

        self.declare_parameter('speed', 0.4)
        self.declare_parameter('K_P', 0.5)
        
        self.declare_parameter('file_name', rclpy.Parameter.Type.STRING)

        self.waypoints = np.genfromtxt(home+'/sim_ws/wps/'+str(self.get_parameter('file_name').get_parameter_value().string_value), delimiter=',')
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_array', 5)
        self.pose_sub = self.create_subscription(Odometry,pose_topic, self.pose_callback, 10)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_point', 5)
        self.init_markers(self.waypoints)        
        self.lookahead_distance = 1.2
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: create ROS subscribers and publishers

    def init_markers(self, data):
        marker_count = 0
        marker_array_msg = MarkerArray()
        for e in data:
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.id = marker_count
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = e[2]
            marker.pose.orientation.y = e[3]
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 0.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.header.frame_id = 'map'
            marker.color.r = 0.0
            marker.color.g = 2.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array_msg.markers.append(marker) 

            
            marker_count += 1
        print(marker_count)
        self.marker_pub.publish(marker_array_msg)

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
        pt = self.waypoints[idx]
        dist = self.calc_dist(pose_msg, pt)
        while dist < self.lookahead_distance:

            idx+=1
            if(idx >= len(self.waypoints)):
                idx=0
            pt=self.waypoints[idx]
            dist = self.calc_dist(pose_msg, pt)
            
        print(pt)
        self.publish_waypoint(pt)

        
        # TODO: calculate curvature/steering angle
        
        lookahead_angle = math.atan2(pt[1] - position[1], pt[0] - position[0])
        #print(lookahead_angle)
        heading = tf_transformations.euler_from_quaternion(quaternion)[2]
        del_y = dist * math.sin(lookahead_angle - heading)
        curvature = 2.0*del_y/(math.pow(dist, 2))
        steering_angle = float(self.get_parameter('K_P').get_parameter_value().double_value)* curvature

        ackermann_message = AckermannDriveStamped()
        ackermann_message.drive.steering_angle = steering_angle
        ackermann_message.drive.speed = self.get_velocity(steering_angle)
        self.drive_pub.publish(ackermann_message)
        # TODO: publish drive message, don't forget to limit the steering angle.

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