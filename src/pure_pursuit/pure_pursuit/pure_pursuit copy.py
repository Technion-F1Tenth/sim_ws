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
from std_msgs.msg import String

home = expanduser("~")

default_parameters = [
    ("laser_topic", "/scan"),
    ("odom_topic", "/ego_racecar/odom"),
    ("map_topic", "/occupancy_grid"),
    ("file_name", "spielberg"),
]

KP = 0.5
SLOW_VEL = 0.6
MAX_ACC = 3
MIN_TURN_RAD = 0.3


class PurePursuit(Node):
    def __init__(self):
        super().__init__("f110_pure_pursuit_node")
        print("new node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("laser_topic", "/scan"),
                ("odom_topic", "/ego_racecar/odom"),
                ("map_topic", "/occupancy_grid"),
                ("file_name", "spk"),
                ("K_P", 1.0),
                ("lookahead_distance", 3.0),
                ("base_velocity", 2.0),
            ],
        )
        print("Parameters in use")
        for param in default_parameters:
            print(param)
        self.pose_sub = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 10
        )
        self.waypoints = np.genfromtxt("/sim_ws/wps/spielberg.csv", delimiter=",")
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.waypoints_viz_pub = self.create_publisher(
            MarkerArray, "/waypoint_array", 10
        )
        self.goal_marker_pub = self.create_publisher(Marker, "/goal_point", 10)
        self.init_markers(self.waypoints)

        self.current_velocity

        self.lookahead_distance_initialised = False

        
        #define rectangular segments (xmin, xmax, ymin, ymax) and categorize them ("striaght", "almost corner", "corner")
        self.segments = [
            ((0, 3, 0, 3), "corner"),  # Segment 1
            ((3, 6, 2, 3), "almost corner"),  # Segment 2
            ((6, 8, 2, 5), "straight"),  # Segment 3
            ((4, 8, 5, 7), "almost corner"),  # Segment 4
            ((1, 4, 5, 7), "straight"),  # Segment 5
            ((0, 1, 0, 5), "corner")   # Segment 6
        ] #for now these are arbitrary examples
        
    
    def calculate_lookahead_distance(self, pose_msg):
        if not self.lookahead_distance_initialised:
            self.lookahead_distance_initialised = True
            return 3
        velocity = self.current_velocity
        A = 1/(2*MAX_ACC)
        B = self.lookahead_context(pose_msg)
        C = MIN_TURN_RAD
        D = 2
        lookahead = A*velocity^2 + B*velocity + C - D*self.distance_to_closest_waypoint(pose_msg)
        return max(lookahead, 0.5)

    def lookahead_context(self, pose_msg):
        segment_type = self.find_segment_type(pose_msg)
        if(segment_type == "corner"):
            return 0.2
        if(segment_type == "almost corner"):
            return 0.5
        if(segment_type == "straight"):
            return 1
    
    def find_segment_type(self, pose_msg):
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        for segment in enumerate(self.segments):
            xmin, xmax, ymin, ymax = segment[0]
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return segment[1]  # Return segment type
            
    def distance_to_closest_waypoint(self, pose_msg):
        best_distance = math.inf
        for index,element in enumerate(self.waypoints):
            distance = self.calc_dist(pose_msg, element)
            if(distance<best_distance):
                best_distance = distance
        return best_distance


    def calc_dist(self, pose_msg, element):
        return math.sqrt(
            math.pow((pose_msg.pose.pose.position.x - element[0]), 2)
            + math.pow((pose_msg.pose.pose.position.y - element[1]), 2)
        )

    def get_closest_point(self, pose_msg):
        best_distance = math.inf
        best_index = 0
        best_element = None

        for index, element in enumerate(self.waypoints):
            distance = self.calc_dist(pose_msg, element)
            if distance < best_distance:
                best_distance = distance
                best_index = index
                best_element = element
        return (best_index, best_element)

    def get_velocity(self, steering_angle):
        return self.get_parameter("base_velocity").get_parameter_value().double_value

    def odom_callback(self, pose_msg):
        lookahead_distance = self.calculate_lookahead_distance(pose_msg)
        print(pose_msg)
        quaternion = np.array(
            [
                pose_msg.pose.pose.orientation.x,
                pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z,
                pose_msg.pose.pose.orientation.w,
            ]
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        position = np.array(
            [
                pose_msg.pose.pose.position.x,
                pose_msg.pose.pose.position.y,
                pose_msg.pose.pose.position.z,
            ]
        )
        (closest_idx, closest_point) = self.get_closest_point(pose_msg)
        idx = closest_idx
        if not math.isnan(idx):
            pt = closest_point
            dist = self.calc_dist(pose_msg, pt)
            while (
                dist
                < self.get_parameter("lookahead_distance")
                .get_parameter_value()
                .double_value
            ):
                idx = (idx + 1) % len(self.waypoints)
                pt = self.waypoints[idx]
                dist = self.calc_dist(pose_msg, pt)
        self.publish_waypoint(pt)
        lookahead_angle = math.atan2(pt[1] - position[1], pt[0] - position[0])
        # print(lookahead_angle)
        heading = tf_transformations.euler_from_quaternion(quaternion)[2]
        del_y = dist * math.sin(lookahead_angle - heading)
        curvature = 2.0 * del_y / (math.pow(dist, 2))
        steering_angle = (
            float(self.get_parameter("K_P").get_parameter_value().double_value)
            * curvature
        )

        ackermann_message = AckermannDriveStamped()
        ackermann_message.drive.steering_angle = steering_angle
        speed = self.get_velocity(steering_angle)
        ackermann_message.drive.speed = speed
        self.drive_pub.publish(ackermann_message)
        self.current_velocity = speed

    def publish_waypoint(self, pt):
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
        marker.header.frame_id = "map"
        marker.color.r = 2.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.goal_marker_pub.publish(marker)

    def init_markers(self, waypoints):
        marker_array = MarkerArray()

        def enumerate2(xs, start=0, step=1):
            for x in xs:
                yield (start, x)
                start += step

        for idx, e in enumerate2(waypoints, start=0, step=5):
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.id = idx
            marker.lifetime = rclpy.duration.Duration(seconds=10).to_msg()
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1]
            marker.pose.position.z = 0.00
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 0.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.header.frame_id = "map"
            marker.color.r = 2.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        self.waypoints_viz_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
