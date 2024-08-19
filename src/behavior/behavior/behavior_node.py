import rclpy
from rclpy.node import Node

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
import sys

sys.path.insert(0, "/sim_ws/src/")
from common import Behavior


class BehaviorNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("file_name", "Spiegel_new_cleaned.csv"),
                ("reactive_node_drive_topic", "/reactive_drive"),
                ("pure_pursuit_node_drive_topic", "/pure_pursuit_drive"),
                ("drive_topic", "/drive"),
                ("command_topic", "/behavior_command"),
                ("goal_topic", "/goal_pose"),
                ("laserscan_topic", "/scan"),
                ("occupancy_grid_topic", "/occupancy_grid"),
                ("poses_topic", "/new_path_pose"),
                ("obstacles_marker_topic", "/obstacles_marker"),
                ("odometry_topic", "/ego_racecar/odom"),
                ("offset_points", 3),
                ("ttc_threshold", 0.1),
                ("state_topic", "/behavior_state"), 
                ("timeout_threshold_ns", 1 * 10**9),
                ],
        )
        self.get_logger().info("Initializing safety behavior node")
        self.file_name = (
            self.get_parameter("file_name").get_parameter_value().string_value
        )

        self.drive_topic = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )
        self.command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        self.goal_topic = (
            self.get_parameter("goal_topic").get_parameter_value().string_value
        )
        self.occupancy_grid_topic = (
            self.get_parameter("occupancy_grid_topic")
            .get_parameter_value()
            .string_value
        )
        self.poses_topic = (
            self.get_parameter("poses_topic").get_parameter_value().string_value
        )
        self.obstacles_marker_topic = (
            self.get_parameter("obstacles_marker_topic")
            .get_parameter_value()
            .string_value
        )
        self.odometry_topic = (
            self.get_parameter("odometry_topic").get_parameter_value().string_value
        )
        self.offset_points = (
            self.get_parameter("offset_points").get_parameter_value().integer_value
        )
        self.laserscan_topic = (
            self.get_parameter("laserscan_topic").get_parameter_value().string_value
        )
        
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, self.drive_topic, self.drive_callback, 10
        )

        # Declare pubsubs
        self.laserscan_sub = self.create_subscription(
            LaserScan, self.laserscan_topic, self.laser_scan_callback, 10
        )
        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, self.odometry_topic, self.odometry_callback, 10
        )
        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.create_subscription(
            OccupancyGrid, self.occupancy_grid_topic, self.occupancy_grid_callback, 10
        )
        self.create_subscription(PoseArray, self.poses_topic, self.poses_callback, 10)
        self.obstacle_marker_pub = self.create_publisher(
            MarkerArray, self.obstacles_marker_topic, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )

        self.behavior_state_pub = self.create_publisher(
            String,
            self.get_parameter("state_topic").get_parameter_value().string_value,
            10,
        )

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.original_waypoints = np.genfromtxt(
            "wps/" + self.get_parameter("file_name").get_parameter_value().string_value,
            delimiter=",",
        )
        self.waypoints = copy.deepcopy(self.original_waypoints)
        self.occupancy_grid = None
        self.car_pose = None
        self.laser_scan = None
        self.ttc_threshold = (
            self.get_parameter("ttc_threshold").get_parameter_value().double_value
        )
        self.only_reactive = False
        self.speed = 0.0
        self.behavior = Behavior.PURE_PURSUIT
        self.active_behavior = Behavior.PURE_PURSUIT

        self.last_drive_msg_time = self.get_clock().now()
        self.occupancy_grid_content = np.array([])

        self.get_logger().info("Safety Behavior Node Initialized")
        

    def command_callback(self, msg):
        if msg.data == Behavior.ONLY_REACTIVE.name:
            self.only_reactive = not self.only_reactive # Toggle
            self.get_logger().info("Only Reactive: " + str(self.only_reactive))
        if msg.data == Behavior.SAFETY_STOP.name:
            self.behavior = Behavior.SAFETY_STOP
            self.safety_callback()

    '''def safety_check(self):
        self.get_logger().info("safety check callback")

        will_crash = False
        min_ttc = np.inf
        min_dist = min(self.laser_scan.ranges)
        ellapsed_time = self.get_clock().now().nanoseconds - self.last_drive_msg_time.nanoseconds
        for index, distance in enumerate(self.laser_scan.ranges):
            angle = self.laser_scan.angle_min + index * self.laser_scan.angle_increment
            projected_vel = self.speed * math.cos(angle)  # Corrected projected velocity
            projected_vel = max(projected_vel, 0)
            if projected_vel != 0:
                time_to_crash = distance / projected_vel
                min_ttc = min(time_to_crash, min_ttc)
                if time_to_crash < self.ttc_threshold or distance < 0.05 or ellapsed_time > self.get_parameter("timeout_threshold_ns").get_parameter_value().integer_value:
                    will_crash = True
        self.get_logger().info("Safety Check")
        print("Will Crash: ", will_crash)
        # print(min_ttc, min_dist, ellapsed_time)


        return not will_crash'''

    def laser_scan_callback(self, msg):
        self.laser_scan = msg
        past_behavior = self.behavior


        self.get_logger().info(self.behavior.name)
        self.get_logger().info("Laser Scan Callback")

        if min(self.laser_scan.ranges) < 0.005:
            self.behavior = Behavior.SAFETY_STOP
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0
            self.drive_pub.publish(brake_msg)
            self.safety_callback()
            self.get_logger().info("Safety Stop")
            return
        else:
            self.behavior = self.active_behavior

        # Average N points in the center of the laser scan
        ranges = self.laser_scan.ranges
        center = len(ranges) // 2
        N = 100
        avg_range = sum(ranges[center - N : center + N]) / len(ranges[center - N : center + N])
        ttc = avg_range / self.speed
        if ttc < self.ttc_threshold:
            self.behavior = Behavior.REACTIVE
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0
            self.drive_pub.publish(brake_msg)
            self.safety_callback()
            self.get_logger().info("Safety Stop")

        if past_behavior != self.behavior:
            behavior_msg = String()
            behavior_msg.data = self.behavior.name
            self.behavior_state_pub.publish(behavior_msg)



    def occupancy_grid_callback(self, msg):
        
        '''old_behavior = self.behavior

        if self.behavior == Behavior.SAFETY_STOP:
            return
        self.occupancy_grid = msg
        self.occupancy_grid_content = np.array(self.occupancy_grid.data).reshape(
            (msg.info.height, msg.info.width)
        )

        # Choose behavior
        is_safe = self.safety_check()
        if self.only_reactive:
            if is_safe:
                self.behavior = Behavior.REACTIVE
            else:
                self.behavior = Behavior.REACTIVE

        if is_safe:
            obstacles = self.check_obstacles_in_grid()
            if len(obstacles) > 0:
                self.get_logger().info(str(obstacles))
                # Switch to reactive node / spline planning
                self.behavior = Behavior.REACTIVE  # self.behavior = Behavior.SPLINE
                self.switch_time = self.get_clock().now().nanoseconds
            
            else:
                # self.get_logger().info("No Obstacles Detected")
                self.behavior = Behavior.PURE_PURSUIT
        else:
            self.get_logger().info("Not safe to drive")
            self.behavior = Behavior.REACTIVE
        # Execute behavior
        if self.behavior != old_behavior and self.get_clock().now().nanoseconds - self.switch_time > pow(10,9) * 0.5:
            behavior_msg = String()
            behavior_msg.data = self.behavior.name
            self.behavior_state_pub.publish(behavior_msg)
            self.get_logger().info(f"Occupancy Grid Callback - Currrent behavior {self.behavior.name}")'''

    def safety_callback(self):
        if self.behavior == Behavior.STOP:
            self.get_logger().info("Not safe to drive")
            brake_msg = AckermannDriveStamped()
            self.drive_pub.publish(brake_msg)
        

    def poses_callback(self, msg):
        self.poses = msg

    def odometry_callback(self, msg):
        self.car_pose = msg.pose.pose
        self.speed = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

    def drive_callback(self, msg):
        self.last_drive_msg_time = self.get_clock().now()


    def get_obstacle_marker(self, obstacle_points):
        pass

    def get_goal(self, current_pose, path):
        pass

    def get_closest_point(self, current_pose, path):
        pass

    """
    Returns the obstructed spots as indexes of the local planning occupancy grid 
    """

    def get_closest_point(self):
        # return index of closest point 
        closest_idx = min(range(len(self.waypoints)), key=lambda i: self.dist_points(self.waypoints[i]))
        return closest_idx 

    def dist_points(self, waypoint):        
        return math.sqrt(
            (self.car_pose.position.x - waypoint[0])**2
            + (self.car_pose.position.y - waypoint[1])**2
        )
            

    def check_obstacles_in_grid(self):
        obstacles = []
        print("Checking obstacles")
        closest_index = self.get_closest_point()
        # define a safety radius in meters 
        safety_radius = 1.0
        # get the closest point to the car
        frame = "ego_racecar/laser"
        transform = None
        if self.tfBuffer.can_transform(frame, "map", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)):
            transform = self.tfBuffer.lookup_transform(frame, "map", rclpy.time.Time())
            i = closest_index
            (upper_bound, lower_bound) = (closest_index,closest_index)

            while self.dist_points(self.waypoints[i]) < safety_radius:
            # check if the point is in the occupancy grid
                upper_bound = i
                i += 1
                if i >= len(self.waypoints):
                    break
        
            i = closest_index 

            while self.dist_points(self.waypoints[i]) < safety_radius:
                # check if the point is in the occupancy grid
                lower_bound = i
                i -= 1
                if i < 0:
                    i = len(self.waypoints) - 1

            for i in range(lower_bound, upper_bound):
                # transform the waypoint 
                pose = Pose()
                pose.position.x = self.waypoints[i][0]
                pose.position.y = self.waypoints[i][1]
                pose.position.z = 0.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0
                pose_car_frame = tf2_geometry_msgs.do_transform_pose(pose, transform)

                # check if the point is in the occupancy grid
                x = self.waypoints[i][0]
                y = self.waypoints[i][1]
                y_grid = (
                    int(pose_car_frame.position.y / self.occupancy_grid.info.resolution)
                    + self.occupancy_grid.info.height // 2
                )
                x_grid = int(
                    pose_car_frame.position.x / self.occupancy_grid.info.resolution
                )

                if (
                    x_grid >= 0
                    and x_grid < self.occupancy_grid.info.width
                    and y_grid >= 0
                    and y_grid < self.occupancy_grid.info.height
                ):
                    # Check if it is occupied
                    if self.occupancy_grid_content[y_grid][x_grid] > 0 or self.occupancy_grid_content[y_grid][x_grid]<0 :
                        obstacles.append((x, y))

        else:
            self.get_logger().error("Transform not available")
        return obstacles

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
