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
                ("file_name", "Spielberg_clean.csv"),
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
                ("ttc_threshold", 0.2),
                ("state_topic", "/behavior_state"), 
                ("timeout_threshold_ns", 0.2 * 10**9),
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
        self.only_reactive = True
        self.speed = 0.0
        self.behavior = None
        self.last_drive_msg_time = self.get_clock().now()
        

    def command_callback(self, msg):
        if msg.data == Behavior.ONLY_REACTIVE.name:
            self.only_reactive = not self.only_reactive # Toggle
            self.get_logger().info("Only Reactive: " + str(self.only_reactive))
        if msg.data == Behavior.SAFETY_STOP.name:
            self.behavior = Behavior.SAFETY_STOP
            self.safety_callback()

    def safety_check(self):
        will_crash = False
        min_ttc = np.inf
        min_dist = min(self.laser_scan.ranges)
        for index, distance in enumerate(self.laser_scan.ranges):
            angle = self.laser_scan.angle_min + index * self.laser_scan.angle_increment
            projected_vel = self.speed * math.cos(angle)  # Corrected projected velocity
            projected_vel = max(projected_vel, 0)
            if projected_vel != 0:
                time_to_crash = distance / projected_vel
                min_ttc = min(time_to_crash, min_ttc)
                if time_to_crash < self.ttc_threshold or distance < 0.2:
                    will_crash = True

        print(min_ttc, min_dist)

        return not will_crash

    def laser_scan_callback(self, msg):
        if self.behavior == Behavior.SAFETY_STOP:
            return
        self.laser_scan = msg
        # Choose behavior
        print("Laser Scan Callback")
        if self.only_reactive:
            is_safe = self.safety_check()
            if is_safe:
                self.behavior = Behavior.REACTIVE
            else:

                self.behavior = Behavior.STOP
            msg = String()
            msg.data = self.behavior.name
            self.behavior_state_pub.publish(msg)

        self.safety_callback()

    def occupancy_grid_callback(self, msg):
        if self.behavior == Behavior.SAFETY_STOP:
            return
        self.occupancy_grid = msg.data
        self.occupancy_grid = np.array(self.occupancy_grid).reshape(
            (self.occupancy_grid.info.height, self.occupancy_grid.info.width)
        )

        # Choose behavior
        is_safe = self.safety_check()
        if self.only_reactive:
            if is_safe:
                self.behavior = Behavior.REACTIVE
            else:
                self.behavior = Behavior.STOP

        if is_safe:
            obstacles = self.check_obstacles_in_grid()
            if len(obstacles) > 0:
                self.get_logger().info("Obstacle Detected")
                self.get_logger().info(str(obstacles))
                # Switch to reactive node / spline planning
                self.behavior = Behavior.REACTIVE  # self.behavior = Behavior.SPLINE
            else:
                self.get_logger().info("No Obstacles Detected")
                self.drive_pub.publish(msg)
                self.behavior = Behavior.PURE_PURSUIT
        else:
            self.get_logger().info("Not safe to drive")
            self.behavior = Behavior.STOP
        # Execute behavior
        behavior_msg = String()
        msg.data = self.behavior.name
        self.behavior_state_pub.publish(behavior_msg)
        self.safety_callback()

    def safety_callback(self):
        if self.behavior == Behavior.STOP:
            self.get_logger().info("Not safe to drive")
            brake_msg = AckermannDriveStamped()
            self.drive_pub.publish(brake_msg)
        if self.get_clock().now().nanoseconds-self.last_drive_msg_time.nanoseconds > self.get_parameter("timeout_threshold_ns").get_parameter_value().integer_value:
            self.get_logger().info("Timeout")
            brake_msg = AckermannDriveStamped()
            self.drive_pub.publish(brake_msg)

    def poses_callback(self, msg):
        pass

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

    def check_obstacles_in_grid(self):
        obstacles_array = []
        try:
            transform = self.tfBuffer.lookup_transform(
                "ego_racecar/laser", "map", rclpy.time.Time()
            )
            # Transform poses position to local map

            obstacles = MarkerArray()
            obstacles.markers = []

            self.get_logger().info(str(len(self.poses)) + "self.poses")
            for index_pose, pose in enumerate(self.poses):
                # elf.get_logger().info('Checking Obstacles')

                # Use transform to get the pose in the map frame
                pose_car_frame = tf2_geometry_msgs.do_transform_pose(pose, transform)
                # Get the pose in the map frame
                """
                    # Obstacle in discretized coordinate frame.
                    y_grid = int(y/self.resolution) + self.height//2 # x coordinate in grid frame
                    x_grid = int(x/self.resolution) # y coordinate in grid frame
                    """

                # Map pose to occupancy grid
                y_grid = (
                    int(pose_car_frame.position.y / self.occupancy_grid.info.resolution)
                    + self.occupancy_grid.info.height // 2
                )
                x_grid = int(
                    pose_car_frame.position.x / self.occupancy_grid.info.resolution
                )

                # Check if the pose is in the map
                if (
                    x_grid >= 0
                    and x_grid < self.occupancy_grid.info.width
                    and y_grid >= 0
                    and y_grid < self.occupancy_grid.info.height
                ):
                    # Check if it is occupied
                    if self.occupancy_grid[y_grid][x_grid] == 100:
                        obstacles_array.append([y_grid, x_grid])
                        # Create a marker for the obstacle
                        obstacle_marker = Marker()

                        obstacle_marker.header.frame_id = "map"
                        # Add a stamp time
                        obstacle_marker.header.stamp = rclpy.time.Time().to_msg()
                        # duration
                        obstacle_marker.lifetime = rclpy.time.Duration(
                            seconds=1.0
                        ).to_msg()
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
                        distance = math.sqrt(
                            (pose.position.x - self.odometry.pose.pose.position.x) ** 2
                            + (pose.position.y - self.odometry.pose.pose.position.y)
                            ** 2
                        )
                        angle = math.atan2(
                            pose.position.y - self.odometry.pose.pose.position.y,
                            pose.position.x - self.odometry.pose.pose.position.x,
                        )
                        self.get_logger().info("Obstacle Detected" + str(pose.position))
                        self.get_logger().info("Distance to Obstacle" + str(distance))
            if not len(obstacles.markers) > 0:
                self.get_logger().info("No Obstacles Detected")
            return obstacles_array
        except Exception as e:
            self.get_logger().info("Error in check_obstacles_in_grid" + str(e))
            return []


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
