#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String,Float64
import sys
sys.path.insert(0, "/sim_ws/src")
from common import Behavior
from visualization_msgs.msg import MarkerArray, Marker

SLOW_MODE = False
MAX_SPEED = 7.0
MIN_SPEED = 1.0
MIN_THRESHOLD_FOR_CORNER = 1.
CORNER_DETECTION_RESOLUTION = 2
CAR_WIDTH = 0.5
class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bubble_radius', 0.3),
                ('laser_topic', '/scan'),
                ('drive_topic', '/drive'),
                ('slow_mode', False),
                ('max_speed', 4.0),
                ('min_speed', 0.8),
                ('max_range', 5.0),
                ('car_width', 0.3),
                ("state_topic", "/behavior_state"),
                ("velocity_topic", "/follow_the_gap_velocity"),
                ("threshold_topic", "/follow_the_gap_threshold"),
                ("safety_factor", 1.6),
                ("state_topic", "/behavior_state"),
                ("max_lidar_range", 5.0),
                ("disparity_threshold", 0.1),
                ("marker_topic", "/chosen_point"),
                ("angle_clip", 3/4 * math.pi)
            ],
        )

        self.STEERING_SENSITIVITY = 1.0
        self.SAFETY_PERCENTAGE = self.get_parameter("safety_factor").value
        self.QUADRANT_FACTOR = 3.5
        self.DIFFERENCE_THRESHOLD = self.get_parameter("disparity_threshold").value
        self.car_width = self.get_parameter("car_width").value
        
        self.min_speed = self.get_parameter("min_speed").value
        self.max_speed = self.get_parameter("max_speed").value
        self.slow_mode = self.get_parameter("slow_mode").value
        self.max_lidar_range = self.get_parameter("max_lidar_range").value
        self.angle_clip = self.get_parameter("angle_clip").value

        self.bubble_radius = self.get_parameter("bubble_radius").value
        self.create_subscription(
            LaserScan, self.get_parameter("laser_topic").value, self.lidar_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.get_parameter("drive_topic").value, 10
        )
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter("marker_topic").value, 10
        )
        self.current_steeering_angle = 0.0
        self.integral = 0.0
        self.prev_best_point = None

        self.behavior_state_sub = self.create_subscription(
            String,
            self.get_parameter("state_topic").value,
            self.behavior_state_callback,
            10,
        )
        self.active = True  # Change to true if running without behavior node
        self.prev_angle = 0

    def behavior_state_callback(self, msg):
        if msg.data == Behavior.REACTIVE.name:
            self.active = True
        else:
            self.active = False
        print(msg)

    def preprocess_lidar(self, data):
        """Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 3m)
        """
        ranges = data.ranges
        ranges = np.array(ranges)
        min_angle = -120 * (math.pi/180) 
        max_angle = 120 * (math.pi/180) 
        min_idx = int(math.floor((min_angle - data.angle_min)/data.angle_increment))
        max_idx = int(math.floor((max_angle - data.angle_min)/data.angle_increment))
        ranges = np.clip(ranges, 0, 16)
        eighth = int(len(ranges) / self.QUADRANT_FACTOR)
        return np.array(ranges[min_idx:max_idx])

    def get_differences(self, ranges):
        differences = [0.0]  # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i] - ranges[i - 1]))
        return differences

    def get_disparities(self, differences, threshold):
        """Gets the indexes of the LiDAR points that were greatly
        different to their adjacent point.
        Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        angle = 1.5 * np.arctan(width / (2 * dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx + 1 + i
                if next_idx >= len(ranges):
                    break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx - 1 - i
                if next_idx < 0:
                    break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):

        width_to_cover = car_width * extra_pct
        for index in disparities:
            first_idx = index - 1
            points = ranges[first_idx : first_idx + 2]
            close_idx = first_idx + np.argmin(points)
            far_idx = first_idx + np.argmax(points)
            close_dist = ranges[close_idx]
            num_points_to_cover = self.get_num_points_to_cover(
                close_dist, width_to_cover
            )
            cover_right = close_idx < far_idx
            ranges = self.cover_points(
                num_points_to_cover, close_idx, cover_right, ranges
            )
        return ranges

    def get_steering_angle(self, range_index, range_len):

        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_point
        steering_angle = (
            np.clip(lidar_angle, np.radians(-90), np.radians(90))
            / self.STEERING_SENSITIVITY
        )
        return steering_angle

    def lidar_callback(self, data):
        if not self.active:
            return False

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()

        ranges = data.ranges
        self.radians_per_point = (data.angle_max - data.angle_min) / len(ranges)
        proc_ranges = self.preprocess_lidar(data)
        differences = self.get_differences(proc_ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(
            disparities, proc_ranges, self.car_width, self.SAFETY_PERCENTAGE
        )
        steering_angle = self.get_steering_angle(proc_ranges.argmax(), len(proc_ranges))
        speed = self.get_speed(steering_angle)

        self.get_logger().info("steering_angle: {}, speed: {}".format(steering_angle, speed))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed 
        self.drive_pub.publish(drive_msg)

    def get_speed(self, steering_angle):
        if self.slow_mode:
            return self.min_speed
        return max(
            self.max_speed * (1 - math.pow(abs(steering_angle), 1 / 3)), self.min_speed
        )


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
