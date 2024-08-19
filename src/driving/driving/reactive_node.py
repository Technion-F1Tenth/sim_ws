#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import String,Float64, Float64MultiArray
import sys
sys.path.insert(0, "/sim_ws/src")
from common import Behavior
from visualization_msgs.msg import MarkerArray, Marker
from f1tenth_messages.msg import ReactiveParams

SLOW_MODE = False
MAX_SPEED = 7.0
MIN_SPEED = 1.0
MIN_THRESHOLD_FOR_CORNER = 1.
CORNER_DETECTION_RESOLUTION = 2
CAR_WIDTH = 0.5
from enum import Enum

class Behavior(Enum):
    REACTIVE = 1
    PURE_PURSUIT = 2
    FOLLOW = 3
    STOP = 4
    ONLY_REACTIVE = 5
    SAFETY_STOP = 6
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
        parameteres_demo = [
                ('bubble_radius', 0.4),
                ('laser_topic', '/scan'),
                ('drive_topic', '/drive'),
                ('slow_mode', True),
                ('max_speed', 5.0),
                ('min_speed', 1.2),
                ('max_range', 5.0),
                ('car_width', 0.3),
                ("state_topic", "/behavior_state"),
                ("velocity_topic", "/follow_the_gap_velocity"),
                ("threshold_topic", "/follow_the_gap_threshold"),
                ("safety_factor", 1.2), #.3
                ("state_topic", "/behavior_state"),
                ("max_lidar_range", 30.0),
                ("disparity_threshold", 0.05),
                ("marker_topic", "/chosen_point"),
                ("angle_clip", 3/4 * math.pi),
                ("param_update", "/reactive/param_update"),
            ]

        self.declare_parameters(
            namespace='',
            parameters=parameteres_demo,
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
        


        # Configuration topics 
       
        self.param_update_sub = self.create_subscription(ReactiveParams, self.get_parameter("param_update").value, self.param_update_callback, 10)
        self.K_P = [0.35, 0.625] # [not slow mode, slow mode 0.325]
        self.K_D = [0.0, -0.135] # [not slow mode, slow mode]
        self.average_tracking_error = 0.0
        print("Reactive Node Initialized")
        print(parameteres_demo)

    def param_update_callback(self, msg):
        self.max_speed = msg.max_speed
        self.min_speed = msg.min_speed
        self.SAFETY_PERCENTAGE = msg.safety_factor
        self.K_P = msg.kp
        self.K_D = msg.kd
        print("changed")
        self.get_logger().info("Parameters Updated")
    
    

    


    def behavior_state_callback(self, msg):
        if msg.data == Behavior.REACTIVE.name:
            self.get_logger().info("Reactive Mode")
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
        min_angle = -90 * (math.pi/180) 
        max_angle = 90 * (math.pi/180) 
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
        

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        
        drive_msg.drive.steering_angle = steering_angle * self.K_P[int(self.slow_mode)]  + self.K_D[int(self.slow_mode)] * (steering_angle-self.prev_angle)

        speed = self.get_speed(drive_msg.drive.steering_angle) 

        # self.get_logger().info("steering_angle: {}, speed: {}".format(steering_angle, speed))

        self.prev_angle = drive_msg.drive.steering_angle
        drive_msg.drive.speed = speed 
        self.drive_pub.publish(drive_msg)

    def get_speed(self, steering_angle):
        velocity = self.min_speed
        if self.slow_mode:
            return velocity
        '''return max(
            self.max_speed * (1 - math.pow(abs(steering_angle), 2)), self.min_speed # 
        )'''
        if abs(steering_angle) < np.deg2rad(5):
            velocity = self.max_speed
        if abs(steering_angle) < np.deg2rad(10):
            velocity = self.max_speed * 0.9
        elif abs(steering_angle) < np.deg2rad(15):
            velocity = self.max_speed * 0.8
        else:
            velocity = self.min_speed
        a = 0.6
        k = 0.9
        return max(1.0, self.max_speed * (1 - a * abs(steering_angle)))


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()