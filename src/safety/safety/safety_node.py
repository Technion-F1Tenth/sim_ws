import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
class Safety(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0
        self.ttc_threshold = 0.5
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 10)
        self.command_pub = self.create_publisher(String, '/behavior_command', 10)
        self.brake_pub = self.create_publisher(AckermannDriveStamped, '/brake', 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        will_crash = False
        min_ttc = np.inf
        for index, distance in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            projected_vel = self.speed * math.cos(angle)  # Corrected projected velocity
            projected_vel = max(projected_vel, 0)
            
            if projected_vel != 0:
                time_to_crash = distance / projected_vel
                min_ttc = min(time_to_crash, min_ttc)
                if time_to_crash < self.ttc_threshold:
                    will_crash = True
                    
        
        if will_crash:
            ackerman_msg = AckermannDriveStamped()
            ackerman_msg.drive.speed = 0.0
            command_msg = String()
            command_msg.data = "BRAKE"
            self.command_pub.publish(command_msg)
            self.brake_pub.publish(ackerman_msg)
            print("BRAAAKE")

def main():
    rclpy.init()
    print("Safety node init")
    sn = Safety()
    rclpy.spin(sn)

    rclpy.shutdown()
