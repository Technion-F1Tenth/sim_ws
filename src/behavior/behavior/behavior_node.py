#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
SLOW_MODE = True
MAX_SPEED = 7.0
MIN_SPEED = 1.0
MIN_THRESHOLD_FOR_CORNER = 1.
CORNER_DETECTION_RESOLUTION = 2
CAR_WIDTH = 0.5
BRAKE_MESSAGE = AckermannDriveStamped()
BRAKE_MESSAGE.drive.speed = 0.0


class Behavior(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        reactive_node_drive_topic = '/reactive_drive'
        drive_topic = '/drive'
        laser_topic = '/scan'
        command_topic = '/behavior_command'
        self.bubble_radius = 0.5
        self.create_subscription(AckermannDriveStamped, reactive_node_drive_topic, self.reactive_drive,10)
        self.create_subscription(String, command_topic, self.command_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.create_subscription(LaserScan, laser_topic, self.lidar_callback, 10)
        self.drive_message = BRAKE_MESSAGE
        self.state = "REACTIVE"
        # TODO: Publish to drive

    def reactive_drive(self, message):
        if self.state == "REACTIVE":
            self.drive_message = message

    def command_callback(self, message):
        command = message.data
        print(command)
        if command=="REACTIVE":
            self.state="REACTIVE"
        elif command=="BRAKE":
            self.state="BRAKE"
            self.drive_message = BRAKE_MESSAGE

    

    def lidar_callback(self, data):
        self.drive_pub.publish(self.drive_message)

    
    
        
        


def main(args=None):
    rclpy.init(args=args)
    print("Behavior Initialized")
    behavior_node = Behavior()
    rclpy.spin(behavior_node)

    behavior_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()