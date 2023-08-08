import math
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from time import gmtime, strftime
import tf_transformations
from os.path import expanduser
from numpy import linalg as LA

home = expanduser('~')

file = open(strftime(home+'/sim_ws/wps/wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv', 'w+')

class WayPointLogger(Node):
    def log_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x,
                               data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z,
                               data.pose.pose.orientation.w])

        euler = tf_transformations.euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([data.twist.twist.linear.x,
                                  data.twist.twist.linear.y,
                                  data.twist.twist.linear.z]), 2)
        

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                         data.pose.pose.position.y,
                                         euler[2],
                                         speed))

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.id = self.count
        
        marker.pose.position.x = data.pose.pose.position.x
        marker.pose.position.y = data.pose.pose.position.y
        marker.pose.position.z = data.pose.pose.position.z
        marker.pose.orientation.x = data.pose.pose.orientation.x
        marker.pose.orientation.y = data.pose.pose.orientation.y
        marker.pose.orientation.z = data.pose.pose.orientation.z
        marker.pose.orientation.w = data.pose.pose.orientation.w
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.header.frame_id = 'map'
        marker.color.r = 0.0
        marker.color.g = 2.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.marker_pub.publish(marker)
        self.count += 1

    def __init__(self):
        super().__init__('waypoint_logger')
        pose_topic = 'ego_racecar/odom'
        self.count = 0
        self.pose_sub = self.create_subscription(Odometry, pose_topic, self.log_waypoint, 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 5)

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    waypoint_node = WayPointLogger()
    rclpy.spin(waypoint_node)

    waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()