# Node 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point, PoseStamped, Pose, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import numpy as np
from scipy.interpolate import BSpline, CubicSpline
import datetime

class waypoint_drawer_node(Node):
    def __init__(self):
        super().__init__('waypoint_drawer_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('point_topic', '/clicked_point'),
                ('command_topic', '/waypoint_command'),
                ('marker_topic', '/waypoint_array'),
                ('inner_points', 100),
                ('global_frame', 'map')

            ]
        )

        self.point_sub = self.create_subscription(PointStamped, self.get_parameter('point_topic').value, self.point_callback, 10)

        self.waypoints = []
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('marker_topic').value, 5)
        self.command_sub = self.create_subscription(String, self.get_parameter('command_topic').value, self.command_callback, 10)
     
    def command_callback(self, data):
        data.data = ''.join(e for e in data.data if e.isalnum()) # remove all non alphanumeric characters from the string
        if data.data == "RESET":
             self.waypoints = []
             self.get_logger().info("Waypoints Reset")
             self.init_markers(self.waypoints)
        elif data.data == "PUBLISH":
            spline = []
            for e in self.waypoints:
                spline.append([e[0], e[1]])
            
            spline = spline + spline[1:]
            x, y = zip(*spline)
            

            dx = np.diff(x)
            dy = np.diff(y)
            ds = np.sqrt(dx**2 + dy**2)
            s = np.cumsum(ds)
            s = np.insert(s, 0, 0)  

            x_spline = CubicSpline(s, x)
            y_spline = CubicSpline(s, y)

            new_s = np.linspace(min(s), max(s), self.get_parameter('inner_points').value)  
            
            new_x = x_spline(new_s)
            new_y = y_spline(new_s)

            # publish the new waypoints
            new_waypoints = []
            for i in range(len(new_x)):
                new_waypoints.append([new_x[i], new_y[i], 0.0, 1.0])
            self.init_markers(new_waypoints)
            self.waypoints = new_waypoints
            self.get_logger().info("Waypoints Published")
            self.save_to_csv()


    def save_to_csv(self):
        # Get current date and time to make file name
        now = datetime.datetime.now()
        date_time = now.strftime("%m-%d-%Y_%H-%M-%S")
        # Save waypoints to csv
        file_name = "wps/waypoints_" + date_time + ".csv"
        np.savetxt(file_name, self.waypoints, delimiter=",")
        self.get_logger().info("Waypoints Saved to CSV")
    
    def point_callback(self, data):
        self.waypoints.append([data.point.x, data.point.y, data.point.z, 1.0])
        self.init_markers(self.waypoints)
        self.get_logger().info("Waypoint Added")



    def init_markers(self, data):
        marker_count = 0
        marker_array_msg = MarkerArray()
        for e in data:
            marker = Marker()
            marker.id = marker_count
            marker_count += 1
            marker.header.frame_id = self.get_parameter('global_frame').value
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            point_stamped = PointStamped()
            point_stamped.point.x = (e[0]) 
            point_stamped.point.y = (e[1]) 
            point_stamped.point.z = (e[2])
            point_stamped.header.frame_id = self.get_parameter('global_frame').value
            # use the derivative to get the orientation of the curve at that point
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.pose.position = point_stamped.point

            marker_array_msg.markers.append(marker)
        self.marker_pub.publish(marker_array_msg)
        self.get_logger().info("Markers Published")
        


def main(args=None):
    rclpy.init(args=args)
    waypoint_drawer = waypoint_drawer_node()
    rclpy.spin(waypoint_drawer)
    waypoint_drawer.destroy_node()
    rclpy.shutdown()