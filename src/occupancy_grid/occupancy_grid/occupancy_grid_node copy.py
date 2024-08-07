import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Bool, String, Header
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from tf_transformations import euler_from_quaternion


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_node")

        self.car_position = None
        self.map_metadata = None
        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_height", 30),
                ("map_width", 40),
                ("resolution", 0.1),
                ("laser_frame", "ego_racecar/laser"),
                ("laser_topic", "/scan"),
                ("odom_topic", "/ego_racecar/odom"),
                ("map_topic", "/occupancy_grid"),
            ],
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.get_parameter("laser_topic").value, self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10
        )
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("map_topic").value, 10
        )
        self.curr_orientation = None
        self.curr_pos = None
        self.width = self.get_parameter("map_width").value
        self.height = self.get_parameter("map_height").value
        self.resolution = self.get_parameter("resolution").value
        self.rotation_matrix = None

    def in_grid(self, x_grid, y_grid):
        return (
            y_grid >= 0 and y_grid < self.height and x_grid >= 0 and x_grid < self.width
        )

    def odom_callback(self, pose_msg):
        self.curr_pos = pose_msg
        self.car_position = pose_msg

        self.map_metadata = self.generate_metadata()
        curr_x = pose_msg.pose.pose.position.x  # global frame
        curr_y = pose_msg.pose.pose.position.y  # global frame
        self.curr_pos = [curr_x, curr_y]  # global frame
        self.curr_orientation = [
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w,
        ]
        # heading = tf.transformations.euler_from_quaternion((pose_msg.pose.pose.orientation.x,
        # pose_msg.pose.pose.orientation.y,
        # pose_msg.pose.pose.orientation.z,
        # pose_msg.pose.pose.orientation.w))[2]

        euler = euler_from_quaternion(self.curr_orientation)
        heading = euler[2]
        self.rotation_matrix = np.array(
            [[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]]
        )

    def set_obstacle(self, grid, angles, dist, lidar_position=0.265):
        for d in range(len(dist)):
            x = dist[d] * np.cos(angles[d])  # x coordinate in lidar frame
            y = dist[d] * np.sin(angles[d])  # y coordinate in lidar frame

            # Obstacle in discretized coordinate frame.
            y_grid = int(y / self.resolution)  # x coordinate in grid frame
            x_grid = int(x / self.resolution)  # y coordinate in grid frame

            y_grid = y_grid + self.height // 2

            if self.in_grid(x_grid, y_grid):
                grid[y_grid, x_grid] = 100
        wangles = np.linspace(-np.pi * 1 / 2, np.pi * 1 / 2, num=len(dist))
        for angle in wangles:
            direction = np.array([np.cos(angle), np.sin(angle)])
            x_0 = 0
            y_0 = self.height // 2 * self.resolution
            xt = x_0
            yt = y_0
            x_grid = int(xt / self.resolution)
            y_grid = int(yt / self.resolution)
            t = 0
            while self.in_grid(x_grid, y_grid):
                if grid[y_grid, x_grid] == 100:
                    break
                xt = x_0 + t * direction[0]
                yt = y_0 + t * direction[1]
                x_grid = int(xt / self.resolution)
                y_grid = int(yt / self.resolution)
                if self.in_grid(x_grid, y_grid):
                    if grid[y_grid, x_grid] == 100:
                        break
                    grid[y_grid, x_grid] = 0

                t += self.resolution
        return

    def scan_callback(self, scan_msg):

        if self.car_position is not None and self.map_metadata is not None:

            grid = np.ndarray(
                (self.height, self.width),
                buffer=np.zeros((self.height, self.width), dtype=np.int),
                dtype=np.int,
            )
            grid.fill(int(-1))
            angle_min = scan_msg.angle_min
            angle_max = scan_msg.angle_max
            dist = list(scan_msg.ranges)
            angles = np.linspace(angle_min, angle_max, num=len(dist))  # laser frame

            self.set_obstacle(grid, angles, dist)

            map_msg = OccupancyGrid()
            map_msg.header.frame_id = "map"
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.info.resolution = self.get_parameter("resolution").value
            map_msg.info.width = self.width
            map_msg.info.height = self.height
            # map_msg.info.origin.position.x = self.curr_pos[0]# - self.width // 2 * self.resolution
            # map_msg.info.origin.position.y = self.curr_pos[1] - self.height // 2 * self.resolution

            # new_origin = [self.curr_pos[0] - (-self.height // 2 * self.resolution)*np.sin(euler[2]), self.curr_pos[1] + (-self.height // 2 * self.resolution)*np.cos(euler[2])]
            # print(new_origin)

            # print(np.dot(rotMatrix, np.array([0, + self.height // 2 * self.resolution])))
            # temp = np.dot(rotMatrix, np.array([0, + self.height // 2 * self.resolution])) + [self.curr_pos[0], self.curr_pos[1]]
            # print(temp)
            if self.rotation_matrix is None:
                return

            new_origin = np.dot(
                self.rotation_matrix, np.array([0, -self.height // 2 * self.resolution])
            ) + [self.curr_pos[0], self.curr_pos[1]]

            map_msg.info.origin.position.x = new_origin[0]
            map_msg.info.origin.position.y = new_origin[1]

            map_msg.info.origin.orientation.x = self.curr_orientation[0]
            map_msg.info.origin.orientation.y = self.curr_orientation[1]
            map_msg.info.origin.orientation.z = self.curr_orientation[2]
            map_msg.info.origin.orientation.w = self.curr_orientation[3]
            data = grid.flatten().tolist()
            map_msg.data = data
            self.occupancy_grid_pub.publish(map_msg)

    def generate_metadata(self):
        md = MapMetaData()
        md.resolution = self.get_parameter("resolution").value
        md.height = self.get_parameter("map_height").value
        md.width = self.get_parameter("map_width").value

        car_x = self.car_position.pose.pose.position.x
        car_y = self.car_position.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion(
            [
                self.car_position.pose.pose.orientation.x,
                self.car_position.pose.pose.orientation.y,
                self.car_position.pose.pose.orientation.z,
                self.car_position.pose.pose.orientation.w,
            ]
        )

        car_theta = (
            yaw  # Implement this method to get the car's orientation (yaw angle)
        )

        # Calculate the position of the top-left cell (0, 0) of the occupancy grid
        map_center_x = 0.0  # 2 meters divided by the total width of the grid in meters
        map_center_y = md.height / 2
        origin_x = (
            car_x
            - map_center_x * md.resolution * math.cos(car_theta)
            + map_center_y * md.resolution * math.sin(car_theta)
        )
        origin_y = (
            car_y
            - map_center_x * md.resolution * math.sin(car_theta)
            - map_center_y * md.resolution * math.cos(car_theta)
        )

        md.origin.position.x = origin_x
        md.origin.position.y = origin_y
        md.origin.position.z = 0.0  # Assuming a 2D map, so z position is 0.0

        # Orientation remains unchanged
        md.origin.orientation = self.car_position.pose.pose.orientation

        return md


def main():
    rclpy.init()
    print("Occupancy node init")
    sn = OccupancyGridNode()
    rclpy.spin(sn)

    rclpy.shutdown()
