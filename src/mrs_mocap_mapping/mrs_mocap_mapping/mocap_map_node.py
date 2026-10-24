"""
Mocap map node: OptiTrack markers -> web-edited obstacles -> OccupancyGrid.

Subscribes to the mocap4r2_optitrack_driver outputs (`markers_topic`,
mocap4r2_msgs/Markers; `rigid_bodies_topic`, mocap4r2_msgs/RigidBodies),
converts them into a Z-up map frame (`up_axis`, see frames.py), and serves
the editor UI on http://<web.host>:<web.port>. Publishes:

  map_topic (default /mocap_map)            nav_msgs/OccupancyGrid, transient_local,
                                            on every edit and every map_publish_period
  <map_topic>/obstacles                     visualization_msgs/MarkerArray (obstacles
                                            + live markers) for Foxglove/RViz
  odom_topic (default /mocap_odom)          nav_msgs/Odometry of the car's base_link in
                                            map_frame, twist in the body frame
  mocap_scan_topic (default /mocap_scan)    scan_topic relayed with frame_id = laser_frame
  TF map_frame -> base_frame -> laser_frame mocap_base_link (per mocap frame) and a
                                            static base_to_laser (publish_tf)

The mocap TF subtree is deliberately parallel to the car's own
odom -> base_link -> laser tree (TF allows one parent per frame), hence the
`mocap_` frame names and the scan relay. TF, odom and relayed scans are
stamped with this node's clock: the mocap PC and the car do not share a
clock, and display tools must be able to resolve the transforms. The twist
fit uses the mocap header stamps, which are evenly spaced at the source.
"""

import array
import logging
import os
import threading

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import LaserScan
from mocap4r2_msgs.msg import Markers, RigidBodies
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data,
                       ReliabilityPolicy)
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from werkzeug.serving import make_server

from . import frames
from .grid import GridSpec
from .map_state import MapState
from .odometry import TwistEstimator
from .web_app import create_app

OBSTACLE_COLORS = {
    'polygon': (0.90, 0.30, 0.20),
    'polyline': (0.95, 0.65, 0.10),
    'loop': (0.20, 0.55, 0.95),
}


class MocapMapNode(Node):

    def __init__(self):
        super().__init__('mocap_map')

        p = self._declare_all()
        if not p['car_rigid_body_name']:
            self.get_logger().warn('car_rigid_body_name is empty: no car pose, TF or exclusion')
        self.car_name = p['car_rigid_body_name']
        self.map_frame = p['map_frame']
        self.base_frame = p['base_frame']
        self.laser_frame = p['laser_frame']
        self.publish_tf = p['publish_tf']
        self.pose_cov = p['pose_cov']
        self.twist_cov = p['twist_cov']
        self.twist = TwistEstimator(window=p['twist_window'])

        grid = GridSpec.from_metres(p['grid.resolution'], p['grid.origin_x'],
                                    p['grid.origin_y'], p['grid.size_x'], p['grid.size_y'])
        self.state = MapState(
            grid, up_axis=p['up_axis'], map_file=p['map_file'] or None,
            car_exclusion_radius=p['car_exclusion_radius'],
            car_marker_tolerance=p['car_marker_tolerance'],
            match_tolerance=p['marker_match_tolerance'],
            body_to_base=p['body_to_base'],
        )
        if self.state.version:
            self.get_logger().info(
                f'Loaded {len(self.state.obstacles)} obstacles and grid from '
                f'{self.state.map_file} (grid.* parameters ignored)')
        if self.state.load_note:
            self.get_logger().warn(self.state.load_note)

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, p['map_topic'], latched)
        self.viz_pub = self.create_publisher(MarkerArray, p['map_topic'] + '/obstacles', 10)
        self.odom_pub = self.create_publisher(Odometry, p['odom_topic'], 10)
        self.tf_broadcaster = None
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.static_broadcaster = StaticTransformBroadcaster(self)
            self.static_broadcaster.sendTransform(self._static_laser_tf(p['base_to_laser']))

        self.scan_pub = None
        if p['mocap_scan_topic']:
            self.scan_pub = self.create_publisher(LaserScan, p['mocap_scan_topic'],
                                                  qos_profile_sensor_data)
            self.create_subscription(LaserScan, p['scan_topic'], self._scan_cb,
                                     qos_profile_sensor_data)

        self.create_subscription(Markers, p['markers_topic'], self._markers_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(RigidBodies, p['rigid_bodies_topic'], self._rigid_bodies_cb,
                                 qos_profile_sensor_data)

        self._published_version = -1
        self._viz_version = -1
        self.create_timer(0.1, self._tick)
        self.create_timer(max(p['map_publish_period'], 0.1), self._republish_map)

        static_dir = os.path.join(get_package_share_directory('mrs_mocap_mapping'), 'web')
        app = create_app(self.state, static_dir)
        logging.getLogger('werkzeug').setLevel(logging.WARNING)   # no per-request log lines
        self._server = make_server(p['web.host'], p['web.port'], app, threaded=True)
        self._web_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._web_thread.start()

        self.get_logger().info(
            f'MocapMapNode: {p["markers_topic"]} + {p["rigid_bodies_topic"]} '
            f'(car="{self.car_name}", up_axis={p["up_axis"]}) -> {p["map_topic"]}; '
            f'editor at http://{p["web.host"]}:{p["web.port"]}/')

    def _declare_all(self) -> dict:
        defaults = {
            'markers_topic': '/markers',
            'rigid_bodies_topic': '/rigid_bodies',
            'car_rigid_body_name': '',
            'up_axis': 'z',
            'map_frame': 'map',
            'base_frame': 'mocap_base_link',
            'laser_frame': 'mocap_laser',
            'publish_tf': True,
            'body_to_base': [0.0, 0.0, 0.0, 0.0],
            'base_to_laser': [0.275, 0.0, 0.0, 0.0],
            'odom_topic': '/mocap_odom',
            'twist_window': 9,
            'pose_cov': 1e-6,
            'twist_cov': 1e-3,
            'scan_topic': '/scan',
            'mocap_scan_topic': '/mocap_scan',
            'car_exclusion_radius': 0.3,
            'car_marker_tolerance': 0.02,
            'marker_match_tolerance': 0.05,
            'map_topic': '/mocap_map',
            'map_file': '~/mocap_maps/arena.json',
            'map_publish_period': 1.0,
            'grid.resolution': 0.05,
            'grid.origin_x': -5.0,
            'grid.origin_y': -5.0,
            'grid.size_x': 10.0,
            'grid.size_y': 10.0,
            'web.host': '0.0.0.0',
            'web.port': 8080,
        }
        p = {k: self.declare_parameter(k, v).value for k, v in defaults.items() if k != 'up_axis'}
        # A bare `y` is a YAML 1.1 boolean, and launch/params files hand it to rcl unquoted,
        # so up_axis is dynamically typed and True is read back as 'y'.
        up = self.declare_parameter('up_axis', defaults['up_axis'],
                                    ParameterDescriptor(dynamic_typing=True)).value
        p['up_axis'] = 'y' if up is True else str(up).lower()
        return p

    # ---- mocap input ------------------------------------------------------

    def _markers_cb(self, msg: Markers) -> None:
        ids = [m.marker_index for m in msg.markers]
        pos = [(m.translation.x, m.translation.y, m.translation.z) for m in msg.markers]
        self.state.set_markers(ids, pos)

    def _rigid_bodies_cb(self, msg: RigidBodies) -> None:
        if not self.car_name:
            return
        for rb in msg.rigidbodies:
            if rb.rigid_body_name != self.car_name:
                continue
            p, q = rb.pose.position, rb.pose.orientation
            mk = [(m.translation.x, m.translation.y, m.translation.z) for m in rb.markers]
            pos, quat = self.state.set_car((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), mk)
            self._publish_car(msg.header.stamp, pos, quat)
            return
        self.twist.reset()
        self.get_logger().warn(
            f'car rigid body "{self.car_name}" not present in this RigidBodies frame',
            throttle_duration_sec=2.0)

    def _scan_cb(self, msg: LaserScan) -> None:
        msg.header.frame_id = self.laser_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)

    # ---- output -----------------------------------------------------------

    def _publish_car(self, mocap_stamp, pos, quat) -> None:
        stamp = self.get_clock().now().to_msg()
        t_mocap = mocap_stamp.sec + mocap_stamp.nanosec * 1e-9
        if t_mocap == 0.0:
            t_mocap = stamp.sec + stamp.nanosec * 1e-9
        vel = self.twist.update(t_mocap, float(pos[0]), float(pos[1]), frames.yaw_from_quat(quat))

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.base_frame
            _fill_transform(tf, pos, quat)
            self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        pp, pq = odom.pose.pose.position, odom.pose.pose.orientation
        pp.x, pp.y, pp.z = (float(v) for v in pos)
        pq.x, pq.y, pq.z, pq.w = (float(v) for v in quat)
        odom.pose.covariance = _diag_cov(self.pose_cov)
        if vel is None:
            odom.twist.covariance = _diag_cov(1e6)     # unknown until the fit window fills
        else:
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = vel[0], vel[1]
            odom.twist.twist.angular.z = vel[2]
            odom.twist.covariance = _diag_cov(self.twist_cov)
        self.odom_pub.publish(odom)

    def _static_laser_tf(self, base_to_laser) -> TransformStamped:
        if len(base_to_laser) != 4:
            raise ValueError('base_to_laser must be [x, y, z, yaw]')
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.laser_frame
        _fill_transform(tf, base_to_laser[:3], frames.quat_from_yaw(base_to_laser[3]))
        return tf

    def _tick(self) -> None:
        if self.state.version != self._published_version:
            self._republish_map()
        self._publish_viz()

    def _republish_map(self) -> None:
        version, spec, grid = self.state.occupancy()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = spec.resolution
        msg.info.width, msg.info.height = spec.width, spec.height
        msg.info.origin.position.x = spec.origin_x
        msg.info.origin.position.y = spec.origin_y
        msg.info.origin.position.z = spec.origin_z
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', grid.ravel().tobytes())
        self.map_pub.publish(msg)
        self._published_version = version

    def _publish_viz(self) -> None:
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        snap_version = self.state.version
        if snap_version != self._viz_version:
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            for k, ob in enumerate(list(self.state.obstacles)):
                arr.markers.append(self._obstacle_marker(ob, k, stamp))
            self._viz_version = snap_version

        positions, excluded = self.state.live_markers()
        live = Marker()
        live.header.frame_id = self.map_frame
        live.header.stamp = stamp
        live.ns = 'live_markers'
        live.type = Marker.SPHERE_LIST
        live.scale.x = live.scale.y = live.scale.z = 0.03
        live.pose.orientation.w = 1.0
        live.lifetime = Duration(nanosec=500_000_000)
        for pt, ex in zip(positions, excluded):
            live.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
            live.colors.append(ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6) if ex
                               else ColorRGBA(r=0.1, g=0.9, b=0.3, a=1.0))
        arr.markers.append(live)
        self.viz_pub.publish(arr)

    def _obstacle_marker(self, ob, k, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = 'obstacles'
        m.id = k
        m.type = Marker.LINE_STRIP
        m.pose.orientation.w = 1.0
        m.scale.x = ob.width if ob.thin else 0.02
        r, g, b = OBSTACLE_COLORS.get(ob.kind, (1.0, 1.0, 1.0))
        m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
        verts = list(ob.vertices)
        if ob.closed:
            verts.append(ob.vertices[0])
        m.points = [Point(x=float(v[0]), y=float(v[1]), z=0.0) for v in verts]
        return m

    def destroy_node(self):
        self._server.shutdown()
        super().destroy_node()


def _fill_transform(tf: TransformStamped, pos, quat) -> None:
    t, r = tf.transform.translation, tf.transform.rotation
    t.x, t.y, t.z = (float(v) for v in pos)
    r.x, r.y, r.z, r.w = (float(v) for v in quat)


def _diag_cov(value: float) -> list:
    return [float(value) if i % 7 == 0 else 0.0 for i in range(36)]


def main(args=None):
    rclpy.init(args=args)
    node = MocapMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
