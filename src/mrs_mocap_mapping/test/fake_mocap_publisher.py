"""
Publish fake mocap4r2 Markers + RigidBodies for trying the editor without the rig.

Run with `python3 test/fake_mocap_publisher.py [--up-axis y]` after sourcing a
workspace that provides mocap4r2_msgs. Scene (map frame, metres): a car
named `car` driving a 1.2 m circle, a ring of 12 cones (r = 2.5 m, z = 0.3),
a 4-marker box at (-3, 1) and a 3-marker straight wall along y = -3.
Also publishes /scan (frame `laser`, 270 deg, 1080 beams) ray-cast against the
cone ring, box and wall from a laser 0.275 m ahead of the car pivot, so the
/mocap_scan relay can be checked against the map.
Not installed and not collected by pytest.
"""

import argparse
import math

from geometry_msgs.msg import Point
from mocap4r2_msgs.msg import Marker, Markers, RigidBodies, RigidBody
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def scene_markers():
    pts = [(2.5 * math.cos(a), 2.5 * math.sin(a), 0.3)
           for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    pts += [(-3.2, 0.8, 0.2), (-2.8, 0.8, 0.2), (-2.8, 1.2, 0.2), (-3.2, 1.2, 0.2)]
    pts += [(-1.5, -3.0, 0.1), (0.0, -3.0, 0.1), (1.5, -3.0, 0.1)]
    return pts


def scene_segments():
    """Line segments (N, 2, 2) the fake lidar sees: cone ring, box outline, wall."""
    pts = scene_markers()
    ring, box, wall = pts[:12], pts[12:16], pts[16:]
    segs = []
    for chain, closed in ((ring, True), (box, True), (wall, False)):
        xy = [p[:2] for p in chain]
        pairs = list(zip(xy, xy[1:] + xy[:1])) if closed else list(zip(xy[:-1], xy[1:]))
        segs += pairs
    return np.array(segs, dtype=float)


def ray_cast(origin, angles, segs, max_range=30.0):
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1)[:, None, :]     # (B, 1, 2)
    a, b = segs[None, :, 0, :], segs[None, :, 1, :]                       # (1, S, 2)
    e = b - a
    w = a - origin
    den = d[..., 0] * e[..., 1] - d[..., 1] * e[..., 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[..., 0] * e[..., 1] - w[..., 1] * e[..., 0]) / den
        u = (w[..., 0] * d[..., 1] - w[..., 1] * d[..., 0]) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    return np.where(hit, t, np.inf).min(axis=1).clip(max=max_range)


class FakeMocap(Node):

    def __init__(self, up_axis):
        super().__init__('fake_mocap')
        self.up_axis = up_axis
        self.markers_pub = self.create_publisher(Markers, '/markers', 10)
        self.bodies_pub = self.create_publisher(RigidBodies, '/rigid_bodies', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.static = scene_markers()
        self.segments = scene_segments()
        self.car = (0.0, -1.2, 0.0)
        self.t = 0.0
        self.create_timer(0.01, self._tick)
        self.create_timer(0.025, self._scan)

    def _scan(self):
        cx, cy, heading = self.car
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min, scan.angle_max = -3 * math.pi / 4, 3 * math.pi / 4
        n = 1080
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (n - 1)
        scan.range_min, scan.range_max = 0.06, 30.0
        origin = np.array([cx + 0.275 * math.cos(heading), cy + 0.275 * math.sin(heading)])
        angles = heading + scan.angle_min + scan.angle_increment * np.arange(n)
        scan.ranges = ray_cast(origin, angles, self.segments).astype(np.float32).tolist()
        self.scan_pub.publish(scan)

    def _raw(self, x, y, z):
        # Map frame (Z-up) -> raw. For Y-up the inverse of map = (x, -z, y) is raw = (x, z, -y).
        return (x, z, -y) if self.up_axis == 'y' else (x, y, z)

    def _tick(self):
        self.t += 0.01
        yaw = 0.5 * self.t
        cx, cy = 1.2 * math.cos(yaw - math.pi / 2), 1.2 * math.sin(yaw - math.pi / 2)
        heading = yaw
        self.car = (cx, cy, heading)
        car_local = [(0.15, 0.0, 0.12), (-0.1, 0.08, 0.12), (-0.1, -0.08, 0.15)]
        car_mk = [(cx + math.cos(heading) * px - math.sin(heading) * py,
                   cy + math.sin(heading) * px + math.cos(heading) * py, pz)
                  for px, py, pz in car_local]

        stamp = self.get_clock().now().to_msg()
        markers = Markers()
        markers.header.stamp = stamp
        markers.header.frame_id = 'mocap'
        for i, p in enumerate(self.static + car_mk):
            m = Marker()
            m.marker_index = i
            m.translation = Point(**dict(zip('xyz', self._raw(*p))))
            markers.markers.append(m)
        self.markers_pub.publish(markers)

        body = RigidBody()
        body.rigid_body_name = 'car'
        body.pose.position = Point(**dict(zip('xyz', self._raw(cx, cy, 0.1))))
        # Yaw about map z; for Y-up that is a rotation about raw +y.
        s, c = math.sin(heading / 2), math.cos(heading / 2)
        q = body.pose.orientation
        if self.up_axis == 'y':
            q.x, q.y, q.z, q.w = 0.0, s, 0.0, c
        else:
            q.x, q.y, q.z, q.w = 0.0, 0.0, s, c
        for p in car_mk:
            m = Marker()
            m.translation = Point(**dict(zip('xyz', self._raw(*p))))
            body.markers.append(m)
        bodies = RigidBodies()
        bodies.header = markers.header
        bodies.rigidbodies.append(body)
        self.bodies_pub.publish(bodies)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--up-axis', choices=('z', 'y'), default='z')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeMocap(args.up_axis)
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
