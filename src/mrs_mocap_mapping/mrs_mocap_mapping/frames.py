"""
Mocap -> map frame conversion (rclpy-free).

OptiTrack/Motive streams Y-up by default; mocap4r2 can also be configured
Z-up. Everything downstream of this module (obstacles, grid, TF) works in a
right-handed, Z-up `map` frame:

    up_axis='z': map = raw
    up_axis='y': map = (x, -z, y)   (a +90 deg rotation about raw x)

Orientations are conjugated by the same rotation (q_R * q * q_R^-1), so a
rigid body whose local axes were created aligned with the Y-up world ends
up with its local axes aligned with the Z-up map frame as well.

Quaternions are (x, y, z, w), matching geometry_msgs.
"""

import math

import numpy as np

UP_AXES = ('z', 'y')

_ROTATIONS = {
    'z': np.eye(3),
    'y': np.array([[1.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0],
                   [0.0, 1.0, 0.0]]),
}

_S = math.sqrt(0.5)
_ROTATION_QUATS = {
    'z': np.array([0.0, 0.0, 0.0, 1.0]),
    'y': np.array([_S, 0.0, 0.0, _S]),
}


def check_up_axis(up_axis: str) -> str:
    if up_axis not in UP_AXES:
        raise ValueError(f'up_axis must be one of {UP_AXES}, got {up_axis!r}')
    return up_axis


def rotation(up_axis: str) -> np.ndarray:
    """Return the 3x3 rotation taking raw mocap coordinates to the map frame."""
    return _ROTATIONS[check_up_axis(up_axis)]


def to_map_frame(points, up_axis: str) -> np.ndarray:
    """Rotate a point (3,) or points (N, 3) from raw mocap into the map frame."""
    p = np.asarray(points, dtype=float)
    return p @ rotation(up_axis).T


def quat_multiply(a, b) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_conjugate(q) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def quat_to_map_frame(q, up_axis: str) -> np.ndarray:
    q_r = _ROTATION_QUATS[check_up_axis(up_axis)]
    out = quat_multiply(quat_multiply(q_r, np.asarray(q, dtype=float)), quat_conjugate(q_r))
    return out / np.linalg.norm(out)


def quat_from_yaw(yaw: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def yaw_from_quat(q) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def apply_body_offset(position, quat, offset):
    """
    Move a pose by a fixed offset expressed in its own (body) frame.

    `offset` is [x, y, z, yaw]: e.g. the Motive rigid-body pivot -> the car's
    base_link (rear axle). Returns (position, quat) of the offset frame.
    """
    dx, dy, dz, dyaw = (float(v) for v in offset)
    pos = np.asarray(position, dtype=float) + quat_to_matrix(quat) @ np.array([dx, dy, dz])
    q = quat_multiply(np.asarray(quat, dtype=float), quat_from_yaw(dyaw))
    return pos, q / np.linalg.norm(q)
