import math

import numpy as np

from mrs_mocap_mapping import frames


def test_z_up_is_identity():
    p = np.array([1.0, 2.0, 3.0])
    assert np.allclose(frames.to_map_frame(p, 'z'), p)
    q = np.array([0.1, 0.2, 0.3, 0.9]) / np.linalg.norm([0.1, 0.2, 0.3, 0.9])
    assert np.allclose(frames.quat_to_map_frame(q, 'z'), q)


def test_y_up_points_map_to_z_up():
    # Raw Y-up: a marker 1 m above the floor and 2 m along raw +Z.
    raw = np.array([[0.5, 1.0, 2.0]])
    assert np.allclose(frames.to_map_frame(raw, 'y'), [[0.5, -2.0, 1.0]])
    assert np.isclose(np.linalg.det(frames.rotation('y')), 1.0)


def test_y_up_yaw_about_raw_y_becomes_yaw_about_map_z():
    # Rotating a body by +a about the raw up axis (Y) is a yaw of +a in the map frame.
    a = 0.7
    q_raw = np.array([0.0, math.sin(a / 2), 0.0, math.cos(a / 2)])
    q_map = frames.quat_to_map_frame(q_raw, 'y')
    assert np.isclose(frames.yaw_from_quat(q_map), a)
    assert np.allclose(q_map, frames.quat_from_yaw(a))


def test_bad_up_axis():
    try:
        frames.to_map_frame([0, 0, 0], 'x')
    except ValueError:
        return
    raise AssertionError('expected ValueError')


def test_body_offset_follows_body_rotation():
    # Body at (1, 2) facing +y (yaw 90 deg); base_link 0.2 m behind the pivot, rotated +10 deg.
    pos, quat = frames.apply_body_offset([1.0, 2.0, 0.1], frames.quat_from_yaw(math.pi / 2),
                                         [-0.2, 0.0, -0.1, math.radians(10)])
    assert np.allclose(pos, [1.0, 1.8, 0.0])
    assert np.isclose(frames.yaw_from_quat(quat), math.pi / 2 + math.radians(10))


def test_zero_offset_is_identity():
    q = frames.quat_from_yaw(0.3)
    pos, quat = frames.apply_body_offset([1.0, 2.0, 3.0], q, [0, 0, 0, 0])
    assert np.allclose(pos, [1.0, 2.0, 3.0]) and np.allclose(quat, q)
