import math

import pytest

from mrs_mocap_mapping.odometry import TwistEstimator

DT = 1.0 / 120.0


def test_circle_gives_forward_speed_and_yaw_rate():
    # Counter-clockwise circle, r = 1.2 m, a = 2t -> w = 2 rad/s, v = 2.4 m/s, heading tangent.
    est, out = TwistEstimator(window=9), None
    for k in range(400):     # crosses the +-pi yaw wrap
        t = k * DT
        a = 2.0 * t
        x, y = 1.2 * math.cos(a - math.pi / 2), 1.2 * math.sin(a - math.pi / 2)
        out = est.update(t, x, y, math.remainder(a, 2 * math.pi))
        if k > 10:
            assert out is not None
    vx, vy, wz = out
    assert vx == pytest.approx(2.4, rel=1e-2)
    assert abs(vy) < 1e-3
    assert wz == pytest.approx(2.0, rel=1e-3)


def test_reversing_gives_negative_vx():
    est, out = TwistEstimator(window=5), None
    for k in range(10):
        out = est.update(k * DT, -0.5 * k * DT, 0.0, 0.0)   # facing +x, moving -x
    assert out[0] == pytest.approx(-0.5) and out[1] == pytest.approx(0.0)


def test_gap_and_duplicate_stamps_reset_or_skip():
    est = TwistEstimator(window=3, max_gap=0.1)
    for k in range(3):
        est.update(k * DT, k * DT, 0.0, 0.0)
    assert est.update(2 * DT, 0.0, 0.0, 0.0) is None          # duplicate stamp ignored
    assert est.update(1.0, 1.0, 0.0, 0.0) is None             # dropout -> window restarts
    assert est.update(1.0 + DT, 1.0 + DT, 0.0, 0.0) is None
    assert est.update(1.0 + 2 * DT, 1.0 + 2 * DT, 0.0, 0.0)[0] == pytest.approx(1.0)
