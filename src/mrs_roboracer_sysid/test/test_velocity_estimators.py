import math

import numpy as np
import pytest

from mrs_roboracer_sysid.velocity_estimators import (
    MocapTwistEstimator,
    OnboardOdomEstimator,
    PoseDerivativeEstimator,
    signed_forward_speed,
)


def _circular_trajectory(n, dt, v, omega):
    """x(t)=R*sin(omega t), y(t)=R*(1-cos(omega t)), theta(t)=omega t, R=v/omega."""
    t = np.arange(n) * dt
    R = v / omega
    x = R * np.sin(omega * t)
    y = R * (1.0 - np.cos(omega * t))
    theta = omega * t
    return t, x, y, theta


def test_pose_derivative_compute_batch_matches_known_velocity():
    dt = 0.02
    v_true = 1.5
    omega = 0.8
    n = 60
    t, x, y, theta = _circular_trajectory(n, dt, v_true, omega)

    v_est, thetadot_est = PoseDerivativeEstimator.compute_batch(
        t, x, y, theta, window=9, polyorder=2,
    )

    half = 9 // 2
    valid = slice(half, n - half)
    np.testing.assert_allclose(v_est[valid], v_true, atol=1e-2)
    np.testing.assert_allclose(thetadot_est[valid], omega, atol=1e-2)

    # edges are NaN (insufficient context for the filter window)
    assert np.all(np.isnan(v_est[:half]))
    assert np.all(np.isnan(v_est[n - half:]))


def test_pose_derivative_update_online_matches_known_velocity():
    dt = 0.02
    v_true = 1.5
    omega = 0.8
    window = 9
    t, x, y, theta = _circular_trajectory(window + 5, dt, v_true, omega)

    estimator = PoseDerivativeEstimator(window=window, polyorder=2)

    result = None
    for i in range(window):
        result = estimator.update(t[i], x[i], y[i], theta[i])

    assert result is not None
    v_est, thetadot_est = result
    assert v_est == pytest.approx(v_true, abs=1e-2)
    assert thetadot_est == pytest.approx(omega, abs=1e-2)


def test_pose_derivative_returns_none_until_buffer_full():
    estimator = PoseDerivativeEstimator(window=5, polyorder=2)
    for i in range(4):
        assert estimator.update(t=i * 0.1, x=float(i), y=0.0, theta=0.0) is None
    assert estimator.update(t=4 * 0.1, x=4.0, y=0.0, theta=0.0) is not None


def test_pose_derivative_invalid_window_raises():
    with pytest.raises(ValueError):
        PoseDerivativeEstimator(window=8, polyorder=2)  # even window
    with pytest.raises(ValueError):
        PoseDerivativeEstimator(window=3, polyorder=3)  # polyorder >= window


def test_signed_forward_speed():
    assert signed_forward_speed(vx=1.0, vy=0.0, theta=0.0) == pytest.approx(1.0)
    assert signed_forward_speed(vx=0.0, vy=1.0, theta=math.pi / 2) == pytest.approx(1.0)
    assert signed_forward_speed(vx=1.0, vy=0.0, theta=math.pi) == pytest.approx(-1.0)


def test_mocap_twist_estimator():
    v = MocapTwistEstimator.velocity(vx=2.0, vy=0.0, theta=0.0)
    assert v == pytest.approx(2.0)


def test_onboard_odom_estimator():
    assert OnboardOdomEstimator.velocity(vx_body=1.7) == pytest.approx(1.7)
