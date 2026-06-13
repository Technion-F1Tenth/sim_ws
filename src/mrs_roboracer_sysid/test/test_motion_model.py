import math

import numpy as np
import pytest

from mrs_roboracer_sysid.motion_model import (
    Control,
    ModelParams,
    State,
    curvature,
    jacobian_df_du,
    lateral_accel,
    quaternion_to_yaw,
    step,
    steering_from_curvature,
)


def test_straight_line_exact():
    """With delta=0 the derivative is constant, so RK4 is exact."""
    params = ModelParams(L=0.33)
    state = State(x=0.0, y=0.0, theta=0.0)
    control = Control(v=2.0, delta=0.0)
    dt = 0.1

    out = step(state, control, dt, params)

    assert out.x == pytest.approx(2.0 * dt)
    assert out.y == pytest.approx(0.0)
    assert out.theta == pytest.approx(0.0)


def test_constant_curvature_arc_matches_analytic():
    """For constant (v, delta) the state traces a circular arc."""
    params = ModelParams(L=0.33)
    v = 1.5
    delta = 0.2
    L = params.L
    kappa = curvature(delta, L)
    omega = v * kappa

    state = State(x=0.0, y=0.0, theta=0.0)
    control = Control(v=v, delta=delta)
    dt = 0.02

    out = step(state, control, dt, params)

    theta_expected = omega * dt
    x_expected = (v / omega) * (math.sin(theta_expected) - math.sin(0.0))
    y_expected = -(v / omega) * (math.cos(theta_expected) - math.cos(0.0))

    assert out.theta == pytest.approx(theta_expected, abs=1e-9)
    assert out.x == pytest.approx(x_expected, abs=1e-9)
    assert out.y == pytest.approx(y_expected, abs=1e-9)


def test_curvature_and_steering_roundtrip():
    L = 0.33
    delta = 0.15
    kappa = curvature(delta, L)
    assert steering_from_curvature(kappa, L) == pytest.approx(delta)


def test_lateral_accel():
    assert lateral_accel(2.0, 0.5) == pytest.approx(2.0)


def test_jacobian_df_du_matches_finite_difference():
    params = ModelParams(L=0.33)
    state = State(x=1.0, y=-0.5, theta=0.4)
    control = Control(v=1.2, delta=0.1)
    eps = 1e-6

    analytic = jacobian_df_du(state, control, params)

    from mrs_roboracer_sysid.motion_model import dynamics

    base = dynamics(state, control, params)

    dv_plus = dynamics(state, Control(v=control.v + eps, delta=control.delta), params)
    dv_minus = dynamics(state, Control(v=control.v - eps, delta=control.delta), params)
    col_v = np.array([
        (dv_plus.x - dv_minus.x) / (2 * eps),
        (dv_plus.y - dv_minus.y) / (2 * eps),
        (dv_plus.theta - dv_minus.theta) / (2 * eps),
    ])

    dd_plus = dynamics(state, Control(v=control.v, delta=control.delta + eps), params)
    dd_minus = dynamics(state, Control(v=control.v, delta=control.delta - eps), params)
    col_delta = np.array([
        (dd_plus.x - dd_minus.x) / (2 * eps),
        (dd_plus.y - dd_minus.y) / (2 * eps),
        (dd_plus.theta - dd_minus.theta) / (2 * eps),
    ])

    np.testing.assert_allclose(analytic[:, 0], col_v, atol=1e-6)
    np.testing.assert_allclose(analytic[:, 1], col_delta, atol=1e-6)

    # base is unused beyond sanity that dynamics() returns a State
    assert isinstance(base, State)


def test_quaternion_to_yaw():
    # identity quaternion -> yaw 0
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)

    # 90 deg yaw: qz=sin(pi/4), qw=cos(pi/4)
    yaw = math.pi / 2.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    assert quaternion_to_yaw(0.0, 0.0, qz, qw) == pytest.approx(yaw)

    # -90 deg yaw
    yaw = -math.pi / 2.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    assert quaternion_to_yaw(0.0, 0.0, qz, qw) == pytest.approx(yaw)


def test_front_axle_not_implemented():
    params = ModelParams(L=0.33, ref_point="front_axle")
    state = State(x=0.0, y=0.0, theta=0.0)
    control = Control(v=1.0, delta=0.0)

    with pytest.raises(NotImplementedError):
        step(state, control, 0.02, params)
