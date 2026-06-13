"""
Rear-axle kinematic bicycle motion model.

State x = (x, y, theta), control u = (v, delta), wheelbase L:

    xdot     = v * cos(theta)
    ydot     = v * sin(theta)
    thetadot = (v / L) * tan(delta)

This is the *model under test* for the offline noise-fitting task: the
sysid node generates commands and logs ground truth so that, offline,
one-step residuals e_l = x_{l+1} - [x_l + f(x_l, u_l) * dt] can be computed
against this exact model.

Pure Python/numpy, no rclpy dependency, so it can be imported by both the
ROS node and the offline bag_to_dataset tool.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class State:
    """Planar pose state (x, y, theta) in meters / radians."""

    x: float
    y: float
    theta: float


@dataclass
class Control:
    """Control input (v, delta): speed (m/s), steering angle (rad)."""

    v: float
    delta: float


@dataclass
class ModelParams:
    """
    Kinematic bicycle model parameters.

    ref_point: "rear_axle" (implemented) or "front_axle" (not yet
    implemented -- raises NotImplementedError; see TODO in README).
    """

    L: float
    ref_point: str = "rear_axle"
    delta_max: float = 0.4189
    delta_dot_max: float = 3.2


def _check_ref_point(params: ModelParams) -> None:
    if params.ref_point != "rear_axle":
        raise NotImplementedError(
            f"ref_point={params.ref_point!r} is not implemented; only "
            "'rear_axle' is supported. front_axle differs only in "
            "bookkeeping per the sysid spec but has not been implemented "
            "(TODO)."
        )


def dynamics(state: State, control: Control, params: ModelParams) -> State:
    """Continuous-time state derivative xdot = f(x, u) (rear-axle model)."""
    _check_ref_point(params)
    v = control.v
    theta = state.theta
    return State(
        x=v * math.cos(theta),
        y=v * math.sin(theta),
        theta=(v / params.L) * math.tan(control.delta),
    )


def step(state: State, control: Control, dt: float, params: ModelParams) -> State:
    """Advance `state` by `dt` under piecewise-constant `control` (RK4)."""
    _check_ref_point(params)

    def deriv(s: State) -> State:
        return dynamics(s, control, params)

    k1 = deriv(state)
    k2 = deriv(State(
        state.x + 0.5 * dt * k1.x,
        state.y + 0.5 * dt * k1.y,
        state.theta + 0.5 * dt * k1.theta,
    ))
    k3 = deriv(State(
        state.x + 0.5 * dt * k2.x,
        state.y + 0.5 * dt * k2.y,
        state.theta + 0.5 * dt * k2.theta,
    ))
    k4 = deriv(State(
        state.x + dt * k3.x,
        state.y + dt * k3.y,
        state.theta + dt * k3.theta,
    ))

    return State(
        x=state.x + (dt / 6.0) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
        y=state.y + (dt / 6.0) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
        theta=state.theta + (dt / 6.0) * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta),
    )


def curvature(delta: float, L: float) -> float:
    """Compute curvature kappa = tan(delta) / L."""
    return math.tan(delta) / L


def steering_from_curvature(kappa: float, L: float) -> float:
    """Compute steering angle delta = atan(L * kappa)."""
    return math.atan(L * kappa)


def lateral_accel(v: float, kappa: float) -> float:
    """Lateral acceleration estimate v^2 * kappa."""
    return v ** 2 * kappa


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Compute yaw (rotation about +z) from a geometry_msgs/Quaternion.

    Avoids a tf_transformations dependency in either the ROS node or
    bag_to_dataset.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def jacobian_df_du(state: State, control: Control, params: ModelParams) -> np.ndarray:
    """
    Analytic d(f)/d(u), a 3x2 array with columns [dv, ddelta].

    f is the continuous-time dynamics xdot = f(x, u); B_l = jacobian_df_du
    * dt is the control-channel Jacobian used by the (future) offline
    orthogonal-complement diagnostic.
    """
    _check_ref_point(params)
    theta = state.theta
    delta = control.delta
    v = control.v
    L = params.L
    sec2_delta = 1.0 / math.cos(delta) ** 2
    return np.array([
        [math.cos(theta), 0.0],
        [math.sin(theta), 0.0],
        [math.tan(delta) / L, v * sec2_delta / L],
    ])
