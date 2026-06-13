"""
Random feasible spline path generation and command inversion.

Per the sysid spec §3: generate a random, feasible xy spline inside the
capture volume, then invert it to a piecewise-constant (v, delta) command
sequence via the rear-axle kinematic bicycle model in `motion_model`.

`generate_path()` expects a `coverage_target` object (typically a
`coverage.CoverageBin`, duck-typed here to avoid a circular import) with
attributes: `profile_type` (one of "speed_sweep", "curvature_sweep",
"figure8", "corner_probe"), `kappa_center` (float), `v_center` (float).

Pure Python/numpy/scipy, no rclpy dependency.
"""

from dataclasses import dataclass
import math
import os
from typing import List, Optional, Tuple

import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.interpolate import make_interp_spline

from mrs_roboracer_sysid.motion_model import ModelParams, curvature, steering_from_curvature


@dataclass
class ParametricSpline:
    """x(u), y(u) parametric spline with analytic 1st/2nd derivatives."""

    x_spl: object
    y_spl: object

    def x(self, u):
        return self.x_spl(u)

    def y(self, u):
        return self.y_spl(u)

    def dx(self, u):
        return self.x_spl.derivative(1)(u)

    def dy(self, u):
        return self.y_spl.derivative(1)(u)

    def ddx(self, u):
        return self.x_spl.derivative(2)(u)

    def ddy(self, u):
        return self.y_spl.derivative(2)(u)


@dataclass
class FeasibilityResult:
    ok: bool
    reason: str = ""


@dataclass
class CommandStep:
    """One piecewise-constant (v, delta) command step."""

    step_index: int
    v: float
    delta: float
    kappa: float
    lat_accel: float
    flag_high_lat_accel: bool
    dt: float
    flag_sync_chirp: bool = False


@dataclass
class GeneratedPath:
    path_id: int
    seed: int
    waypoints: np.ndarray
    spline_coeffs: dict
    commands: List[CommandStep]
    coverage_target: dict
    sync_chirp_steps: int = 0


# ---------------------------------------------------------------------------
# Waypoint sampling
# ---------------------------------------------------------------------------

def sample_waypoints(rng: np.random.Generator, n_waypoints: int,
                     workspace_bounds: Tuple[float, float, float, float]) -> np.ndarray:
    """Uniform-random waypoints inside `workspace_bounds = (x_min,x_max,y_min,y_max)`."""
    x_min, x_max, y_min, y_max = workspace_bounds
    x = rng.uniform(x_min, x_max, size=n_waypoints)
    y = rng.uniform(y_min, y_max, size=n_waypoints)
    return np.column_stack([x, y])


def sample_arc_waypoints(rng: np.random.Generator, n_waypoints: int, radius: float,
                         workspace_bounds: Tuple[float, float, float, float]) -> np.ndarray:
    """
    Waypoints along a circular arc of the given radius.

    The arc center is randomized within the workspace (shrunk by `radius`
    where possible); arc span and direction (CW/CCW) are randomized too.
    Used for speed_sweep / curvature_sweep / corner_probe profiles, whose
    coverage target pins a desired curvature (`1/radius`).
    """
    x_min, x_max, y_min, y_max = workspace_bounds
    arc_span = rng.uniform(math.pi / 3.0, math.pi)
    direction = rng.choice([-1.0, 1.0])
    start_angle = rng.uniform(0.0, 2.0 * math.pi)
    angles = start_angle + direction * np.linspace(0.0, arc_span, n_waypoints)

    half_w = max((x_max - x_min) / 2.0 - radius, 0.0)
    half_h = max((y_max - y_min) / 2.0 - radius, 0.0)
    cx = (x_min + x_max) / 2.0 + (rng.uniform(-half_w, half_w) if half_w > 0 else 0.0)
    cy = (y_min + y_max) / 2.0 + (rng.uniform(-half_h, half_h) if half_h > 0 else 0.0)

    x = cx + radius * np.cos(angles)
    y = cy + radius * np.sin(angles)
    return np.column_stack([x, y])


def sample_figure8_waypoints(rng: np.random.Generator, n_waypoints: int, radius: float,
                             workspace_bounds: Tuple[float, float, float, float]) -> np.ndarray:
    """Waypoints along a figure-8 (lemniscate of Gerono) of the given lobe radius."""
    x_min, x_max, y_min, y_max = workspace_bounds
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0
    rotation = rng.uniform(0.0, 2.0 * math.pi)
    t = np.linspace(0.0, 2.0 * math.pi, n_waypoints, endpoint=False)
    x0 = radius * np.sin(t)
    y0 = radius * np.sin(t) * np.cos(t)
    x = cx + x0 * math.cos(rotation) - y0 * math.sin(rotation)
    y = cy + x0 * math.sin(rotation) + y0 * math.cos(rotation)
    return np.column_stack([x, y])


# ---------------------------------------------------------------------------
# Spline fit / geometry
# ---------------------------------------------------------------------------

def fit_spline(waypoints: np.ndarray, order: int) -> ParametricSpline:
    """
    Fit a chord-length-parameterized x(u), y(u) spline through waypoints.

    `order` is reduced to `len(waypoints) - 1` if there are too few points
    for the requested degree.
    """
    n = len(waypoints)
    k = min(order, n - 1)
    if k < 1:
        raise ValueError("need at least 2 waypoints to fit a spline")

    diffs = np.diff(waypoints, axis=0)
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    if np.any(seg_len <= 0):
        raise ValueError("duplicate consecutive waypoints")
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    u = cum / cum[-1]

    x_spl = make_interp_spline(u, waypoints[:, 0], k=k)
    y_spl = make_interp_spline(u, waypoints[:, 1], k=k)
    return ParametricSpline(x_spl=x_spl, y_spl=y_spl)


def arclength(spline: ParametricSpline, u_max: float = 1.0,
              n_samples: int = 500) -> Tuple[np.ndarray, np.ndarray]:
    """Return (u_grid, s_grid): cumulative arc length over u in [0, u_max]."""
    u_grid = np.linspace(0.0, u_max, n_samples)
    speed = np.hypot(spline.dx(u_grid), spline.dy(u_grid))
    s_grid = cumulative_trapezoid(speed, u_grid, initial=0.0)
    return u_grid, s_grid


def curvature_profile(spline: ParametricSpline, u) -> np.ndarray:
    """kappa(u) = (x'y'' - y'x'') / (x'^2 + y'^2)^1.5."""
    u = np.atleast_1d(u)
    dx, dy = spline.dx(u), spline.dy(u)
    ddx, ddy = spline.ddx(u), spline.ddy(u)
    denom = np.power(dx ** 2 + dy ** 2, 1.5)
    denom = np.where(denom < 1e-12, 1e-12, denom)
    return (dx * ddy - dy * ddx) / denom


def _segments_intersect(p1, p2, p3, p4) -> bool:
    """Return True if segment p1-p2 properly intersects segment p3-p4."""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _has_self_intersection(x: np.ndarray, y: np.ndarray) -> bool:
    points = np.column_stack([x, y])
    n = len(points)
    for i in range(n - 1):
        for j in range(i + 2, n - 1):
            if i == 0 and j == n - 2:
                continue  # adjacent at the closure point only for closed paths
            if _segments_intersect(points[i], points[i + 1], points[j], points[j + 1]):
                return True
    return False


def check_feasibility(spline: ParametricSpline, params: ModelParams, kappa_max: float,
                      n_samples: int = 500,
                      check_self_intersection: bool = False) -> FeasibilityResult:
    """ok=False if curvature exceeds `kappa_max`, or (optionally) the path self-intersects."""
    u_grid = np.linspace(0.0, 1.0, n_samples)
    kappa = curvature_profile(spline, u_grid)
    max_kappa = float(np.max(np.abs(kappa)))
    if max_kappa > kappa_max:
        return FeasibilityResult(
            ok=False,
            reason=f"max|kappa|={max_kappa:.3f} > kappa_max={kappa_max:.3f}")

    if check_self_intersection:
        x, y = spline.x(u_grid), spline.y(u_grid)
        if _has_self_intersection(x, y):
            return FeasibilityResult(ok=False, reason="path self-intersects")

    return FeasibilityResult(ok=True)


# ---------------------------------------------------------------------------
# Speed profile + inversion
# ---------------------------------------------------------------------------

_V_FLOOR = 0.1


def assign_speed_profile(s_grid: np.ndarray, kappa_s: np.ndarray, profile_type: str,
                         v_min: float, v_max: float, a_max: float,
                         a_lat_max_safe: float, a_lat_probe_max: float) -> np.ndarray:
    """
    Build v(s) for the requested profile, then enforce |a| <= a_max.

    `corner_probe` is permitted to reach `a_lat_probe_max`; all other
    profiles are limited to `a_lat_max_safe`.
    """
    n = len(s_grid)
    a_lat_limit = a_lat_probe_max if profile_type == "corner_probe" else a_lat_max_safe

    kappa_abs = np.abs(kappa_s)
    with np.errstate(divide='ignore'):
        v_lat = np.where(kappa_abs > 1e-6,
                         np.sqrt(a_lat_limit / np.maximum(kappa_abs, 1e-12)), v_max)

    if profile_type == "speed_sweep":
        v_base = np.linspace(v_min, v_max, n)
    elif profile_type == "corner_probe":
        v_base = np.full(n, v_max)
    else:  # curvature_sweep, figure8, default
        v_base = np.full(n, 0.5 * (v_min + v_max))

    v_target = np.clip(np.minimum(v_base, v_lat), _V_FLOOR, v_max)

    # Two-pass kinematic smoothing: v_{i+1}^2 <= v_i^2 + 2*a_max*ds
    ds = np.diff(s_grid)
    v = v_target.copy()
    for i in range(1, n):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_max * ds[i - 1]))
    for i in range(n - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_max * ds[i]))

    return np.maximum(v, _V_FLOOR)


def invert_to_commands(spline: ParametricSpline, u_grid: np.ndarray, s_grid: np.ndarray,
                       v_of_s: np.ndarray, control_rate_hz: float, params: ModelParams,
                       a_lat_max_safe: float) -> Optional[List[CommandStep]]:
    """
    Invert the spline+speed-profile to a fixed-dt (v, delta) command sequence.

    Returns None if the resampled sequence violates `params.delta_dot_max`
    (signal to `generate_path` to reject and regenerate) or if the path is
    too short to produce a single control step.
    """
    ds = np.diff(s_grid)
    v_mid = 0.5 * (v_of_s[:-1] + v_of_s[1:])
    dt_seg = ds / v_mid
    t_grid = np.concatenate([[0.0], np.cumsum(dt_seg)])
    t_total = t_grid[-1]

    dt = 1.0 / control_rate_hz
    n_steps = int(math.floor(t_total / dt))
    if n_steps < 1:
        return None

    t_cmd = np.arange(n_steps) * dt
    v_cmd = np.interp(t_cmd, t_grid, v_of_s)
    s_cmd = np.interp(t_cmd, t_grid, s_grid)
    u_cmd = np.interp(s_cmd, s_grid, u_grid)

    dx, dy = spline.dx(u_cmd), spline.dy(u_cmd)
    theta_cmd = np.arctan2(dy, dx)  # available for diagnostics; not stored on CommandStep
    kappa_cmd = curvature_profile(spline, u_cmd)
    delta_cmd = np.array([
        np.clip(steering_from_curvature(k, params.L), -params.delta_max, params.delta_max)
        for k in kappa_cmd
    ])

    if n_steps >= 2:
        delta_dot = np.diff(delta_cmd) / dt
        if np.max(np.abs(delta_dot)) > params.delta_dot_max:
            return None

    lat_accel = v_cmd ** 2 * kappa_cmd
    flag_high = np.abs(lat_accel) > a_lat_max_safe

    commands = []
    for i in range(n_steps):
        commands.append(CommandStep(
            step_index=i,
            v=float(v_cmd[i]),
            delta=float(delta_cmd[i]),
            kappa=float(kappa_cmd[i]),
            lat_accel=float(lat_accel[i]),
            flag_high_lat_accel=bool(flag_high[i]),
            dt=dt,
        ))
    # theta_cmd retained for potential future debugging use; not part of CommandStep.
    del theta_cmd
    return commands


# ---------------------------------------------------------------------------
# Top-level orchestration
# ---------------------------------------------------------------------------

def _sample_waypoints_for_profile(rng: np.random.Generator, n_waypoints: int, profile_type: str,
                                  kappa_center: float, workspace_bounds,
                                  coverage_config: dict) -> np.ndarray:
    if profile_type == "figure8":
        radius = coverage_config.get("figure8_radius", 1.5)
        return sample_figure8_waypoints(rng, n_waypoints, radius, workspace_bounds)
    profiles_with_kappa = ("speed_sweep", "curvature_sweep", "corner_probe")
    if profile_type in profiles_with_kappa and abs(kappa_center) > 1e-6:
        radius = 1.0 / abs(kappa_center)
        return sample_arc_waypoints(rng, n_waypoints, radius, workspace_bounds)
    return sample_waypoints(rng, n_waypoints, workspace_bounds)


def generate_path(path_id: int, seed: int, rng: np.random.Generator, config: dict,
                  coverage_target) -> Optional[GeneratedPath]:
    """
    Generate one feasible path for `coverage_target`, or None after exhausting attempts.

    `config` is the `sysid_data_collector.ros__parameters` dict (model,
    limits, paths, coverage, rate sections). `coverage_target` is duck-typed
    (e.g. `coverage.CoverageBin`) with `.profile_type`, `.kappa_center`.
    """
    model_cfg = config["model"]
    limits_cfg = config["limits"]
    paths_cfg = config["paths"]
    coverage_cfg = config.get("coverage", {})
    control_rate_hz = config["rate"]["control_rate_hz"]

    params = ModelParams(
        L=model_cfg["L"],
        ref_point=model_cfg.get("ref_point", "rear_axle"),
        delta_max=model_cfg["delta_max"],
        delta_dot_max=model_cfg["delta_dot_max"],
    )
    kappa_max = curvature(params.delta_max, params.L)

    workspace_bounds = paths_cfg["workspace_bounds"]
    n_waypoints = paths_cfg["waypoints_per_path"]
    spline_order = paths_cfg["spline_order"]
    max_attempts = paths_cfg.get("max_generation_attempts", 50)
    check_self_intersection = paths_cfg.get("check_self_intersection", False)

    v_min = limits_cfg["v_min"]
    v_max = limits_cfg["v_max"]
    a_max = limits_cfg["a_max"]
    a_lat_max_safe = limits_cfg["a_lat_max_safe"]
    a_lat_probe_max = limits_cfg["a_lat_probe_max"]

    profile_type = coverage_target.profile_type
    kappa_center = getattr(coverage_target, "kappa_center", 0.0)

    for _ in range(max_attempts):
        waypoints = _sample_waypoints_for_profile(
            rng, n_waypoints, profile_type, kappa_center, workspace_bounds, coverage_cfg)

        order = spline_order
        try:
            spline = fit_spline(waypoints, order)
        except ValueError:
            try:
                spline = fit_spline(waypoints, order=3)
                order = 3
            except ValueError:
                continue

        feas = check_feasibility(spline, params, kappa_max,
                                 check_self_intersection=check_self_intersection)
        if not feas.ok:
            continue

        u_grid, s_grid = arclength(spline)
        kappa_s = curvature_profile(spline, u_grid)
        v_of_s = assign_speed_profile(s_grid, kappa_s, profile_type, v_min, v_max,
                                      a_max, a_lat_max_safe, a_lat_probe_max)

        commands = invert_to_commands(spline, u_grid, s_grid, v_of_s, control_rate_hz,
                                      params, a_lat_max_safe)
        if commands is None:
            continue

        sync_chirp_steps = 0
        sync_cfg = config.get("sync", {})
        if sync_cfg.get("enable_sync_chirp", True):
            # deferred import: sync_chirp imports CommandStep from this module
            from mrs_roboracer_sysid.sync_chirp import prepend_sync_chirp
            n_before = len(commands)
            commands = prepend_sync_chirp(
                commands,
                chirp_amplitude=sync_cfg.get("chirp_amplitude", 0.2),
                chirp_duration=sync_cfg.get("chirp_duration", 0.5),
                control_rate_hz=control_rate_hz,
                chirp_speed=sync_cfg.get("chirp_speed", v_min),
                params=params,
                a_lat_max_safe=a_lat_max_safe,
            )
            sync_chirp_steps = len(commands) - n_before

        spline_coeffs = {
            "order": order,
            "x_t": np.asarray(spline.x_spl.t),
            "x_c": np.asarray(spline.x_spl.c),
            "y_t": np.asarray(spline.y_spl.t),
            "y_c": np.asarray(spline.y_spl.c),
        }
        coverage_target_dict = {
            "profile_type": profile_type,
            "kappa_center": kappa_center,
            "v_center": getattr(coverage_target, "v_center", None),
            "v_idx": getattr(coverage_target, "v_idx", None),
            "kappa_idx": getattr(coverage_target, "kappa_idx", None),
        }

        return GeneratedPath(
            path_id=path_id,
            seed=seed,
            waypoints=waypoints,
            spline_coeffs=spline_coeffs,
            commands=commands,
            coverage_target=coverage_target_dict,
            sync_chirp_steps=sync_chirp_steps,
        )

    return None


def save_generated_paths(paths: List[GeneratedPath], out_dir: str) -> None:
    """Persist each GeneratedPath to `<out_dir>/path_<NNN>.npz` for replay/debugging."""
    os.makedirs(out_dir, exist_ok=True)
    for p in paths:
        cmd_array = np.array([
            (c.step_index, c.v, c.delta, c.kappa, c.lat_accel,
             float(c.flag_high_lat_accel), c.dt, float(c.flag_sync_chirp))
            for c in p.commands
        ], dtype=np.float64)

        out_path = os.path.join(out_dir, f"path_{p.path_id:03d}.npz")
        np.savez(
            out_path,
            path_id=p.path_id,
            seed=p.seed,
            waypoints=p.waypoints,
            commands=cmd_array,
            sync_chirp_steps=p.sync_chirp_steps,
            spline_order=p.spline_coeffs["order"],
            spline_x_t=p.spline_coeffs["x_t"],
            spline_x_c=p.spline_coeffs["x_c"],
            spline_y_t=p.spline_coeffs["y_t"],
            spline_y_c=p.spline_coeffs["y_c"],
            coverage_profile_type=p.coverage_target["profile_type"],
            coverage_kappa_center=p.coverage_target["kappa_center"],
        )
