import math
from dataclasses import dataclass

import numpy as np
import pytest

from mrs_roboracer_sysid.motion_model import ModelParams, curvature, steering_from_curvature
from mrs_roboracer_sysid.spline_paths import (
    GeneratedPath,
    arclength,
    assign_speed_profile,
    check_feasibility,
    curvature_profile,
    fit_spline,
    generate_path,
    invert_to_commands,
    sample_waypoints,
    sample_generated_path,
    save_generated_paths,
)


@dataclass
class _FakeCoverageTarget:
    profile_type: str
    kappa_center: float = 0.0
    v_center: float = 1.0
    v_idx: int = 0
    kappa_idx: int = 0


def _base_config():
    return {
        "model": {"L": 0.33, "ref_point": "rear_axle", "delta_max": 0.4189, "delta_dot_max": 3.2},
        "limits": {"v_min": 0.5, "v_max": 3.0, "a_max": 2.0,
                   "a_lat_max_safe": 4.0, "a_lat_probe_max": 8.0},
        "paths": {
            "waypoints_per_path": 6,
            "spline_order": 5,
            "workspace_bounds": [-5.0, 5.0, -5.0, 5.0],
            "max_generation_attempts": 50,
            "check_self_intersection": False,
        },
        "coverage": {"figure8_radius": 1.5},
        "rate": {"control_rate_hz": 50.0},
    }


def _circular_arc_waypoints(radius, n_waypoints=6, span=math.pi / 2.0):
    angles = np.linspace(0.0, span, n_waypoints)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    return np.column_stack([x, y])


def test_fit_spline_curvature_matches_circular_arc():
    radius = 2.0
    waypoints = _circular_arc_waypoints(radius, n_waypoints=6)
    spline = fit_spline(waypoints, order=3)

    u_grid = np.linspace(0.0, 1.0, 50)
    kappa = curvature_profile(spline, u_grid)

    # interior points (avoid spline-end artifacts)
    interior = kappa[5:-5]
    np.testing.assert_allclose(interior, 1.0 / radius, rtol=0.05)


def test_arclength_matches_known_arc_length():
    radius = 2.0
    span = math.pi / 2.0
    waypoints = _circular_arc_waypoints(radius, n_waypoints=6, span=span)
    spline = fit_spline(waypoints, order=3)

    _, s_grid = arclength(spline, n_samples=500)
    expected_length = radius * span
    assert s_grid[-1] == pytest.approx(expected_length, rel=0.02)
    assert np.all(np.diff(s_grid) >= 0)


def test_check_feasibility_ok_and_violation():
    params = ModelParams(L=0.33, delta_max=0.4189)
    kappa_max = curvature(params.delta_max, params.L)

    wide_arc = fit_spline(_circular_arc_waypoints(radius=2.0), order=3)
    feas_ok = check_feasibility(wide_arc, params, kappa_max)
    assert feas_ok.ok

    tight_arc = fit_spline(_circular_arc_waypoints(radius=0.3), order=3)
    feas_bad = check_feasibility(tight_arc, params, kappa_max)
    assert not feas_bad.ok
    assert "kappa" in feas_bad.reason


def test_assign_speed_profile_respects_a_max():
    s_grid = np.linspace(0.0, 10.0, 100)
    kappa_s = np.zeros_like(s_grid)
    v_min, v_max, a_max = 0.5, 3.0, 2.0

    v = assign_speed_profile(s_grid, kappa_s, "speed_sweep", v_min, v_max,
                             a_max, a_lat_max_safe=4.0, a_lat_probe_max=8.0)

    ds = np.diff(s_grid)
    dv2 = np.diff(v ** 2)
    assert np.all(dv2 <= 2.0 * a_max * ds + 1e-6)
    assert np.all(v > 0.0)


def test_assign_speed_profile_corner_probe_allows_higher_lat_accel():
    s_grid = np.linspace(0.0, 10.0, 100)
    kappa_s = np.full_like(s_grid, 1.0)  # tight, constant curvature
    v_min, v_max = 0.5, 3.0

    v_safe = assign_speed_profile(s_grid, kappa_s, "curvature_sweep", v_min, v_max,
                                  a_max=2.0, a_lat_max_safe=4.0, a_lat_probe_max=8.0)
    v_probe = assign_speed_profile(s_grid, kappa_s, "corner_probe", v_min, v_max,
                                   a_max=2.0, a_lat_max_safe=4.0, a_lat_probe_max=8.0)

    # corner_probe is allowed to reach higher lateral accel -> higher speed
    assert np.max(v_probe) >= np.max(v_safe)


def test_invert_to_commands_constant_curvature_path():
    radius = 2.0
    waypoints = _circular_arc_waypoints(radius, n_waypoints=6)
    spline = fit_spline(waypoints, order=3)
    params = ModelParams(L=0.33, delta_max=0.4189, delta_dot_max=3.2)

    u_grid, s_grid = arclength(spline, n_samples=200)
    kappa_s = curvature_profile(spline, u_grid)
    v_of_s = assign_speed_profile(s_grid, kappa_s, "curvature_sweep",
                                  v_min=0.5, v_max=3.0, a_max=2.0,
                                  a_lat_max_safe=4.0, a_lat_probe_max=8.0)

    commands = invert_to_commands(spline, u_grid, s_grid, v_of_s,
                                  control_rate_hz=50.0, params=params, a_lat_max_safe=4.0)

    assert commands is not None
    assert len(commands) > 1

    kappas = np.array([c.kappa for c in commands])
    deltas = np.array([c.delta for c in commands])
    expected_delta = steering_from_curvature(1.0 / radius, params.L)

    np.testing.assert_allclose(kappas[5:-5], 1.0 / radius, rtol=0.06)
    np.testing.assert_allclose(deltas[5:-5], expected_delta, rtol=0.06)

    delta_dot = np.diff(deltas) / commands[0].dt
    assert np.max(np.abs(delta_dot)) <= params.delta_dot_max + 1e-6


def test_generate_path_speed_sweep_returns_valid_path():
    rng = np.random.default_rng(12345)
    config = _base_config()
    target = _FakeCoverageTarget(profile_type="speed_sweep", kappa_center=0.0)

    result = generate_path(path_id=0, seed=12345, rng=rng, config=config, coverage_target=target)

    assert result is not None
    assert isinstance(result, GeneratedPath)
    assert result.path_id == 0
    assert len(result.commands) > 1
    assert result.waypoints.shape == (config["paths"]["waypoints_per_path"], 2)


def test_generate_path_corner_probe_uses_arc_waypoints():
    rng = np.random.default_rng(7)
    config = _base_config()
    target = _FakeCoverageTarget(profile_type="corner_probe", kappa_center=1.0)

    result = generate_path(path_id=1, seed=7, rng=rng, config=config, coverage_target=target)

    assert result is not None
    kappas = np.array([c.kappa for c in result.commands])
    np.testing.assert_allclose(np.mean(kappas), 1.0, rtol=0.2)


def test_sample_generated_path_uses_stored_spline_geometry():
    rng = np.random.default_rng(123)
    config = _base_config()
    target = _FakeCoverageTarget(profile_type="speed_sweep", kappa_center=0.0)
    result = generate_path(path_id=3, seed=123, rng=rng, config=config, coverage_target=target)
    assert result is not None

    samples = sample_generated_path(result, n_samples=25)

    assert samples.shape == (25, 3)
    np.testing.assert_allclose(samples[0, :2], result.waypoints[0], atol=1e-8)
    np.testing.assert_allclose(samples[-1, :2], result.waypoints[-1], atol=1e-8)
    assert np.all(np.isfinite(samples))


def test_save_generated_paths_roundtrip(tmp_path):
    rng = np.random.default_rng(99)
    config = _base_config()
    target = _FakeCoverageTarget(profile_type="speed_sweep", kappa_center=0.0)
    result = generate_path(path_id=2, seed=99, rng=rng, config=config, coverage_target=target)
    assert result is not None

    out_dir = tmp_path / "paths"
    save_generated_paths([result], str(out_dir))

    loaded = np.load(out_dir / "path_002.npz")
    assert loaded["path_id"] == 2
    assert loaded["commands"].shape[0] == len(result.commands)
    np.testing.assert_allclose(loaded["waypoints"], result.waypoints)


def test_sample_waypoints_within_bounds():
    rng = np.random.default_rng(0)
    bounds = (-5.0, 5.0, -2.0, 2.0)
    waypoints = sample_waypoints(rng, n_waypoints=6, workspace_bounds=bounds)
    assert waypoints.shape == (6, 2)
    assert np.all(waypoints[:, 0] >= bounds[0]) and np.all(waypoints[:, 0] <= bounds[1])
    assert np.all(waypoints[:, 1] >= bounds[2]) and np.all(waypoints[:, 1] <= bounds[3])
