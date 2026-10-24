import math

import numpy as np
import pandas as pd
import pytest

from mrs_roboracer_sysid.fit_input_noise import fit_zero_mean_gaussian, select_channels
from mrs_roboracer_sysid.segments import compute_residuals, segment_table

L = 0.33
DT = 0.02


def _simulate(segments, w, v0=1.5, delta0=0.0, lag_s=0.0, sub=10):
    """
    Ground-truth rate-input bicycle with per-segment held noise `w`.

    segments: list of (u1, u2, n_steps). Returns (per-step table, pose_t,
    v, yawrate) with v/yawrate sampled at DT/sub, response delayed by lag_s.
    """
    rows, t_cmd, seg_id = [], 0.0, 0
    v_cmd, d_cmd = v0, delta0
    traj_t, traj_v, traj_w = [], [], []
    v, d, t = v0, delta0, 0.0
    h = DT / sub
    for (u1, u2, n), (w1, w2) in zip(segments, w):
        for _ in range(n):
            v_cmd += u1 * DT
            d_cmd += u2 * DT
            rows.append({"path_id": 0, "segment_id": seg_id, "t_cmd": t_cmd, "dt_cmd": DT,
                         "u1_cmd": u1, "u2_cmd": u2, "v_cmd": v_cmd, "delta_cmd": d_cmd,
                         "flag_stale_pose": False, "flag_safety_override": False})
            for _ in range(sub):
                traj_t.append(t + lag_s)
                traj_v.append(v)
                traj_w.append(v * math.tan(d) / L)
                v += (u1 + w1) * h
                d += (u2 + w2) * h
                t += h
            t_cmd += DT
        seg_id += 1
    traj_t.append(t + lag_s)
    traj_v.append(v)
    traj_w.append(v * math.tan(d) / L)
    return pd.DataFrame(rows), np.array(traj_t), np.array(traj_v), np.array(traj_w)


def test_segment_table_reconstructs_held_segments():
    steps, *_ = _simulate([(1.5, 0.0, 15), (0.0, 1.5, 20)], [(0, 0), (0, 0)])
    seg = segment_table(steps)
    assert list(seg["n_steps"]) == [15, 20]
    assert seg["T"].tolist() == pytest.approx([0.3, 0.4])
    assert seg["v_cmd_start"].iloc[0] == pytest.approx(1.5)
    assert seg["delta_cmd_start"].iloc[1] == pytest.approx(0.0)
    assert seg["delta_cmd_end"].iloc[1] == pytest.approx(1.5 * 0.4)


def test_noise_free_residuals_are_zero():
    steps, t, v, w = _simulate([(1.5, 0.0, 15), (0.0, 1.5, 20), (-1.5, -3.0, 15)],
                               [(0, 0)] * 3)
    res = compute_residuals(segment_table(steps), t, v, w, L=L)
    assert res["accel_resid"].abs().max() < 1e-6
    assert res["steer_rate_resid"].abs().max() < 1e-3
    assert res["steer_rate_resid_cmd"].abs().max() < 1e-3


def test_recovers_held_input_noise():
    segs = [(1.5, 0.0, 15), (0.0, 1.5, 20), (-1.5, -1.5, 20), (0.0, 3.0, 10)]
    noise = [(0.3, 0.0), (-0.2, 0.25), (0.1, -0.3), (0.0, 0.2)]
    steps, t, v, w = _simulate(segs, noise)
    res = compute_residuals(segment_table(steps), t, v, w, L=L)
    assert res["accel_resid"].to_numpy() == pytest.approx([n[0] for n in noise], abs=1e-6)
    # first-order steering inference; small residual curvature error is expected
    assert res["steer_rate_resid"].to_numpy() == pytest.approx([n[1] for n in noise], abs=0.03)
    # the commanded-angle variant accumulates earlier segments' steering error
    assert abs(res["steer_rate_resid_cmd"].iloc[2] - noise[2][1]) > 0.1


def test_lag_shift_realigns_delayed_response():
    segs = [(1.5, 0.0, 15), (0.0, 1.5, 20)]
    steps, t, v, w = _simulate(segs, [(0, 0)] * 2, lag_s=0.06)
    unshifted = compute_residuals(segment_table(steps), t, v, w, L=L)
    shifted = compute_residuals(segment_table(steps), t, v, w, L=L, lag_s=0.06)
    assert unshifted["accel_resid"].abs().max() > 0.1
    assert shifted["accel_resid"].abs().max() < 1e-6


def test_arc_gate_and_channel_selection():
    steps, t, v, w = _simulate([(0.0, 1.5, 10), (0.0, -1.5, 10)], [(0, 0)] * 2, v0=0.6)
    res = compute_residuals(segment_table(steps), t, v, w, L=L, min_arc_m=0.2)
    assert not res["gate_arc"].any()  # 0.6 m/s * 0.2 s = 0.12 m
    r_accel, r_steer, counts = select_channels(res)
    assert len(r_accel) == 2 and len(r_steer) == 0
    assert counts["steer_segments_gated"] == 0


def test_override_taints_rest_of_program():
    steps, *_ = _simulate([(1.5, 0.0, 10), (0.0, 0.0, 10), (-1.5, 0.0, 10)], [(0, 0)] * 3)
    steps.loc[(steps["segment_id"] == 1) & (steps.index % 10 == 3), "flag_safety_override"] = True
    seg = segment_table(steps)
    assert seg["flag_safety_override"].tolist() == [False, True, False]
    assert seg["flag_after_override"].tolist() == [False, True, True]


def test_input_dependence_flags_lag_like_residuals():
    from mrs_roboracer_sysid.fit_input_noise import input_dependence

    u = np.tile([-3.0, -1.5, 0.0, 1.5, 3.0], 20)
    lagged = -0.2 * u + np.random.default_rng(1).normal(0, 0.05, u.size)
    dep = input_dependence(lagged, u)
    assert dep["corr_with_u"] < -0.9
    assert dep["slope_vs_u"] == pytest.approx(-0.2, abs=0.02)
    assert abs(np.mean(lagged)) < 0.05  # invisible to the mean
    assert input_dependence(np.ones(5), np.zeros(5))["corr_with_u"] is None


def test_fit_zero_mean_gaussian():
    rng = np.random.default_rng(0)
    r = rng.normal(0.0, 0.3, size=20000)
    fit = fit_zero_mean_gaussian(np.append(r, np.nan))
    assert fit["n"] == 20000
    assert fit["sigma"] == pytest.approx(0.3, rel=0.02)
    assert fit["truncation"] == pytest.approx([-3 * fit["sigma"], 3 * fit["sigma"]])
    assert abs(fit["mean"]) < 0.01
    assert fit_zero_mean_gaussian(np.array([]))["n"] == 0
