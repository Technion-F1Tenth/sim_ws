"""
Per-segment input-noise residuals for held-rate-step programs.

Each measurement segment holds (u1, u2) for T seconds (see `rate_steps.py`).
Residuals are one-segment predictions started from the *measured* state at
the segment start, so error accumulated in earlier segments cancels:

Acceleration channel (v is measured directly):
    r_v          = v_b - (v_a + u1 * T)
    accel_resid  = r_v / T                                    [m/s^2]

Steering-rate channel (steering angle is not measured; yaw rate is):
    delta_a      = atan(L * yawrate_a / v_a)    (inferred steering at start)
    delta_b_pred = delta_a + u2 * T
    r_yawrate    = yawrate_b - v_b * tan(delta_b_pred) / L    [rad/s]
    steer_rate_resid = L cos^2(delta_b_pred) / (v_b * T) * r_yawrate   [rad/s]

The `_cmd` variants use the commanded steering angle instead of the
inferred one (exposes steering trim/offset as bias). `arc_m` is the
measured mean speed times T; the steering channel is only trustworthy for
`arc_m >= min_arc_m` (`gate_arc`).

`_a`/`_b` quantities are measured at `t_start + lag_s` and
`t_start + T + lag_s`, linearly interpolated from the pose-derivative
streams; `lag_s` shifts the measurement window to account for
command-to-response delay (estimate it from the sync chirp).

Pure numpy/pandas, no rclpy dependency.
"""

import math

import numpy as np
import pandas as pd

_V_EPS = 0.05  # m/s; below this the steering inference is undefined


def segment_table(steps: pd.DataFrame) -> pd.DataFrame:
    """
    Collapse the per-step table to one row per (path_id, segment_id >= 0).

    Requires columns: path_id, segment_id, t_cmd, dt_cmd, u1_cmd, u2_cmd,
    v_cmd, delta_cmd, flag_stale_pose; uses flag_safety_override if present.
    The `*_cmd_start/end` columns describe the reference trajectory: taken
    from v_ref/delta_ref when present (they differ from the published
    v_cmd/delta_cmd under lag compensation), else from v_cmd/delta_cmd.

    `flag_after_override` marks every segment from the first overridden one
    to the end of its program: once safety_node has altered the command, the
    car's state no longer follows the setpoint ramp the later segments assume.
    """
    rows = []
    moving = steps[steps["segment_id"] >= 0].sort_values("t_cmd")
    v_col = "v_ref" if "v_ref" in steps else "v_cmd"
    d_col = "delta_ref" if "delta_ref" in steps else "delta_cmd"
    for (path_id, segment_id), g in moving.groupby(["path_id", "segment_id"], sort=True):
        first, last = g.iloc[0], g.iloc[-1]
        dt = float(first["dt_cmd"])
        u1, u2 = float(first["u1_cmd"]), float(first["u2_cmd"])
        rows.append({
            "path_id": int(path_id),
            "segment_id": int(segment_id),
            "n_steps": len(g),
            "t_start": float(first["t_cmd"]),
            "T": len(g) * dt,
            "u1_cmd": u1,
            "u2_cmd": u2,
            # Step i publishes the setpoint reached at the end of its tick.
            "v_cmd_start": float(first[v_col]) - u1 * dt,
            "v_cmd_end": float(last[v_col]),
            "delta_cmd_start": float(first[d_col]) - u2 * dt,
            "delta_cmd_end": float(last[d_col]),
            "flag_stale_pose": bool(g["flag_stale_pose"].any()),
            "flag_safety_override": bool(g["flag_safety_override"].any())
            if "flag_safety_override" in g else False,
            "flag_segment_aborted": bool(g["flag_segment_aborted"].any())
            if "flag_segment_aborted" in g else False,
        })
    seg = pd.DataFrame(rows)
    if not seg.empty:
        seg["flag_after_override"] = (
            seg.groupby("path_id")["flag_safety_override"].cummax().astype(bool))
    return seg


def mark_aborted(steps: pd.DataFrame, aborted) -> pd.DataFrame:
    """
    Add `flag_segment_aborted` to the per-step table.

    `aborted` is the list from the collector's `<bag_path>_aborted.json`
    (dicts with `path_id` and `segment_id`). Only the segment that was
    running when its program was aborted is flagged; it was cut short, so
    its T and end state no longer describe a full held-rate segment.
    """
    cut = {(int(a["path_id"]), int(a["segment_id"]))
           for a in aborted if int(a["segment_id"]) >= 0}
    steps = steps.copy()
    steps["flag_segment_aborted"] = [
        (int(p), int(s)) in cut for p, s in zip(steps["path_id"], steps["segment_id"])]
    return steps


def _interp(t_query: float, t: np.ndarray, y: np.ndarray, max_gap_s: float) -> float:
    """Linear interpolation over finite samples; NaN outside range or across a gap."""
    ok = np.isfinite(y)
    t, y = t[ok], y[ok]
    if len(t) < 2 or t_query < t[0] or t_query > t[-1]:
        return float("nan")
    i = int(np.searchsorted(t, t_query))
    i = min(max(i, 1), len(t) - 1)
    if t[i] - t[i - 1] > max_gap_s:
        return float("nan")
    return float(np.interp(t_query, t[i - 1:i + 1], y[i - 1:i + 1]))


def compute_residuals(segments: pd.DataFrame, pose_t: np.ndarray, v_meas: np.ndarray,
                      yawrate_meas: np.ndarray, L: float, lag_s: float = 0.0,
                      min_arc_m: float = 0.2, delta_max: float = 0.4189,
                      max_gap_s: float = 0.1) -> pd.DataFrame:
    """Add measured endpoints and channel residuals (module docstring) to `segments`."""
    pose_t = np.asarray(pose_t, dtype=float)
    v_meas = np.asarray(v_meas, dtype=float)
    yawrate_meas = np.asarray(yawrate_meas, dtype=float)

    out = []
    for _, seg in segments.iterrows():
        T = seg["T"]
        ta = seg["t_start"] + lag_s
        tb = ta + T
        v_a = _interp(ta, pose_t, v_meas, max_gap_s)
        v_b = _interp(tb, pose_t, v_meas, max_gap_s)
        w_a = _interp(ta, pose_t, yawrate_meas, max_gap_s)
        w_b = _interp(tb, pose_t, yawrate_meas, max_gap_s)
        u1, u2 = seg["u1_cmd"], seg["u2_cmd"]

        r_v = v_b - (v_a + u1 * T)
        arc = 0.5 * (v_a + v_b) * T

        nan = float("nan")
        moving = v_a > _V_EPS and v_b > _V_EPS
        delta_a = math.atan(L * w_a / v_a) if moving else nan
        delta_b_pred = delta_a + u2 * T
        r_w = w_b - v_b * math.tan(delta_b_pred) / L if moving else nan
        steer = L * math.cos(delta_b_pred) ** 2 / (v_b * T) * r_w if moving else nan

        delta_b_cmd = seg["delta_cmd_end"]
        r_w_cmd = w_b - v_b * math.tan(delta_b_cmd) / L if moving else nan
        steer_cmd = (L * math.cos(delta_b_cmd) ** 2 / (v_b * T) * r_w_cmd
                     if moving else nan)

        row = seg.to_dict()
        row.update({
            "v_meas_a": v_a, "v_meas_b": v_b,
            "yawrate_meas_a": w_a, "yawrate_meas_b": w_b,
            "r_v": r_v, "accel_resid": r_v / T,
            "delta_inferred_a": delta_a, "delta_pred_b": delta_b_pred,
            "r_yawrate": r_w, "steer_rate_resid": steer,
            "r_yawrate_cmd": r_w_cmd, "steer_rate_resid_cmd": steer_cmd,
            "arc_m": arc,
            "gate_arc": bool(arc >= min_arc_m) if np.isfinite(arc) else False,
            "flag_delta_saturated": bool(abs(delta_b_pred) >= delta_max)
            if np.isfinite(delta_b_pred) else False,
            "lag_s": lag_s,
        })
        out.append(row)
    return pd.DataFrame(out)
