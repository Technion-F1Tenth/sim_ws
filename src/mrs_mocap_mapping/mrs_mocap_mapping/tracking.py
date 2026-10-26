"""
Frame-to-frame marker tracking for obstacles that follow their markers (rclpy-free).

A tracked obstacle keeps one estimate per vertex. Each mocap frame, all
tracked vertices are assigned jointly to live markers (Hungarian, cost =
distance, gated), so two vertices never claim the same marker and the
vertex order chosen at creation is preserved: moving or reshaping the
obstacle only requires each marker to stay near its previous position
between frames. Vertices with no marker inside the gate (occluded, removed)
are carried by a rigid 2D fit of the matched ones, which also predicts where
to re-acquire them; with no match at all the obstacle holds its last pose.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment

_INFEASIBLE = 1e9


def assign(prev, live, gate: float) -> np.ndarray:
    """Match (V, 3) vertex estimates to (M, 3) live markers; returns (V,) marker index or -1."""
    prev = np.asarray(prev, dtype=float).reshape(-1, 3)
    live = np.asarray(live, dtype=float).reshape(-1, 3)
    out = np.full(len(prev), -1, dtype=int)
    if len(prev) == 0 or len(live) == 0:
        return out
    d = np.linalg.norm(prev[:, None, :] - live[None, :, :], axis=-1)
    rows, cols = linear_sum_assignment(np.where(d <= gate, d, _INFEASIBLE))
    ok = d[rows, cols] <= gate
    out[rows[ok]] = cols[ok]
    return out


def rigid_fill(prev, new, matched) -> np.ndarray:
    """
    Fill unmatched rows of `new` by moving `prev` with the rigid motion of the matched rows.

    The motion is a yaw rotation + xy translation (2D Kabsch) plus the mean z
    shift; one matched vertex gives a pure translation, none leaves `prev`.
    """
    prev = np.asarray(prev, dtype=float)
    out = np.array(new, dtype=float)
    matched = np.asarray(matched, dtype=bool)
    if matched.all():
        return out
    if not matched.any():
        return prev.copy()
    p, q = prev[matched], out[matched]
    pc, qc = p[:, :2].mean(axis=0), q[:, :2].mean(axis=0)
    h = (p[:, :2] - pc).T @ (q[:, :2] - qc)
    th = np.arctan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1]) if len(p) >= 2 else 0.0
    c, s = np.cos(th), np.sin(th)
    rot = np.array([[c, -s], [s, c]])
    miss = ~matched
    out[miss, :2] = (prev[miss, :2] - pc) @ rot.T + qc
    out[miss, 2] = prev[miss, 2] + float(np.mean(q[:, 2] - p[:, 2]))
    return out
