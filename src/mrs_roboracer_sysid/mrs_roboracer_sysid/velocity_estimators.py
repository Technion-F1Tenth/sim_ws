"""
Velocity ground-truth estimators: Savitzky-Golay pose derivative, mocap twist, onboard odom.

Per §6 of the sysid spec, `velocity_source` is configurable and the final
choice is pinned during mocap integration; this module exists so that
whichever sources are available can be compared.

Rclpy-free / pure numpy so it is usable both:

- *online*, by sysid_data_collector -- per the "node logs raw, doesn't
  compute" design commitment, the node does NOT compute v_meas_l for
  logging (pose/twist/odom go straight into the bag unmodified). The
  only online use of this module is PoseDerivativeEstimator for the
  SETTLE-phase |v| ~= 0 check.
- *offline*, by bag_to_dataset, to compute v_meas_l (and source-tagged
  comparisons) from whichever streams are present in the bag.
"""

import math
import warnings
from collections import deque
from typing import Optional, Tuple

import numpy as np
from scipy.signal import savgol_filter

SG_NOISE_WARNING = (
    "pose_derivative velocity estimation differentiates noisy position "
    "measurements, which amplifies position noise by ~1/dt. The "
    "Savitzky-Golay filter mitigates but does not remove this -- treat "
    "pose_derivative velocity/heading-rate estimates accordingly."
)


def signed_forward_speed(vx: float, vy: float, theta: float) -> float:
    """Project a velocity vector onto the heading direction -> signed v."""
    return vx * math.cos(theta) + vy * math.sin(theta)


class PoseDerivativeEstimator:
    """
    Savitzky-Golay derivative of (x, y, theta) -> (v, thetadot).

    Online: call `update()` once per pose sample; returns an estimate for
    the buffer-center sample once `window` samples have been collected
    (i.e. with a `window // 2`-sample delay).

    Offline: call `compute_batch()` once with the full trajectory arrays
    (no causality constraint, so every sample gets an estimate).

    Assumes the pose stream is close to uniformly sampled. If the
    actually-observed jitter ratio (std(dt) / mean(dt)) exceeds
    `jitter_warn_ratio`, a one-time warning is logged.
    """

    def __init__(self, window: int, polyorder: int,
                 jitter_warn_ratio: float = 0.1, logger=None):
        if window % 2 == 0 or window <= polyorder:
            raise ValueError("window must be odd and > polyorder")
        self.window = window
        self.polyorder = polyorder
        self.jitter_warn_ratio = jitter_warn_ratio
        self._logger = logger
        self._warned_noise = False
        self._warned_jitter = False
        self._buf_t: deque = deque(maxlen=window)
        self._buf_x: deque = deque(maxlen=window)
        self._buf_y: deque = deque(maxlen=window)
        self._buf_theta: deque = deque(maxlen=window)

    def _warn(self, message: str, already_warned_attr: str) -> None:
        if getattr(self, already_warned_attr):
            return
        if self._logger is not None:
            self._logger.warn(message)
        else:
            warnings.warn(message)
        setattr(self, already_warned_attr, True)

    def update(self, t: float, x: float,
               y: float, theta: float) -> Optional[Tuple[float, float]]:
        """
        Push a new (t, x, y, theta) sample.

        Returns (v, thetadot) for the buffer-center sample once the
        rolling buffer is full, else None.
        """
        self._warn(SG_NOISE_WARNING, '_warned_noise')
        self._buf_t.append(t)
        self._buf_x.append(x)
        self._buf_y.append(y)
        self._buf_theta.append(theta)
        if len(self._buf_t) < self.window:
            return None

        t_arr = np.asarray(self._buf_t, dtype=float)
        dts = np.diff(t_arr)
        dt_mean = float(np.mean(dts))
        if dt_mean <= 0:
            return None
        if dt_mean > 0:
            jitter_ratio = float(np.std(dts) / dt_mean)
            if jitter_ratio > self.jitter_warn_ratio:
                self._warn(
                    f"pose timestamp jitter ratio {jitter_ratio:.3f} exceeds "
                    f"sg_dt_jitter_warn_ratio={self.jitter_warn_ratio}; "
                    "Savitzky-Golay derivative assumes near-uniform sampling.",
                    '_warned_jitter',
                )

        x_arr = np.asarray(self._buf_x, dtype=float)
        y_arr = np.asarray(self._buf_y, dtype=float)
        theta_arr = np.unwrap(np.asarray(self._buf_theta, dtype=float))

        xdot = savgol_filter(x_arr, self.window, self.polyorder, deriv=1, delta=dt_mean)
        ydot = savgol_filter(y_arr, self.window, self.polyorder, deriv=1, delta=dt_mean)
        thetadot = savgol_filter(theta_arr, self.window, self.polyorder, deriv=1, delta=dt_mean)

        center = self.window // 2
        v = signed_forward_speed(xdot[center], ydot[center], theta_arr[center])
        return v, float(thetadot[center])

    @classmethod
    def compute_batch(cls, t: np.ndarray, x: np.ndarray, y: np.ndarray, theta: np.ndarray,
                      window: int, polyorder: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Offline batch derivative -> (v, thetadot) arrays, same length as input.

        A local least-squares polynomial of order `polyorder` is fit over
        `window` samples using their actual timestamps. On a uniform grid
        this equals the Savitzky-Golay derivative. Real mocap streams that
        cross WiFi drop frames and repeat stamps, so the trajectory is first
        split into runs at non-increasing stamps and at gaps over
        `gap_factor` times the median sample interval. Each run is filtered
        on its own, and windows never straddle a gap. Samples within
        `window // 2` of a run end, repeated stamps, and runs shorter than
        `window` are NaN.
        """
        n = len(t)
        v = np.full(n, np.nan)
        thetadot = np.full(n, np.nan)
        if n < window:
            return v, thetadot

        t_arr = np.asarray(t, dtype=float)
        x_arr = np.asarray(x, dtype=float)
        y_arr = np.asarray(y, dtype=float)
        th_arr = np.asarray(theta, dtype=float)

        # Keep the first sample of each repeated/out-of-order stamp.
        keep = np.ones(n, dtype=bool)
        last = t_arr[0]
        for i in range(1, n):
            if t_arr[i] <= last:
                keep[i] = False
            else:
                last = t_arr[i]
        idx = np.flatnonzero(keep)
        dt = np.diff(t_arr[idx])
        if len(dt) == 0:
            return v, thetadot
        max_gap = cls.GAP_FACTOR * float(np.median(dt))
        breaks = np.flatnonzero(dt > max_gap) + 1
        half = window // 2
        for run in np.split(idx, breaks):
            if len(run) < window:
                continue
            vr, wr = cls._local_poly_derivative(
                t_arr[run], x_arr[run], y_arr[run], np.unwrap(th_arr[run]), window, polyorder)
            v[run[half:len(run) - half]] = vr
            thetadot[run[half:len(run) - half]] = wr
        return v, thetadot

    GAP_FACTOR = 3.0

    @staticmethod
    def _local_poly_derivative(t, x, y, theta, window, polyorder):
        """Return (v, thetadot) at interior samples from a local polynomial fit on real stamps."""
        from numpy.lib.stride_tricks import sliding_window_view

        half = window // 2
        tw = sliding_window_view(t, window)                    # (m, window)
        tau = tw - t[half:len(t) - half, None]
        scale = float(np.median(np.diff(t)))                   # conditioning
        tau = tau / scale
        A = tau[..., None] ** np.arange(polyorder + 1)         # (m, window, p+1)
        AtA = np.einsum('mwi,mwj->mij', A, A)
        pinv_row1 = np.linalg.solve(AtA, np.transpose(A, (0, 2, 1)))[:, 1, :]  # (m, window)

        def deriv(sig):
            return np.einsum('mw,mw->m', pinv_row1, sliding_window_view(sig, window)) / scale

        xdot, ydot, thetadot = deriv(x), deriv(y), deriv(theta)
        th_c = theta[half:len(t) - half]
        return xdot * np.cos(th_c) + ydot * np.sin(th_c), thetadot


class MocapTwistEstimator:
    """Velocity from a mocap-republished twist (vx, vy) via heading projection."""

    @staticmethod
    def velocity(vx: float, vy: float, theta: float) -> float:
        return signed_forward_speed(vx, vy, theta)


class OnboardOdomEstimator:
    """
    Velocity from the car's own (VESC-based) odometry estimate.

    Lowest trust per §6 -- fallback only. nav_msgs/Odometry already reports
    forward body-frame speed directly (twist.twist.linear.x).
    """

    @staticmethod
    def velocity(vx_body: float) -> float:
        return vx_body
