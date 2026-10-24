"""
Causal planar twist estimate from a mocap pose stream (rclpy-free).

Least-squares line fit of x, y and unwrapped yaw over the last `window`
samples. A line's slope is the velocity at the window's mean time, so the
estimate lags the newest sample by (window - 1) / 2 samples (~33 ms for
window 9 at 120 Hz); the world velocity is therefore rotated into the body
frame with the window's mean yaw, not the newest one, or turning would leak
into vy. Output follows nav_msgs/Odometry: vx forward, vy left, wz yaw rate.
(The Savitzky-Golay estimator in mrs_roboracer_sysid has the same centring
lag; this one avoids depending on sysid/pandas and resets on dropouts.)
"""

from collections import deque
import math
from typing import Optional, Tuple

import numpy as np


class TwistEstimator:

    def __init__(self, window: int = 9, max_gap: float = 0.1):
        if window < 2:
            raise ValueError('twist window must be >= 2 samples')
        self.window = window
        self.max_gap = max_gap
        self._buf = deque(maxlen=window)

    def reset(self) -> None:
        self._buf.clear()

    def update(self, t: float, x: float, y: float,
               yaw: float) -> Optional[Tuple[float, float, float]]:
        """Push a sample; returns (vx, vy, wz) in the body frame once the window is full."""
        if self._buf:
            dt = t - self._buf[-1][0]
            if dt <= 0.0:
                return None          # duplicate / out-of-order stamp: ignore the sample
            if dt > self.max_gap:
                self.reset()         # tracking dropout: don't differentiate across it
        if self._buf:
            # Unwrap yaw against the previous sample so the fit sees a continuous angle.
            prev = self._buf[-1][3]
            yaw = prev + math.remainder(yaw - prev, 2.0 * math.pi)
        self._buf.append((t, x, y, yaw))
        if len(self._buf) < self.window:
            return None
        a = np.asarray(self._buf)
        tc = a[:, 0] - a[:, 0].mean()
        denom = float(tc @ tc)
        if denom <= 0.0:
            return None
        xdot, ydot, wz = (tc @ (a[:, k] - a[:, k].mean()) / denom for k in (1, 2, 3))
        yaw_mid = float(a[:, 3].mean())
        c, s = math.cos(yaw_mid), math.sin(yaw_mid)
        return c * xdot + s * ydot, -s * xdot + c * ydot, float(wz)
