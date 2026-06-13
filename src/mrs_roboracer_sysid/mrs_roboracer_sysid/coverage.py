"""
(v, kappa) coverage tracking and path-template scheduling.

Per §4/§10 of the sysid spec: the data collector should spend its session
sweeping the (v, kappa) operating envelope evenly, with extra attention to
the high-lateral-accel "corner" (high v, high |kappa|) where the kinematic
model is expected to break, plus periodic figure-8 paths that sweep many
bins at once.

Pure Python/numpy, no rclpy dependency.
"""

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

# |kappa_center| below this is treated as "straight enough" for a
# speed_sweep template rather than a curvature_sweep.
_STRAIGHT_KAPPA_THRESHOLD = 0.5


@dataclass
class CoverageBin:
    """A (v, kappa) generation target, with the profile template to use."""

    v_idx: int
    kappa_idx: int
    v_center: float
    kappa_center: float
    profile_type: str


class CoverageTracker:
    """
    Tracks per-(v,kappa)-bin visit counts and picks the next generation target.

    `v_bins`/`kappa_bins` are bin *edges* (as in `np.digitize`): `len(edges)-1`
    interior bins. Samples outside `[edges[0], edges[-1])` map to bin index -1.
    """

    def __init__(self, v_bins: List[float], kappa_bins: List[float], target_revisits: int,
                 include_figure8: bool = True, figure8_period: int = 5,
                 figure8_speed: float = None, prioritize_corner_first: bool = True,
                 corner_probe_paths: int = 2):
        self.v_bins = np.asarray(v_bins, dtype=float)
        self.kappa_bins = np.asarray(kappa_bins, dtype=float)
        self.n_v = len(self.v_bins) - 1
        self.n_kappa = len(self.kappa_bins) - 1
        self.target_revisits = target_revisits
        self.include_figure8 = include_figure8
        self.figure8_period = figure8_period
        self.figure8_speed = (
            figure8_speed if figure8_speed is not None
            else 0.5 * (self.v_bins[0] + self.v_bins[-1])
        )
        self.prioritize_corner_first = prioritize_corner_first
        self.corner_probe_paths = corner_probe_paths

        self.occupancy = np.zeros((self.n_v, self.n_kappa), dtype=int)
        self._path_count = 0

    def bin_for(self, v: float, kappa: float) -> Tuple[int, int]:
        """Return (v_idx, kappa_idx), or -1 for either if out of grid range."""
        v_idx = int(np.digitize(v, self.v_bins)) - 1
        kappa_idx = int(np.digitize(kappa, self.kappa_bins)) - 1
        if v_idx < 0 or v_idx >= self.n_v:
            v_idx = -1
        if kappa_idx < 0 or kappa_idx >= self.n_kappa:
            kappa_idx = -1
        return v_idx, kappa_idx

    def bin_id(self, v_idx: int, kappa_idx: int) -> int:
        """Flattened bin id `v_idx*n_kappa + kappa_idx`, or -1 if either index is -1."""
        if v_idx < 0 or kappa_idx < 0:
            return -1
        return v_idx * self.n_kappa + kappa_idx

    def record_step(self, v: float, kappa: float) -> Tuple[int, int]:
        """Increment occupancy for the bin containing (v, kappa); return its indices."""
        v_idx, kappa_idx = self.bin_for(v, kappa)
        if v_idx >= 0 and kappa_idx >= 0:
            self.occupancy[v_idx, kappa_idx] += 1
        return v_idx, kappa_idx

    def _bin_center(self, v_idx: int, kappa_idx: int) -> Tuple[float, float]:
        v_center = 0.5 * (self.v_bins[v_idx] + self.v_bins[v_idx + 1])
        kappa_center = 0.5 * (self.kappa_bins[kappa_idx] + self.kappa_bins[kappa_idx + 1])
        return float(v_center), float(kappa_center)

    def _profile_for_bin(self, v_idx: int, kappa_idx: int) -> CoverageBin:
        v_center, kappa_center = self._bin_center(v_idx, kappa_idx)
        is_corner = (v_idx == self.n_v - 1) and (kappa_idx in (0, self.n_kappa - 1))
        if is_corner:
            profile_type = "corner_probe"
        elif abs(kappa_center) < _STRAIGHT_KAPPA_THRESHOLD:
            profile_type = "speed_sweep"
        else:
            profile_type = "curvature_sweep"
        return CoverageBin(v_idx=v_idx, kappa_idx=kappa_idx, v_center=v_center,
                           kappa_center=kappa_center, profile_type=profile_type)

    def next_target(self) -> CoverageBin:
        """
        Pick the next (v, kappa) generation target.

        Path numbering is 1-based internally: every `figure8_period`-th path
        is a figure8 (regardless of bin occupancy); if `prioritize_corner_first`,
        the first `corner_probe_paths` paths target the highest-(v,|kappa|)
        corner; otherwise greedily pick the bin with the lowest
        occupancy/target_revisits ratio.
        """
        self._path_count += 1
        path_num = self._path_count

        if (self.include_figure8 and self.figure8_period > 0
                and path_num % self.figure8_period == 0):
            return CoverageBin(v_idx=-1, kappa_idx=-1, v_center=self.figure8_speed,
                               kappa_center=0.0, profile_type="figure8")

        if self.prioritize_corner_first and path_num <= self.corner_probe_paths:
            v_idx, kappa_idx = self.n_v - 1, self.n_kappa - 1
            self.occupancy[v_idx, kappa_idx] += 1
            return self._profile_for_bin(v_idx, kappa_idx)

        ratio = self.occupancy.astype(float) / max(self.target_revisits, 1)
        v_idx, kappa_idx = np.unravel_index(np.argmin(ratio), ratio.shape)
        v_idx, kappa_idx = int(v_idx), int(kappa_idx)
        # Reserve this bin immediately so back-to-back next_target() calls
        # (e.g. generating all paths for a session up front, before any
        # steps have executed) diversify rather than all picking bin (0,0).
        self.occupancy[v_idx, kappa_idx] += 1
        return self._profile_for_bin(v_idx, kappa_idx)

    def report(self) -> dict:
        """Occupancy grid + per-bin visit/target/met status + overall summary."""
        bins = []
        bins_met = 0
        for v_idx in range(self.n_v):
            for kappa_idx in range(self.n_kappa):
                visits = int(self.occupancy[v_idx, kappa_idx])
                met = visits >= self.target_revisits
                if met:
                    bins_met += 1
                v_center, kappa_center = self._bin_center(v_idx, kappa_idx)
                bins.append({
                    "v_idx": v_idx,
                    "kappa_idx": kappa_idx,
                    "v_center": v_center,
                    "kappa_center": kappa_center,
                    "visits": visits,
                    "target": self.target_revisits,
                    "met": met,
                })

        bins_total = self.n_v * self.n_kappa
        return {
            "occupancy": self.occupancy.tolist(),
            "bins": bins,
            "bins_total": bins_total,
            "bins_met": bins_met,
            "pct_met": 100.0 * bins_met / bins_total if bins_total else 0.0,
        }
