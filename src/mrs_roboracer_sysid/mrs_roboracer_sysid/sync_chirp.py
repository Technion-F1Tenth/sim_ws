"""
Sync-chirp calibration maneuver.

Per the sysid spec's hard sync requirement: prepend a distinctive
steering-step maneuver to every generated path so that, offline, the
command -> pose lag can be estimated by cross-correlation. A steering step
(rather than a smooth chirp) gives a sharp, easy-to-locate heading-rate
response.

Pure Python/numpy, no rclpy dependency.
"""

from dataclasses import replace
from typing import List

from mrs_roboracer_sysid.motion_model import ModelParams, curvature
from mrs_roboracer_sysid.spline_paths import CommandStep

import numpy as np


def generate_chirp_steps(chirp_amplitude: float, chirp_duration: float, control_rate_hz: float,
                         base_v: float, base_delta: float, params: ModelParams,
                         a_lat_max_safe: float, hold_duration: float = 0.2) -> List[CommandStep]:
    """
    Build the chirp command sequence: hold base_delta, step, step back.

    `base_delta` is held for `hold_duration`, then stepped to
    `base_delta + chirp_amplitude` (clipped to `params.delta_max`) for
    `chirp_duration`, then stepped back to `base_delta` for `chirp_duration`.
    Speed is held at `base_v` throughout. All steps get `flag_sync_chirp=True`.
    """
    dt = 1.0 / control_rate_hz
    n_hold = max(int(round(hold_duration * control_rate_hz)), 1)
    n_step = max(int(round(chirp_duration * control_rate_hz)), 1)

    delta_step = float(np.clip(base_delta + chirp_amplitude, -params.delta_max, params.delta_max))
    deltas = [base_delta] * n_hold + [delta_step] * n_step + [base_delta] * n_step

    steps = []
    for i, delta in enumerate(deltas):
        kappa = curvature(delta, params.L)
        lat_accel = base_v ** 2 * kappa
        steps.append(CommandStep(
            step_index=i,
            v=base_v,
            delta=delta,
            kappa=kappa,
            lat_accel=lat_accel,
            flag_high_lat_accel=abs(lat_accel) > a_lat_max_safe,
            dt=dt,
            flag_sync_chirp=True,
        ))
    return steps


def prepend_chirp(path_commands: List[CommandStep],
                  chirp_steps: List[CommandStep]) -> List[CommandStep]:
    """Concatenate chirp_steps + path_commands with `step_index` renumbered from 0."""
    combined = [replace(c, step_index=i) for i, c in enumerate(chirp_steps)]
    offset = len(chirp_steps)
    combined.extend(replace(c, step_index=c.step_index + offset) for c in path_commands)
    return combined


def prepend_sync_chirp(path_commands: List[CommandStep], chirp_amplitude: float,
                       chirp_duration: float, control_rate_hz: float, chirp_speed: float,
                       params: ModelParams, a_lat_max_safe: float,
                       hold_duration: float = 0.2) -> List[CommandStep]:
    """Generate a chirp handing off smoothly to `path_commands[0].delta`, then prepend it."""
    base_delta = path_commands[0].delta
    chirp_steps = generate_chirp_steps(chirp_amplitude, chirp_duration, control_rate_hz,
                                       chirp_speed, base_delta, params,
                                       a_lat_max_safe, hold_duration)
    return prepend_chirp(path_commands, chirp_steps)
