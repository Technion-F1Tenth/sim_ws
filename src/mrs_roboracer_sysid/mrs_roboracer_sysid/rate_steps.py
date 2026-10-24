"""
Held-rate-step command programs for input-noise identification.

Target model (rate-input kinematic bicycle, noise on the rate inputs):

    v_dot     = u1 + w1          (u1: acceleration, m/s^2)
    delta_dot = u2 + w2          (u2: steering rate, rad/s)
    theta_dot = v * tan(delta) / L,  x_dot = v cos(theta),  y_dot = v sin(theta)

A program is a sequence of *segments*, each holding (u1, u2) constant for
T seconds. The car only accepts speed/steering-angle setpoints, so each
segment is realized by integrating the held rates into ramped reference
(v_ref, delta_ref) trajectories at the control rate. Consecutive segments
are continuous in the reference. Offline, each segment yields one residual
per channel (see `segments.py`).

First-order lag compensation: if the actuators behave as
v_dot = (v_sp - v) / tau_v (likewise tau_delta for steering), publishing the
led setpoints

    v_sp = v_ref + tau_v * u1,    delta_sp = delta_ref + tau_delta * u2

makes the actual (v, delta) follow the reference ramps exactly (given a
matching initial condition), so the planner's v_dot = u1 model holds. The
setpoints jump by tau * (change in u) at segment boundaries; that is the
exact inverse, not an artefact. tau = 0 (default) disables it.

Segment selection is greedy: least-visited (u1, u2) cell for the current
speed band first, subject to feasibility under a setpoint-tracking rollout
(speed/steering/lateral-accel limits, and an excursion radius around the
arm pose that includes the straight-line stopping distance). Steering
segments (u2 != 0) are only scheduled when the predicted arc length
v_mean * T meets `min_arc_m`, mirroring the offline gate.

Pure Python/numpy, no rclpy dependency.
"""

from dataclasses import dataclass, field, replace
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from mrs_roboracer_sysid.motion_model import ModelParams, State, Control, curvature, step
from mrs_roboracer_sysid.spline_paths import CommandStep, GeneratedPath


@dataclass
class RateStepConfig:
    """Generation parameters; built from the node's `steps.*` + model/limits config."""

    L: float
    delta_max: float
    delta_dot_max: float
    v_min: float
    v_max: float
    a_lat_max_safe: float
    control_rate_hz: float
    u1_levels: List[float] = field(default_factory=lambda: [-3.0, -1.5, 0.0, 1.5, 3.0])
    u2_levels: List[float] = field(default_factory=lambda: [-3.0, -1.5, 0.0, 1.5, 3.0])
    min_segment_s: float = 0.2
    max_segment_s: float = 0.6
    segments_per_path: int = 15
    min_arc_m: float = 0.2
    v_min_steer: float = 1.0
    delta_margin: float = 0.02
    max_excursion_m: float = 3.0
    stop_decel: float = 2.0
    lead_in_accel: float = 1.5
    lead_in_speed: float = 1.0
    v_band_edges: List[float] = field(default_factory=lambda: [0.0, 1.0, 1.75, 3.0])
    tau_v: float = 0.0
    tau_delta: float = 0.0

    @classmethod
    def from_config(cls, config: dict) -> "RateStepConfig":
        model, limits, steps = config["model"], config["limits"], config.get("steps", {})
        cfg = cls(
            L=model["L"],
            delta_max=model["delta_max"],
            delta_dot_max=model["delta_dot_max"],
            v_min=limits["v_min"],
            v_max=limits["v_max"],
            a_lat_max_safe=limits["a_lat_max_safe"],
            control_rate_hz=config["rate"]["control_rate_hz"],
        )
        for key in ("u1_levels", "u2_levels", "v_band_edges"):
            if key in steps:
                setattr(cfg, key, [float(x) for x in steps[key]])
        for key in ("min_segment_s", "max_segment_s", "min_arc_m", "v_min_steer",
                    "delta_margin", "max_excursion_m", "stop_decel", "lead_in_accel",
                    "lead_in_speed", "tau_v", "tau_delta"):
            if key in steps:
                setattr(cfg, key, float(steps[key]))
        if "segments_per_path" in steps:
            cfg.segments_per_path = int(steps["segments_per_path"])
        cfg.validate()
        return cfg

    def validate(self) -> None:
        if max(abs(u) for u in self.u2_levels) > self.delta_dot_max:
            raise ValueError(
                f"steps.u2_levels exceed model.delta_dot_max={self.delta_dot_max}")
        if not 0.0 < self.min_segment_s <= self.max_segment_s:
            raise ValueError("need 0 < steps.min_segment_s <= steps.max_segment_s")
        if self.stop_decel <= 0.0 or self.lead_in_accel <= 0.0:
            raise ValueError("steps.stop_decel and steps.lead_in_accel must be > 0")
        if self.tau_v < 0.0 or self.tau_delta < 0.0:
            raise ValueError("steps.tau_v and steps.tau_delta must be >= 0")

    def setpoints(self, v_ref: float, delta_ref: float, u1: float,
                  u2: float) -> Tuple[float, float]:
        """Return the published (v_sp, delta_sp) leading the reference; v_sp never reverses."""
        return max(v_ref + self.tau_v * u1, 0.0), delta_ref + self.tau_delta * u2


class InputCoverage:
    """Segment counts per (u1 level, u2 level, start-speed band) cell."""

    def __init__(self, u1_levels, u2_levels, v_band_edges, min_arc_m: float):
        self.u1_levels = list(u1_levels)
        self.u2_levels = list(u2_levels)
        self.v_band_edges = np.asarray(v_band_edges, dtype=float)
        self.n_bands = len(self.v_band_edges) - 1
        self.min_arc_m = min_arc_m
        shape = (len(self.u1_levels), len(self.u2_levels), max(self.n_bands, 1))
        self.counts = np.zeros(shape, dtype=int)
        self.gated_counts = np.zeros(shape, dtype=int)

    def band_for(self, v: float) -> int:
        idx = int(np.digitize(v, self.v_band_edges)) - 1
        return idx if 0 <= idx < self.n_bands else -1

    def ranked_cells(self, v: float, rng: np.random.Generator) -> List[Tuple[int, int]]:
        """All (i1, i2) cells, least-visited first at this speed band (random tie-break)."""
        band = max(self.band_for(v), 0)
        cells = [(i1, i2) for i1 in range(len(self.u1_levels))
                 for i2 in range(len(self.u2_levels))]
        tie_break = rng.random(len(cells))
        order = sorted(range(len(cells)),
                       key=lambda k: (self.counts[cells[k][0], cells[k][1], band], tie_break[k]))
        return [cells[k] for k in order]

    def record(self, i1: int, i2: int, v_start: float, arc_m: float) -> None:
        band = self.band_for(v_start)
        if band < 0:
            return
        self.counts[i1, i2, band] += 1
        if arc_m >= self.min_arc_m:
            self.gated_counts[i1, i2, band] += 1

    def report(self) -> dict:
        return {
            "u1_levels": self.u1_levels,
            "u2_levels": self.u2_levels,
            "v_band_edges": self.v_band_edges.tolist(),
            "counts[u1][u2][band]": self.counts.tolist(),
            "arc_gated_counts[u1][u2][band]": self.gated_counts.tolist(),
            "segments_total": int(self.counts.sum()),
            "steering_segments_passing_arc_gate": int(
                sum(self.gated_counts[:, i2, :].sum()
                    for i2, u2 in enumerate(self.u2_levels) if u2 != 0.0)),
        }


@dataclass
class _SimState:
    x: float
    y: float
    theta: float
    v: float
    delta: float


def _advance(s: _SimState, u1: float, u2: float, dt: float, params: ModelParams) -> _SimState:
    """One control tick assuming perfect setpoint tracking (rollout for feasibility only)."""
    v_next = s.v + u1 * dt
    delta_next = s.delta + u2 * dt
    pose = step(State(s.x, s.y, s.theta),
                Control(0.5 * (s.v + v_next), 0.5 * (s.delta + delta_next)), dt, params)
    return _SimState(pose.x, pose.y, pose.theta, v_next, delta_next)


def _hold_setpoints(s: _SimState, v: float, delta: float, dt: float,
                    params: ModelParams) -> _SimState:
    pose = step(State(s.x, s.y, s.theta), Control(v, delta), dt, params)
    return _SimState(pose.x, pose.y, pose.theta, v, delta)


def _within_excursion(s: _SimState, cfg: RateStepConfig) -> bool:
    d_stop = max(s.v, 0.0) ** 2 / (2.0 * cfg.stop_decel)
    xs = s.x + d_stop * math.cos(s.theta)
    ys = s.y + d_stop * math.sin(s.theta)
    return math.hypot(xs, ys) <= cfg.max_excursion_m


def _feasible_steps(s0: _SimState, u1: float, u2: float, n_max: int,
                    cfg: RateStepConfig, params: ModelParams) -> List[_SimState]:
    """Roll out up to `n_max` ticks; return the states before the first violation."""
    dt = 1.0 / cfg.control_rate_hz
    delta_lim = cfg.delta_max - cfg.delta_margin
    states = []
    s = s0
    for _ in range(n_max):
        s = _advance(s, u1, u2, dt, params)
        lat = s.v ** 2 * curvature(s.delta, cfg.L)
        v_sp, delta_sp = cfg.setpoints(s.v, s.delta, u1, u2)
        if (s.v < cfg.v_min or s.v > cfg.v_max or abs(s.delta) > delta_lim
                or v_sp > cfg.v_max or abs(delta_sp) > delta_lim
                or abs(lat) > cfg.a_lat_max_safe or not _within_excursion(s, cfg)):
            break
        states.append(s)
    return states


def _emit(states: List[_SimState], u1: float, u2: float, segment_id: int,
          dt: float, cfg: RateStepConfig) -> List[CommandStep]:
    steps = []
    for s in states:
        v_sp, delta_sp = cfg.setpoints(s.v, s.delta, u1, u2)
        kappa = curvature(delta_sp, cfg.L)
        lat = v_sp ** 2 * kappa
        steps.append(CommandStep(
            step_index=0, v=float(v_sp), delta=float(delta_sp), kappa=float(kappa),
            lat_accel=float(lat), flag_high_lat_accel=abs(lat) > cfg.a_lat_max_safe,
            dt=dt, segment_id=segment_id, u1=float(u1), u2=float(u2),
            v_ref=float(s.v), delta_ref=float(s.delta)))
    return steps


def generate_rate_step_path(path_id: int, seed: int, rng: np.random.Generator,
                            cfg: RateStepConfig, coverage: InputCoverage,
                            sync_cfg: Optional[Dict] = None,
                            segment_id_start: int = 0) -> Optional[GeneratedPath]:
    """
    Build one held-rate-step program (optionally preceded by the sync chirp).

    Segment ids are unique across the session: they start at
    `segment_id_start`; the next free id is `max(segment_id) + 1` of the
    returned commands. Returns None if not even one measurement segment fits.
    """
    from mrs_roboracer_sysid.sync_chirp import generate_chirp_steps, prepend_chirp

    params = ModelParams(L=cfg.L, delta_max=cfg.delta_max, delta_dot_max=cfg.delta_dot_max)
    dt = 1.0 / cfg.control_rate_hz
    n_min = max(int(math.ceil(cfg.min_segment_s * cfg.control_rate_hz - 1e-9)), 1)
    n_max = max(int(math.floor(cfg.max_segment_s * cfg.control_rate_hz + 1e-9)), n_min)

    s = _SimState(0.0, 0.0, 0.0, 0.0, 0.0)
    trace = [s]

    chirp_steps: List[CommandStep] = []
    sync_cfg = sync_cfg or {}
    chirp_speed = float(sync_cfg.get("chirp_speed", cfg.v_min))
    if sync_cfg.get("enable_sync_chirp", True) and cfg.v_min <= chirp_speed <= cfg.v_max:
        chirp_steps = generate_chirp_steps(
            sync_cfg.get("chirp_amplitude", 0.2), sync_cfg.get("chirp_duration", 0.5),
            cfg.control_rate_hz, chirp_speed, 0.0, params, cfg.a_lat_max_safe)
        for c in chirp_steps:
            s = _hold_setpoints(s, c.v, c.delta, dt, params)
            trace.append(s)

    body: List[CommandStep] = []
    seg_id = segment_id_start

    # Lead-in: ramp up from standstill (no chirp) to a speed where segments are feasible.
    if s.v < cfg.v_min:
        target_v = max(cfg.lead_in_speed, cfg.v_min)
        n_lead = int(math.ceil((target_v - s.v) / (cfg.lead_in_accel * dt)))
        lead = []
        for _ in range(n_lead):
            s = _advance(s, cfg.lead_in_accel, 0.0, dt, params)
            lead.append(s)
        body += _emit(lead, cfg.lead_in_accel, 0.0, seg_id, dt, cfg)
        trace += lead
        seg_id += 1

    n_measure = 0
    for _ in range(cfg.segments_per_path):
        chosen = None
        for i1, i2 in coverage.ranked_cells(s.v, rng):
            u1, u2 = cfg.u1_levels[i1], cfg.u2_levels[i2]
            if u2 != 0.0 and s.v < cfg.v_min_steer:
                continue
            n_try = int(rng.integers(n_min, n_max + 1))
            states = _feasible_steps(s, u1, u2, n_try, cfg, params)
            if len(states) < n_min:
                continue
            T = len(states) * dt
            arc = 0.5 * (s.v + states[-1].v) * T
            if u2 != 0.0 and arc < cfg.min_arc_m:
                continue
            chosen = (i1, i2, u1, u2, states, arc)
            break
        if chosen is None:
            break
        i1, i2, u1, u2, states, arc = chosen
        coverage.record(i1, i2, s.v, arc)
        body += _emit(states, u1, u2, seg_id, dt, cfg)
        trace += states
        s = states[-1]
        seg_id += 1
        n_measure += 1

    if n_measure == 0:
        return None

    # Run-out: brake straight to a stop at constant decel, steering held.
    stop = []
    while s.v > 0.0:
        s = _advance(s, -cfg.stop_decel, 0.0, dt, params)
        s.v = max(s.v, 0.0)
        stop.append(s)
    body += _emit(stop, -cfg.stop_decel, 0.0, seg_id, dt, cfg)
    trace += stop

    body = [replace(c, step_index=i) for i, c in enumerate(body)]
    commands = prepend_chirp(body, chirp_steps)
    preview = np.array([(t.x, t.y, t.theta) for t in trace], dtype=float)

    return GeneratedPath(
        path_id=path_id,
        seed=seed,
        waypoints=preview[:, :2],
        spline_coeffs=None,
        commands=commands,
        coverage_target={"profile_type": "rate_steps", "kappa_center": 0.0,
                         "n_measurement_segments": n_measure},
        sync_chirp_steps=len(chirp_steps),
        preview_xytheta=preview,
    )
