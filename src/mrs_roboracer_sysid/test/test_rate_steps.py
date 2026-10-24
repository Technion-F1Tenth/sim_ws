import math

import numpy as np
import pytest

from mrs_roboracer_sysid.rate_steps import (
    InputCoverage,
    RateStepConfig,
    generate_rate_step_path,
)
from mrs_roboracer_sysid.spline_paths import sample_generated_path, save_generated_paths

_SYNC = {"enable_sync_chirp": True, "chirp_amplitude": 0.2,
         "chirp_duration": 0.5, "chirp_speed": 1.5}


def _cfg(**overrides):
    cfg = RateStepConfig(L=0.33, delta_max=0.4189, delta_dot_max=3.2, v_min=0.5, v_max=3.0,
                         a_lat_max_safe=4.0, control_rate_hz=50.0)
    for k, v in overrides.items():
        setattr(cfg, k, v)
    cfg.validate()
    return cfg


def _coverage(cfg):
    return InputCoverage(cfg.u1_levels, cfg.u2_levels, cfg.v_band_edges, cfg.min_arc_m)


def _segments(path):
    segs = {}
    for c in path.commands:
        if c.segment_id >= 0:
            segs.setdefault(c.segment_id, []).append(c)
    return segs


@pytest.mark.parametrize("sync", [_SYNC, {"enable_sync_chirp": False}])
def test_program_respects_limits_and_is_continuous(sync):
    cfg = _cfg()
    path = generate_rate_step_path(0, 0, np.random.default_rng(1), cfg, _coverage(cfg), sync)
    assert path is not None
    dt = 1.0 / cfg.control_rate_hz

    assert [c.step_index for c in path.commands] == list(range(len(path.commands)))
    body = path.commands[path.sync_chirp_steps:]
    v = np.array([c.v for c in body])
    delta = np.array([c.delta for c in body])
    assert np.all(np.abs(delta) <= cfg.delta_max - cfg.delta_margin + 1e-9)
    assert np.all(v <= cfg.v_max + 1e-9) and np.all(v >= -1e-9)
    # held rates: every step's reference change equals its own (u1, u2) * dt
    for prev, cur in zip(body[:-1], body[1:]):
        if cur.v_ref > 0.0:
            assert cur.v_ref - prev.v_ref == pytest.approx(cur.u1 * dt, abs=1e-9)
        assert cur.delta_ref - prev.delta_ref == pytest.approx(cur.u2 * dt, abs=1e-9)
    # no compensation configured: published setpoints are the reference
    assert all(c.v == c.v_ref and c.delta == c.delta_ref for c in body)
    assert body[-1].v == 0.0


def test_segments_hold_constant_inputs_and_meet_gates():
    cfg = _cfg()
    path = generate_rate_step_path(0, 0, np.random.default_rng(2), cfg, _coverage(cfg), _SYNC)
    segs = _segments(path)
    n_min = round(cfg.min_segment_s * cfg.control_rate_hz)
    measurement = list(segs.values())[:-1]  # last one is the run-out brake
    assert len(measurement) == path.coverage_target["n_measurement_segments"]
    for steps in measurement:
        assert len({(c.u1, c.u2) for c in steps}) == 1
        assert len(steps) >= n_min
        if steps[0].u2 != 0.0:
            T = len(steps) / cfg.control_rate_hz
            v0 = steps[0].v - steps[0].u1 / cfg.control_rate_hz
            assert 0.5 * (v0 + steps[-1].v) * T >= cfg.min_arc_m - 1e-9
            assert v0 >= cfg.v_min_steer - 1e-9


def test_preview_stays_within_excursion_radius():
    cfg = _cfg(max_excursion_m=2.5)
    rng = np.random.default_rng(3)
    cov = _coverage(cfg)
    for pid in range(10):
        path = generate_rate_step_path(pid, 0, rng, cfg, cov, _SYNC)
        if path is None:
            continue
        r = np.hypot(path.preview_xytheta[:, 0], path.preview_xytheta[:, 1])
        assert r.max() <= cfg.max_excursion_m + 1e-6


def test_segment_ids_unique_across_programs_and_coverage_spreads():
    cfg = _cfg()
    rng = np.random.default_rng(4)
    cov = _coverage(cfg)
    next_id, seen = 0, set()
    for pid in range(20):
        path = generate_rate_step_path(pid, 0, rng, cfg, cov, _SYNC, segment_id_start=next_id)
        ids = set(_segments(path))
        assert not ids & seen
        seen |= ids
        next_id = max(ids) + 1
    counts = np.array(cov.report()["counts[u1][u2][band]"]).sum(axis=2)
    # every steering-rate level, including the full +/-3 rad/s, gets exercised
    assert np.all(counts.sum(axis=0) > 0)


def test_sample_and_save_without_spline(tmp_path):
    cfg = _cfg()
    path = generate_rate_step_path(5, 0, np.random.default_rng(5), cfg, _coverage(cfg), _SYNC)
    samples = sample_generated_path(path, n_samples=50)
    assert samples.shape == (50, 3)
    assert samples[0] == pytest.approx([0.0, 0.0, 0.0])
    save_generated_paths([path], str(tmp_path))
    loaded = np.load(tmp_path / "path_005.npz")
    assert loaded["commands"].shape == (len(path.commands), 13)
    assert "preview_xytheta" in loaded


def test_rejects_u2_levels_beyond_delta_dot_max():
    with pytest.raises(ValueError):
        _cfg(u2_levels=[-4.0, 0.0, 4.0])


def test_from_config_reads_steps_section():
    config = {
        "model": {"L": 0.33, "delta_max": 0.4189, "delta_dot_max": 3.2},
        "limits": {"v_min": 0.5, "v_max": 3.0, "a_lat_max_safe": 4.0},
        "rate": {"control_rate_hz": 50.0},
        "steps": {"u1_levels": [-1.0, 0.0, 1.0], "segments_per_path": 4,
                  "max_excursion_m": 4.0},
    }
    cfg = RateStepConfig.from_config(config)
    assert cfg.u1_levels == [-1.0, 0.0, 1.0]
    assert cfg.segments_per_path == 4
    assert math.isclose(cfg.max_excursion_m, 4.0)


def test_lag_compensation_leads_reference_within_limits():
    cfg = _cfg(tau_v=0.08, tau_delta=0.05)
    path = generate_rate_step_path(0, 0, np.random.default_rng(6), cfg, _coverage(cfg), _SYNC)
    body = path.commands[path.sync_chirp_steps:]
    for c in body:
        assert c.v == pytest.approx(max(c.v_ref + 0.08 * c.u1, 0.0))
        assert c.delta == pytest.approx(c.delta_ref + 0.05 * c.u2)
        assert abs(c.delta) <= cfg.delta_max - cfg.delta_margin + 1e-9
        assert 0.0 <= c.v <= cfg.v_max + 1e-9


def _drive_lagged_plant(commands, tau_v, tau_delta, sub=10):
    """First-order-lag actuators driven by the published setpoints -> step table + streams."""
    import pandas as pd
    L, dt = 0.33, commands[0].dt
    h = dt / sub
    v = d = t = 0.0
    rows, tt, vv, ww = [], [], [], []
    for i, c in enumerate(commands):
        rows.append({"path_id": 0, "segment_id": c.segment_id, "t_cmd": i * dt, "dt_cmd": dt,
                     "u1_cmd": c.u1, "u2_cmd": c.u2, "v_cmd": c.v, "delta_cmd": c.delta,
                     "v_ref": c.v if c.v_ref is None else c.v_ref,
                     "delta_ref": c.delta if c.delta_ref is None else c.delta_ref,
                     "flag_stale_pose": False, "flag_safety_override": False})
        for _ in range(sub):
            tt.append(t)
            vv.append(v)
            ww.append(v * math.tan(d) / L)
            v += (c.v - v) / tau_v * h
            d += (c.delta - d) / tau_delta * h
            t += h
    return pd.DataFrame(rows), np.array(tt), np.array(vv), np.array(ww)


def test_lag_compensation_removes_residual_bias():
    from mrs_roboracer_sysid.segments import compute_residuals, segment_table

    tau_v, tau_delta = 0.08, 0.05
    results = {}
    for name, comp in (("raw", (0.0, 0.0)), ("compensated", (tau_v, tau_delta))):
        cfg = _cfg(tau_v=comp[0], tau_delta=comp[1])
        cov = _coverage(cfg)
        rng = np.random.default_rng(7)
        resid = []
        for pid in range(10):
            path = generate_rate_step_path(pid, 0, rng, cfg, cov, _SYNC)
            steps, t, v, w = _drive_lagged_plant(path.commands, tau_v, tau_delta)
            seg = compute_residuals(segment_table(steps), t, v, w, L=0.33)
            seg = seg[(seg["v_meas_a"] > 0.3) & (seg["v_meas_b"] > 0.3)]
            resid.append(seg[["u1_cmd", "accel_resid", "steer_rate_resid"]])
        import pandas as pd
        results[name] = pd.concat(resid)

    raw, comp = results["raw"], results["compensated"]
    # uncompensated lag: accel residual is strongly anti-correlated with u1
    assert np.corrcoef(raw["u1_cmd"], raw["accel_resid"])[0, 1] < -0.8
    rms = {k: float(np.sqrt(np.mean(r["accel_resid"] ** 2))) for k, r in results.items()}
    assert rms["compensated"] < 0.25 * rms["raw"]
    steer_rms = {k: float(np.sqrt(np.nanmean(r["steer_rate_resid"] ** 2)))
                 for k, r in results.items()}
    assert steer_rms["compensated"] < 0.5 * steer_rms["raw"]
