import pytest

from mrs_roboracer_sysid.motion_model import ModelParams, curvature
from mrs_roboracer_sysid.spline_paths import CommandStep
from mrs_roboracer_sysid.sync_chirp import generate_chirp_steps, prepend_chirp, prepend_sync_chirp


def _params():
    return ModelParams(L=0.33, delta_max=0.4189, delta_dot_max=3.2)


def test_generate_chirp_steps_structure():
    params = _params()
    steps = generate_chirp_steps(
        chirp_amplitude=0.1, chirp_duration=0.5, control_rate_hz=50.0,
        base_v=1.5, base_delta=0.0, params=params, a_lat_max_safe=4.0, hold_duration=0.2,
    )

    n_hold = round(0.2 * 50.0)
    n_step = round(0.5 * 50.0)
    assert len(steps) == n_hold + 2 * n_step
    assert all(s.flag_sync_chirp for s in steps)
    assert all(s.v == pytest.approx(1.5) for s in steps)

    # hold phase at base_delta
    for s in steps[:n_hold]:
        assert s.delta == pytest.approx(0.0)
    # stepped phase at base_delta + amplitude
    for s in steps[n_hold:n_hold + n_step]:
        assert s.delta == pytest.approx(0.1)
    # step-back phase returns to base_delta
    for s in steps[n_hold + n_step:]:
        assert s.delta == pytest.approx(0.0)

    # kappa/lat_accel consistent with delta
    for s in steps:
        assert s.kappa == pytest.approx(curvature(s.delta, params.L))
        assert s.lat_accel == pytest.approx(s.v ** 2 * s.kappa)


def test_generate_chirp_steps_clips_to_delta_max():
    params = _params()
    steps = generate_chirp_steps(
        chirp_amplitude=1.0, chirp_duration=0.1, control_rate_hz=50.0,
        base_v=1.5, base_delta=0.3, params=params, a_lat_max_safe=4.0, hold_duration=0.1,
    )
    n_hold = round(0.1 * 50.0)
    stepped = steps[n_hold]
    assert stepped.delta == pytest.approx(params.delta_max)


def test_prepend_chirp_renumbers_steps():
    chirp_steps = [
        CommandStep(step_index=0, v=1.0, delta=0.0, kappa=0.0, lat_accel=0.0,
                    flag_high_lat_accel=False, dt=0.02, flag_sync_chirp=True),
        CommandStep(step_index=1, v=1.0, delta=0.1, kappa=0.3, lat_accel=0.1,
                    flag_high_lat_accel=False, dt=0.02, flag_sync_chirp=True),
    ]
    path_commands = [
        CommandStep(step_index=0, v=2.0, delta=0.05, kappa=0.15, lat_accel=0.6,
                    flag_high_lat_accel=False, dt=0.02, flag_sync_chirp=False),
        CommandStep(step_index=1, v=2.0, delta=0.05, kappa=0.15, lat_accel=0.6,
                    flag_high_lat_accel=False, dt=0.02, flag_sync_chirp=False),
    ]

    combined = prepend_chirp(path_commands, chirp_steps)

    assert [c.step_index for c in combined] == [0, 1, 2, 3]
    assert [c.flag_sync_chirp for c in combined] == [True, True, False, False]
    assert combined[2].v == pytest.approx(2.0)


def test_prepend_sync_chirp_uses_first_command_delta_as_base():
    params = _params()
    path_commands = [
        CommandStep(step_index=0, v=2.0, delta=0.05, kappa=curvature(0.05, params.L),
                    lat_accel=0.0, flag_high_lat_accel=False, dt=0.02, flag_sync_chirp=False),
        CommandStep(step_index=1, v=2.0, delta=0.05, kappa=curvature(0.05, params.L),
                    lat_accel=0.0, flag_high_lat_accel=False, dt=0.02, flag_sync_chirp=False),
    ]

    combined = prepend_sync_chirp(
        path_commands, chirp_amplitude=0.1, chirp_duration=0.2, control_rate_hz=50.0,
        chirp_speed=1.5, params=params, a_lat_max_safe=4.0, hold_duration=0.1,
    )

    n_hold = round(0.1 * 50.0)
    n_step = round(0.2 * 50.0)
    n_chirp = n_hold + 2 * n_step

    # step-back phase hands off at the path's first command delta
    handoff_step = combined[n_chirp - 1]
    assert handoff_step.delta == pytest.approx(0.05)

    # path commands follow, renumbered
    assert combined[n_chirp].step_index == n_chirp
    assert combined[n_chirp].flag_sync_chirp is False
    assert combined[n_chirp].delta == pytest.approx(0.05)
