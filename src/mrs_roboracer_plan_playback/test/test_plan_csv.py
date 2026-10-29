import textwrap

import pytest

from mrs_roboracer_plan_playback.plan_csv import load_plan

_PLAN = textwrap.dedent("""\
    # kmcts-d60-t120-s1002  dt=0.1s  nominal goal entry step: 2  car model: noramp  goal: stop  actions: rate
    step,t,x,y,v,yaw,steer,speed_setpoint,steer_setpoint,speed_setpoint_rate_cmd,steer_setpoint_rate_cmd
    0,0.0,1.08,-0.97,0.0,3.0892,0.0,0.0,0.0,0.0,1.1
    1,0.1,1.08,-0.97,0.0,3.0892,0.003,0.0,0.11,0.26,-0.39
    2,0.2,1.08,-0.97,0.01,3.09,0.004,1.4,-0.14,NaN,NaN
    """)


def test_load_plan(tmp_path):
    csv_path = tmp_path / 'plan.csv'
    csv_path.write_text(_PLAN)

    plan = load_plan(csv_path)

    assert plan.dt == pytest.approx(0.1)
    # speed_setpoint/steer_setpoint stay valid through the last row -- only
    # the (unused) rate_cmd columns go NaN there.
    assert len(plan.rows) == 3
    assert plan.rows[0].step == 0
    assert plan.rows[0].x == pytest.approx(1.08)
    assert plan.rows[1].speed_setpoint == pytest.approx(0.0)
    assert plan.rows[2].speed_setpoint == pytest.approx(1.4)
    assert plan.rows[2].steer_setpoint == pytest.approx(-0.14)


def test_load_plan_requires_metadata_line(tmp_path):
    csv_path = tmp_path / 'plan.csv'
    csv_path.write_text('step,t,x,y,v,yaw,steer,speed_setpoint,steer_setpoint\n0,0,0,0,0,0,0,0,0\n')

    with pytest.raises(ValueError):
        load_plan(csv_path)


def test_load_plan_requires_rows(tmp_path):
    csv_path = tmp_path / 'plan.csv'
    csv_path.write_text(textwrap.dedent("""\
        # x dt=0.1s x
        step,t,x,y,v,yaw,steer,speed_setpoint,steer_setpoint,speed_setpoint_rate_cmd,steer_setpoint_rate_cmd
        """))

    with pytest.raises(ValueError):
        load_plan(csv_path)
