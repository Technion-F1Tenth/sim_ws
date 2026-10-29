"""Parser for plan CSVs: a trajectory plus the speed/steer setpoints that
drive it. Any planner's output works as long as it matches the format
(e.g. OL_MCTS/KinoMCTS's tuning-runs/*/plans/kmcts-*.csv): a single
'#'-prefixed metadata comment line carrying `dt=<seconds>s`, then a standard
CSV header (step,t,x,y,v,yaw,steer,speed_setpoint,steer_setpoint,
speed_setpoint_rate_cmd,steer_setpoint_rate_cmd), then one row per planning
step. The terminal row's *_rate_cmd columns go NaN (there is no next step to
compute a transition into), but speed_setpoint/steer_setpoint -- the columns
actually used here -- stay valid through the last row, so no special
end-of-plan handling is needed while parsing; the rate_cmd columns aren't
read at all.
"""

import csv
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List

_DT_RE = re.compile(r'dt=([0-9.eE+-]+)s')


@dataclass
class PlanRow:
    step: int
    t: float
    x: float
    y: float
    v: float
    yaw: float
    steer: float
    speed_setpoint: float
    steer_setpoint: float


@dataclass
class Plan:
    dt: float
    rows: List[PlanRow]


def load_plan(path) -> Plan:
    """Load a plan CSV."""
    path = Path(path)
    with path.open('r', newline='') as f:
        first_line = f.readline()
        dt_match = _DT_RE.search(first_line)
        if not first_line.startswith('#') or not dt_match:
            raise ValueError(
                f"{path}: expected a '#...dt=<seconds>s...' metadata line first, got: "
                f'{first_line!r}')
        dt = float(dt_match.group(1))

        rows: List[PlanRow] = [
            PlanRow(
                step=int(raw['step']),
                t=float(raw['t']),
                x=float(raw['x']),
                y=float(raw['y']),
                v=float(raw['v']),
                yaw=float(raw['yaw']),
                steer=float(raw['steer']),
                speed_setpoint=float(raw['speed_setpoint']),
                steer_setpoint=float(raw['steer_setpoint']),
            )
            for raw in csv.DictReader(f)
        ]

    if not rows:
        raise ValueError(f'{path}: no data rows')
    return Plan(dt=dt, rows=rows)
