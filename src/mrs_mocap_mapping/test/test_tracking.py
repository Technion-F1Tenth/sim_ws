import math

import numpy as np
import pytest

from mrs_mocap_mapping.grid import GridSpec
from mrs_mocap_mapping.map_state import MapState
from mrs_mocap_mapping.obstacles import load_map_file, Obstacle
from mrs_mocap_mapping.tracking import assign, rigid_fill
from mrs_mocap_mapping.web_app import create_app

BOX = np.array([(0.0, 0.0, 0.2), (0.4, 0.0, 0.2), (0.4, 0.3, 0.2), (0.0, 0.3, 0.2)])
CAR = (3.0, 3.0, 0.1)


def _move(pts, dx=0.0, dy=0.0, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.array(pts, dtype=float)
    out[:, :2] = out[:, :2] @ np.array([[c, -s], [s, c]]).T + (dx, dy)
    return out


def test_assign_is_one_to_one_and_gated():
    prev = [(0, 0, 0), (0.05, 0, 0), (1, 1, 0)]
    live = [(0.04, 0, 0), (0.0, 0.01, 0), (5, 5, 0)]
    idx = assign(prev, live, gate=0.1)
    assert list(idx) == [1, 0, -1]
    assert list(assign(prev, [], 0.1)) == [-1, -1, -1]


def test_rigid_fill_recovers_rotation_and_translation():
    new = _move(BOX, 0.3, -0.2, 0.4)
    new[:, 2] += 0.05
    matched = np.array([True, True, False, True])
    out = rigid_fill(BOX, np.where(matched[:, None], new, 0.0), matched)
    assert np.allclose(out, new, atol=1e-9)
    one = rigid_fill(BOX, np.where([[True], [False], [False], [False]], new, 0.0),
                     [True, False, False, False])
    assert np.allclose(one[1] - BOX[1], new[0] - BOX[0])   # one marker: pure translation
    assert np.allclose(rigid_fill(BOX, BOX * 0, [False] * 4), BOX)


@pytest.fixture
def state(tmp_path):
    s = MapState(GridSpec(resolution=0.05, origin_x=-2, origin_y=-2, width=120, height=120),
                 up_axis='z', map_file=str(tmp_path / 'arena.json'))
    s.set_car(CAR, (0, 0, 0, 1))
    return s


def _feed(state, pts, ids=None):
    pts = list(map(tuple, pts)) + [CAR]
    state.set_markers(ids if ids is not None else list(range(len(pts))), pts)


def _make(state, pts, track, order='as_given'):
    _feed(state, pts)
    return state.add_obstacle({
        'kind': 'polygon', 'order': order, 'track': track,
        'points': [{'x': p[0], 'y': p[1], 'z': p[2]} for p in pts]})


def test_tracked_obstacle_follows_markers_and_static_one_stays(state):
    static = BOX + (1.0, 0.0, 0.0)
    _feed(state, list(BOX) + list(static))
    tracked = state.add_obstacle({'kind': 'polygon', 'order': 'as_given', 'track': True,
                                  'points': [dict(zip('xyz', p)) for p in BOX]})
    fixed = state.add_obstacle({'kind': 'polygon', 'order': 'as_given',
                                'points': [dict(zip('xyz', p)) for p in static]})
    v0, e0 = state.version, state.edit_version
    # carry the box 0.5 m and turn it 90 deg in small steps; relabel ids every frame
    for k in range(1, 51):
        pts = _move(BOX, 0.5 * k / 50, 0.0, math.pi / 2 * k / 50)
        _feed(state, list(pts[::-1]) + list(static), ids=list(range(100 * k, 100 * k + 9)))
    assert np.allclose(tracked.vertices, _move(BOX, 0.5, 0.0, math.pi / 2), atol=0.011)
    assert np.allclose(fixed.vertices, static)
    assert state.version > v0 and state.edit_version == e0
    snap = {o['id']: o for o in state.snapshot()['obstacles']}
    assert snap[tracked.id]['tracking'] == {'matched': 4, 'total': 4}
    assert 'tracking' not in snap[fixed.id]


def test_reshape_keeps_vertex_order(state):
    ob = _make(state, BOX, track=True)
    target = BOX.copy()
    target[2, :2] = (0.8, 0.7)             # drag one corner out
    for a in np.linspace(0, 1, 30):
        _feed(state, BOX + a * (target - BOX))
    assert np.allclose(ob.vertices, target, atol=0.011)


def test_occluded_marker_is_carried_then_reacquired(state):
    ob = _make(state, BOX, track=True)
    for k in range(1, 31):
        pts = _move(BOX, 0.3 * k / 30, 0.2 * k / 30, 0.3 * k / 30)
        _feed(state, pts[[0, 1, 3]])       # corner 2 hidden the whole way
    final = _move(BOX, 0.3, 0.2, 0.3)
    assert np.allclose(state._tracks[ob.id]['pos'], final, atol=1e-6)
    assert state.snapshot()['obstacles'][0]['tracking']['matched'] == 3
    _feed(state, final)
    assert state.snapshot()['obstacles'][0]['tracking']['matched'] == 4


def test_lost_obstacle_holds_and_ignores_far_and_car_markers(state):
    ob = _make(state, BOX, track=True)
    _feed(state, [(1.5, 1.5, 0.2)])        # markers removed; a far marker appears
    assert np.allclose(ob.vertices, BOX)
    assert state.snapshot()['obstacles'][0]['tracking']['matched'] == 0
    state.set_car((0.0, 0.05, 0.1), (0, 0, 0, 1))
    _feed(state, [])
    state.set_markers([1], [(0.0, 0.0, 0.2)])  # a marker now within the car exclusion radius
    assert state.snapshot()['obstacles'][0]['tracking']['matched'] == 0


def test_track_flag_persists_and_old_files_default_static(state, tmp_path):
    ob = _make(state, BOX, track=True)
    state.save()
    assert load_map_file(state.map_file)['obstacles'][0].track
    s2 = MapState(GridSpec(), map_file=state.map_file)
    assert s2.obstacles[0].track and ob.id in s2._tracks
    s2.set_markers([0, 1, 2, 3], BOX + (0.02, 0.0, 0.0))
    assert np.allclose(s2.obstacles[0].vertices, BOX + (0.02, 0.0, 0.0))

    d = ob.to_dict()
    del d['track']
    assert not Obstacle.from_dict(d).track


def test_patch_toggles_tracking(state, tmp_path):
    client = create_app(state, str(tmp_path)).test_client()
    ob = _make(state, BOX, track=False)
    assert ob.id not in state._tracks
    assert client.patch(f'/api/obstacles/{ob.id}', json={'track': 'yes'}).status_code == 400
    r = client.patch(f'/api/obstacles/{ob.id}', json={'track': True})
    assert r.status_code == 200 and r.get_json()['track'] and ob.id in state._tracks
    _feed(state, BOX + (0.05, 0.0, 0.0))
    assert np.allclose(ob.vertices, BOX + (0.05, 0.0, 0.0))
    client.patch(f'/api/obstacles/{ob.id}', json={'track': False})
    _feed(state, BOX + (0.1, 0.0, 0.0))
    assert np.allclose(ob.vertices, BOX + (0.05, 0.0, 0.0)) and ob.id not in state._tracks
