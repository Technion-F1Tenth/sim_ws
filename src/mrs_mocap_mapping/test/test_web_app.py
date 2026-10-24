import base64

import numpy as np
import pytest

from mrs_mocap_mapping.grid import GridSpec
from mrs_mocap_mapping.map_state import MapState
from mrs_mocap_mapping.web_app import create_app

CONES = [(x, y, 0.3) for x, y in [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 1.4)]]


@pytest.fixture
def state(tmp_path):
    s = MapState(GridSpec(resolution=0.1, origin_x=-1, origin_y=-1, width=40, height=40),
                 up_axis='z', map_file=str(tmp_path / 'arena.json'))
    s.set_markers(list(range(10, 10 + len(CONES))) + [99], CONES + [(3.0, 3.0, 0.1)])
    s.set_car((3.0, 3.05, 0.1), (0, 0, 0, 1), [(3.0, 3.0, 0.1)])
    return s


@pytest.fixture
def client(state, tmp_path):
    return create_app(state, str(tmp_path)).test_client()


def _pts(idx):
    return [{'x': CONES[i][0], 'y': CONES[i][1], 'z': CONES[i][2]} for i in idx]


def test_state_flags_car_markers(client):
    st = client.get('/api/state').get_json()
    by_id = {m['id']: m for m in st['markers']}
    assert by_id[99]['excluded'] and not by_id[10]['excluded']
    assert st['car']['x'] == pytest.approx(3.0)


def test_create_preview_patch_delete(client, state):
    v0 = state.version
    pv = client.post('/api/preview', json={'kind': 'polygon', 'points': _pts([0, 1, 2, 3])})
    assert pv.status_code == 200 and state.version == v0 and not state.obstacles

    r = client.post('/api/obstacles', json={'kind': 'loop', 'width': 0.1,
                                            'points': _pts([2, 0, 3, 1, 4])})
    assert r.status_code == 201
    ob = r.get_json()
    assert ob['kind'] == 'loop' and len(ob['vertices']) == 5 and state.version == v0 + 1

    r = client.patch(f'/api/obstacles/{ob["id"]}', json={'name': 'track', 'inflation': 0.2})
    assert r.get_json()['name'] == 'track'
    assert client.patch(f'/api/obstacles/{ob["id"]}', json={'inflation': -1}).status_code == 400
    assert state.obstacles[0].inflation == 0.2

    occ = client.get('/api/occupancy').get_json()
    grid = np.frombuffer(base64.b64decode(occ['data']), np.int8)
    assert grid.size == 40 * 40 and (grid == 100).any()

    assert client.delete(f'/api/obstacles/{ob["id"]}').status_code == 200
    assert client.delete('/api/obstacles/nope').status_code == 404
    assert not state.obstacles


def test_rejects_car_marker_missing_marker_and_bad_width(client):
    car = [{'x': 3.0, 'y': 3.0, 'z': 0.1}]
    r = client.post('/api/obstacles', json={'kind': 'polygon', 'points': _pts([0, 1]) + car})
    assert r.status_code == 400 and 'car' in r.get_json()['error']
    ghost = [{'x': 9.0, 'y': 9.0, 'z': 0.0}]
    r = client.post('/api/obstacles', json={'kind': 'polygon', 'points': _pts([0, 1]) + ghost})
    assert r.status_code == 400 and 'no longer visible' in r.get_json()['error']
    r = client.post('/api/obstacles', json={'kind': 'polyline', 'points': _pts([0, 1])})
    assert r.status_code == 400
    r = client.post('/api/obstacles', json={'kind': 'polygon', 'points': _pts([0, 0, 1])})
    assert r.status_code == 400 and 'duplicates' in r.get_json()['error']


def test_selection_matches_relabelled_markers(client, state):
    # Same physical markers, new ids: requests are matched by position.
    state.set_markers([50, 51, 52, 53], [CONES[i] for i in range(4)])
    r = client.post('/api/obstacles', json={'kind': 'polygon', 'points': _pts([0, 1, 2, 3])})
    assert r.status_code == 201
    assert sorted(r.get_json()['source_marker_ids']) == [50, 51, 52, 53]


def test_grid_fit_save_and_reload(client, state, tmp_path):
    client.post('/api/obstacles', json={'kind': 'polygon', 'points': _pts([0, 1, 2, 3])})
    assert client.put('/api/grid', json={'resolution': 0}).status_code == 400
    spec = client.post('/api/grid/fit', json={'margin': 0.5}).get_json()
    assert spec['origin_x'] <= -0.5 and spec['resolution'] == 0.1
    saved = client.post('/api/save', json={}).get_json()
    for key in ('json', 'pgm', 'yaml'):
        assert (tmp_path / saved[key].split('/')[-1]).exists()

    again = MapState(GridSpec(), map_file=str(tmp_path / 'arena.json'))
    assert len(again.obstacles) == 1
    assert again.grid.to_dict() == state.grid.to_dict()
    assert (again.occupancy()[2] == state.occupancy()[2]).all()


def test_non_json_body_is_400(client):
    assert client.post('/api/obstacles', data='x').status_code == 400
