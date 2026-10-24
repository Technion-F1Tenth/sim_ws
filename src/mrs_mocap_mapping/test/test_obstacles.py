import math

import numpy as np
import pytest

from mrs_mocap_mapping.obstacles import (auto_order, convex_hull_2d, load_map_file,
                                         make_obstacle, Obstacle, save_map_file)


def _ring(n, r=2.0):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.column_stack([r * np.cos(a), r * np.sin(a), np.zeros(n)])


def _is_cyclic_order(order, n):
    """Check that `order` visits 0..n-1 consecutively (either direction, any start)."""
    steps = {(b - a) % n for a, b in zip(order, np.roll(order, -1))}
    return steps in ({1}, {n - 1})


def test_convex_hull_drops_interior_point():
    pts = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0.5, 0.5, 0]], dtype=float)
    hull = convex_hull_2d(pts)
    assert sorted(hull.tolist()) == [0, 1, 2, 3]


def test_auto_order_untangles_shuffled_cone_ring():
    rng = np.random.default_rng(0)
    n = 20
    perm = rng.permutation(n)
    pts = _ring(n)[perm]
    order = auto_order(pts, closed=True)
    assert sorted(order.tolist()) == list(range(n))
    assert _is_cyclic_order(perm[order], n)


def test_auto_order_open_line_starts_at_an_end():
    rng = np.random.default_rng(1)
    n = 12
    perm = rng.permutation(n)
    x = np.arange(n, dtype=float) * 0.5
    pts = np.column_stack([x, 0.01 * rng.standard_normal(n), np.zeros(n)])[perm]
    order = perm[auto_order(pts, closed=False)].tolist()
    assert order in (list(range(n)), list(range(n))[::-1])


def test_auto_order_open_l_shape():
    pts = np.array([[0, 0, 0], [0, 1, 0], [0, 2, 0], [1, 2, 0], [2, 2, 0]], dtype=float)
    perm = [3, 0, 4, 2, 1]
    order = np.array(perm)[auto_order(pts[perm], closed=False)].tolist()
    assert order in ([0, 1, 2, 3, 4], [4, 3, 2, 1, 0])


def test_make_obstacle_kinds_and_validation():
    sq = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)
    poly = make_obstacle('box', 'polygon', sq)
    assert poly.closed and not poly.thin and len(poly.vertices) == 4
    wall = make_obstacle('wall', 'polyline', sq[:2], width=0.1)
    assert wall.thin and not wall.closed
    with pytest.raises(ValueError):
        make_obstacle('wall', 'polyline', sq[:2])            # missing width
    with pytest.raises(ValueError):
        make_obstacle('wall', 'polyline', sq[:2], order='hull', width=0.1)
    with pytest.raises(ValueError):
        make_obstacle('tri', 'polygon', sq[:2])              # too few markers
    with pytest.raises(ValueError):
        make_obstacle('loop', 'loop', np.vstack([sq, [[0.5, 0.5, 0]]]), order='hull', width=0.1)


def test_default_z_range_reaches_floor():
    pts = np.array([[0, 0, 0.3], [1, 0, 0.35], [1, 1, 0.3]], dtype=float)
    ob = make_obstacle('cones', 'loop', pts, width=0.1, z_margin=0.05)
    assert ob.z_min == 0.0 and ob.z_max == pytest.approx(0.4)


def test_marker_ids_follow_vertex_order():
    pts = _ring(6)
    perm = [3, 1, 5, 0, 2, 4]
    ob = make_obstacle('loop', 'loop', pts[perm], width=0.1, marker_ids=[10 + i for i in perm])
    for v, mid in zip(ob.vertices, ob.source_marker_ids):
        assert np.allclose(v, pts[mid - 10])


def test_json_round_trip(tmp_path):
    obs = [make_obstacle('box', 'polygon', _ring(5), inflation=0.1),
           make_obstacle('loop', 'loop', _ring(8), width=0.2)]
    path = tmp_path / 'sub' / 'arena.json'
    save_map_file(str(path), obs, {'resolution': 0.05}, 'y')
    loaded = load_map_file(str(path))
    assert loaded['up_axis'] == 'y'
    assert loaded['grid'] == {'resolution': 0.05}
    for a, b in zip(obs, loaded['obstacles']):
        assert a.to_dict() == b.to_dict()
    assert isinstance(loaded['obstacles'][0], Obstacle)
