import numpy as np
import pytest

from mrs_mocap_mapping.grid import (export_pgm_yaml, fit_to_obstacles, GridSpec, OCCUPIED,
                                    rasterize_2d, rasterize_3d)
from mrs_mocap_mapping.obstacles import make_obstacle, Obstacle


def _square(x0, y0, s):
    return np.array([[x0, y0, 0], [x0 + s, y0, 0], [x0 + s, y0 + s, 0], [x0, y0 + s, 0]], float)


def test_square_hits_expected_cells_row_is_y():
    # 0.1 m cells, origin (-1, -2). Square x in [0, 0.5], y in [0, 0.3] (vertices on cell edges
    # are nudged inward so the boundary stroke does not spill into neighbours).
    spec = GridSpec(resolution=0.1, origin_x=-1.0, origin_y=-2.0, width=30, height=40)
    pts = np.array([[0.01, 0.01, 0], [0.49, 0.01, 0], [0.49, 0.29, 0], [0.01, 0.29, 0]])
    ob = make_obstacle('sq', 'polygon', pts)
    g = rasterize_2d(spec, [ob])
    rows, cols = np.nonzero(g == OCCUPIED)
    assert set(cols) == set(range(10, 15))       # x: (0 - -1) / 0.1 = 10 .. 14
    assert set(rows) == set(range(20, 23))       # y: (0 - -2) / 0.1 = 20 .. 22
    assert len(rows) == 15
    assert g.dtype == np.int8 and g.shape == (40, 30)


def test_inflation_grows_by_expected_cells():
    spec = GridSpec(resolution=0.1, origin_x=-1.0, origin_y=-1.0, width=20, height=20)
    base = make_obstacle('sq', 'polygon', _square(-0.19, -0.19, 0.38))
    infl = make_obstacle('sq', 'polygon', _square(-0.19, -0.19, 0.38), inflation=0.2)
    a = rasterize_2d(spec, [base]) == OCCUPIED
    b = rasterize_2d(spec, [infl]) == OCCUPIED
    ra, ca = np.nonzero(a)
    rb, cb = np.nonzero(b)
    assert rb.min() == ra.min() - 2 and rb.max() == ra.max() + 2
    assert cb.min() == ca.min() - 2 and cb.max() == ca.max() + 2
    assert b[a].all() and b.sum() > a.sum()


def test_polyline_width_and_open_ends():
    spec = GridSpec(resolution=0.05, origin_x=-1.0, origin_y=-1.0, width=40, height=40)
    wall = make_obstacle('w', 'polyline', np.array([[-0.5, 0.0, 0], [0.5, 0.0, 0],
                                                    [0.5, 0.5, 0]]), width=0.1, order='as_given')
    g = rasterize_2d(spec, [wall]) == OCCUPIED
    # Column through x = 0: occupied cells span ~width around y = 0.
    col = int((0.0 - spec.origin_x) / spec.resolution)
    occ_rows = np.nonzero(g[:, col])[0]
    ys = spec.origin_y + (occ_rows + 0.5) * spec.resolution
    assert ys.min() >= -0.05 - 1e-9 and ys.max() <= 0.05 + 1e-9 and len(occ_rows) == 2
    # The open chain must not be closed back from (0.5, 0.5) to (-0.5, 0).
    r = int((0.25 - spec.origin_y) / spec.resolution)
    c = int((0.0 - spec.origin_x) / spec.resolution)
    assert not g[r, c]


def test_loop_leaves_interior_free():
    spec = GridSpec(resolution=0.05, origin_x=-2.0, origin_y=-2.0, width=80, height=80)
    loop = make_obstacle('cones', 'loop', _square(-1.0, -1.0, 2.0), width=0.1)
    g = rasterize_2d(spec, [loop]) == OCCUPIED
    centre = (40, 40)
    assert not g[centre]
    # Closing edge (x = -1) is present.
    assert g[40, int((-1.0 + 2.0) / 0.05)]


def test_obstacle_outside_grid_is_ignored_and_clipped():
    spec = GridSpec(resolution=0.1, origin_x=0.0, origin_y=0.0, width=10, height=10)
    far = make_obstacle('far', 'polygon', _square(5.0, 5.0, 1.0))
    edge = make_obstacle('edge', 'polygon', _square(-0.5, 0.21, 0.58), inflation=0.1)
    g = rasterize_2d(spec, [far, edge])
    assert g.shape == (10, 10)
    assert (g[:, 0] == OCCUPIED).any()


def test_rigid_body_is_skipped():
    spec = GridSpec(resolution=0.1, width=10, height=10)
    rb = Obstacle(name='rb', kind='rigid_body', vertices=np.zeros((0, 3)))
    assert (rasterize_2d(spec, [rb]) == 0).all()


def test_rasterize_3d_respects_z_range():
    spec = GridSpec(resolution=0.1, origin_x=-1, origin_y=-1, origin_z=0.0,
                    width=20, height=20, depth=10)
    ob = make_obstacle('box', 'polygon', _square(-0.29, -0.29, 0.58), z_min=0.0, z_max=0.3)
    vox = rasterize_3d(spec, [ob])
    assert vox[:3].any(axis=(1, 2)).all()
    assert not vox[3:].any()


def test_fit_to_obstacles_covers_everything():
    obs = [make_obstacle('a', 'polygon', _square(1.0, 2.0, 1.0)),
           make_obstacle('b', 'polyline', np.array([[-3, 0, 0], [-2, 0.5, 0]], float), width=0.2)]
    spec = fit_to_obstacles(obs, 0.05, margin=0.5)
    assert spec.origin_x <= -3.6 and spec.origin_y <= -0.6
    assert spec.origin_x + spec.width * 0.05 >= 2.5
    assert spec.origin_y + spec.height * 0.05 >= 3.5
    with pytest.raises(ValueError):
        fit_to_obstacles([], 0.05, 0.5)


def test_export_pgm_yaml_flips_rows(tmp_path):
    spec = GridSpec(resolution=0.5, origin_x=1.0, origin_y=2.0, width=3, height=2)
    g = np.zeros((2, 3), np.int8)
    g[0, 0] = OCCUPIED                              # bottom-left cell
    pgm, yaml_path = export_pgm_yaml(str(tmp_path / 'm'), spec, g)
    raw = open(pgm, 'rb').read()
    pixels = np.frombuffer(raw[-6:], np.uint8).reshape(2, 3)
    assert pixels[1, 0] == 0 and pixels[0, 0] == 254   # occupied cell is on the image's last row
    text = open(yaml_path).read()
    assert 'image: m.pgm' in text and 'origin: [1.0, 2.0, 0.0]' in text
