"""
Obstacle model and vertex ordering (rclpy-free).

Obstacles are static snapshots of marker positions, stored in the map frame
(see `frames.py`) with full 3D vertices so the same file can later drive a
voxel map. Kinds:

  polygon     filled area bounded by the vertices.
  polyline    open thin wall: vertex chain swept by `width`.
  loop        closed thin wall: polyline whose last vertex connects back to
              the first, swept by `width`, NOT filled (e.g. a ring of cones).
  rigid_body  reserved: shape defined in a Motive rigid body's frame and
              moved by its live pose. Not implemented yet; rasterizers reject it.

Every obstacle is extruded vertically over [z_min, z_max] (used by the 3D
rasterizer only) and grown by `inflation` metres in the grid.
"""

from dataclasses import dataclass, field
import json
import os
import tempfile
from typing import List, Optional
import uuid

import numpy as np

SCHEMA_VERSION = 1

KINDS = ('polygon', 'polyline', 'loop', 'rigid_body')
SHAPE_KINDS = ('polygon', 'polyline', 'loop')
THIN_KINDS = ('polyline', 'loop')
MIN_VERTICES = {'polygon': 3, 'polyline': 2, 'loop': 3}

ORDERS = ('hull', 'as_given', 'auto')
DEFAULT_ORDER = {'polygon': 'hull', 'polyline': 'auto', 'loop': 'auto'}


@dataclass
class Obstacle:
    name: str
    kind: str
    vertices: np.ndarray
    width: float = 0.0
    z_min: float = 0.0
    z_max: float = 0.0
    inflation: float = 0.0
    source_marker_ids: List[int] = field(default_factory=list)
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])

    def __post_init__(self):
        self.vertices = np.asarray(self.vertices, dtype=float).reshape(-1, 3)
        self.validate()

    @property
    def closed(self) -> bool:
        return self.kind in ('polygon', 'loop')

    @property
    def thin(self) -> bool:
        return self.kind in THIN_KINDS

    def validate(self) -> None:
        if self.kind not in KINDS:
            raise ValueError(f'unknown obstacle kind {self.kind!r} (expected one of {KINDS})')
        if self.kind == 'rigid_body':
            return
        n_min = MIN_VERTICES[self.kind]
        if len(self.vertices) < n_min:
            raise ValueError(f'{self.kind} needs at least {n_min} vertices, '
                             f'got {len(self.vertices)}')
        if self.thin and not self.width > 0.0:
            raise ValueError(f'{self.kind} needs a positive width')
        if self.inflation < 0.0:
            raise ValueError('inflation must be >= 0')
        if self.z_max < self.z_min:
            raise ValueError('z_max must be >= z_min')

    def xy_bounds(self, pad: float = 0.0):
        """Return (xmin, xmax, ymin, ymax) of the swept shape plus `pad`."""
        r = (self.width / 2.0 if self.thin else 0.0) + self.inflation + pad
        xy = self.vertices[:, :2]
        return (xy[:, 0].min() - r, xy[:, 0].max() + r, xy[:, 1].min() - r, xy[:, 1].max() + r)

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'name': self.name,
            'kind': self.kind,
            'vertices': self.vertices.round(5).tolist(),
            'width': self.width,
            'z_min': self.z_min,
            'z_max': self.z_max,
            'inflation': self.inflation,
            'source_marker_ids': list(self.source_marker_ids),
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'Obstacle':
        return cls(
            id=str(d['id']),
            name=str(d['name']),
            kind=str(d['kind']),
            vertices=d['vertices'],
            width=float(d.get('width', 0.0)),
            z_min=float(d.get('z_min', 0.0)),
            z_max=float(d.get('z_max', 0.0)),
            inflation=float(d.get('inflation', 0.0)),
            source_marker_ids=[int(i) for i in d.get('source_marker_ids', [])],
        )


def convex_hull_2d(points) -> np.ndarray:
    """Return indices of the CCW convex hull of (N, >=2) points (monotone chain)."""
    xy = np.asarray(points, dtype=float)[:, :2]
    idx = sorted(range(len(xy)), key=lambda i: (xy[i, 0], xy[i, 1]))

    def cross(o, a, b):
        return ((xy[a, 0] - xy[o, 0]) * (xy[b, 1] - xy[o, 1])
                - (xy[a, 1] - xy[o, 1]) * (xy[b, 0] - xy[o, 0]))

    lower, upper = [], []
    for i in idx:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], i) <= 0:
            lower.pop()
        lower.append(i)
    for i in reversed(idx):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], i) <= 0:
            upper.pop()
        upper.append(i)
    return np.array(lower[:-1] + upper[:-1], dtype=int)


def _nearest_neighbour(d: np.ndarray, start: int) -> list:
    n = len(d)
    tour, used = [start], np.zeros(n, dtype=bool)
    used[start] = True
    for _ in range(n - 1):
        row = np.where(used, np.inf, d[tour[-1]])
        nxt = int(np.argmin(row))
        tour.append(nxt)
        used[nxt] = True
    return tour


def _two_opt(d: np.ndarray, tour: list) -> list:
    """Uncross a closed tour in place with 2-opt until no move shortens it."""
    n, eps = len(tour), 1e-12
    improved = True
    while improved:
        improved = False
        for i in range(n - 2):
            for j in range(i + 2, n if i > 0 else n - 1):
                a, b = tour[i], tour[i + 1]
                c, e = tour[j], tour[(j + 1) % n]
                if d[a, c] + d[b, e] < d[a, b] + d[c, e] - eps:
                    tour[i + 1:j + 1] = tour[i + 1:j + 1][::-1]
                    improved = True
    return tour


def auto_order(points, closed: bool) -> np.ndarray:
    """
    Order points into a short, non-crossing path or loop.

    Nearest-neighbour construction followed by 2-opt. An open path is solved
    as a loop through an extra dummy point at zero distance from every
    marker, then cut at the dummy, so its endpoints are chosen freely.
    Intended for tens to a couple of hundred markers (cones along a boundary).
    """
    xy = np.asarray(points, dtype=float)[:, :2]
    n = len(xy)
    if n <= 2:
        return np.arange(n)
    d = np.linalg.norm(xy[:, None, :] - xy[None, :, :], axis=-1)
    if not closed:
        d = np.pad(d, ((0, 1), (0, 1)))
    tour = _two_opt(d, _nearest_neighbour(d, 0))
    if not closed:
        k = tour.index(n)
        tour = tour[k + 1:] + tour[:k]
    return np.array(tour, dtype=int)


def resolve_order(points, kind: str, order: Optional[str]) -> np.ndarray:
    """Return the vertex index order for `kind` given an ordering mode."""
    order = order or DEFAULT_ORDER[kind]
    if order not in ORDERS:
        raise ValueError(f'unknown order {order!r} (expected one of {ORDERS})')
    n = len(points)
    if order == 'as_given':
        return np.arange(n)
    if order == 'hull':
        if kind == 'polyline':
            raise ValueError("order 'hull' is not meaningful for an open polyline")
        hull = convex_hull_2d(points)
        if len(hull) < 3:
            raise ValueError('markers are collinear or coincident; convex hull is degenerate')
        if len(hull) < n and kind == 'loop':
            # A polygon may drop interior markers; a loop silently losing cones would not be ok.
            raise ValueError(f'{n - len(hull)} marker(s) are inside the hull; '
                             "use order 'auto' or 'as_given' for this loop")
        return hull
    return auto_order(points, closed=(kind == 'loop'))


def make_obstacle(name: str, kind: str, points, order: Optional[str] = None,
                  width: float = 0.0, inflation: float = 0.0,
                  z_min: Optional[float] = None, z_max: Optional[float] = None,
                  z_margin: float = 0.05, marker_ids=None) -> Obstacle:
    """Build an obstacle from marker positions (N, 3) in the map frame."""
    if kind not in SHAPE_KINDS:
        raise ValueError(f'cannot build a {kind!r} obstacle from markers')
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(pts) < MIN_VERTICES[kind]:
        raise ValueError(f'{kind} needs at least {MIN_VERTICES[kind]} markers, got {len(pts)}')
    ids = list(marker_ids) if marker_ids is not None else []
    idx = resolve_order(pts, kind, order)
    z_lo = float(pts[:, 2].min()) - z_margin if z_min is None else float(z_min)
    z_hi = float(pts[:, 2].max()) + z_margin if z_max is None else float(z_max)
    return Obstacle(
        name=name or kind,
        kind=kind,
        vertices=pts[idx],
        width=float(width) if kind in THIN_KINDS else 0.0,
        z_min=min(z_lo, 0.0) if z_min is None else z_lo,
        z_max=z_hi,
        inflation=float(inflation),
        source_marker_ids=[int(ids[i]) for i in idx] if ids else [],
    )


def save_map_file(path: str, obstacles, grid_spec: dict, up_axis: str) -> None:
    """Atomically write obstacles + grid spec as JSON."""
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    data = {
        'schema_version': SCHEMA_VERSION,
        'frame': 'map',
        'up_axis': up_axis,
        'grid': grid_spec,
        'obstacles': [o.to_dict() for o in obstacles],
    }
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)), suffix='.tmp')
    with os.fdopen(fd, 'w') as f:
        json.dump(data, f, indent=2)
    os.chmod(tmp, 0o644)   # mkstemp creates 0600
    os.replace(tmp, path)


def load_map_file(path: str) -> dict:
    """Read a map JSON; returns dict with 'grid', 'up_axis' and Obstacle objects."""
    with open(os.path.expanduser(path)) as f:
        data = json.load(f)
    version = data.get('schema_version')
    if version != SCHEMA_VERSION:
        raise ValueError(f'{path}: unsupported schema_version {version!r}')
    return {
        'grid': data.get('grid'),
        'up_axis': data.get('up_axis'),
        'obstacles': [Obstacle.from_dict(o) for o in data.get('obstacles', [])],
    }
