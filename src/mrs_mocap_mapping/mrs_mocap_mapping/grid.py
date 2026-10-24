"""
Grid spec and obstacle rasterization (rclpy-free).

Cell (row, col) covers x in [ox + col*res, ox + (col+1)*res) and
y in [oy + row*res, oy + (row+1)*res); a cell is occupied when its centre is
inside the obstacle. `rasterize_2d` returns int8[height, width] in the
row-major, row = +y layout nav_msgs/OccupancyGrid expects (0 free,
100 occupied). `rasterize_3d` extrudes the same footprints over each
obstacle's [z_min, z_max] into bool[depth, height, width] (voxel hook for the
drone extension; not published yet).
"""

from dataclasses import asdict, dataclass
import math
import os

import numpy as np
from scipy import ndimage

from .obstacles import Obstacle

FREE = 0
OCCUPIED = 100
MAX_CELLS = 25_000_000


@dataclass
class GridSpec:
    resolution: float = 0.05
    origin_x: float = -5.0
    origin_y: float = -5.0
    origin_z: float = 0.0
    width: int = 200
    height: int = 200
    depth: int = 1

    def __post_init__(self):
        self.resolution = float(self.resolution)
        self.origin_x, self.origin_y, self.origin_z = (
            float(self.origin_x), float(self.origin_y), float(self.origin_z))
        self.width, self.height, self.depth = int(self.width), int(self.height), int(self.depth)
        if not self.resolution > 0.0:
            raise ValueError('grid resolution must be > 0')
        if min(self.width, self.height, self.depth) < 1:
            raise ValueError('grid width/height/depth must be >= 1 cell')
        if self.width * self.height > MAX_CELLS:
            raise ValueError(f'grid too large ({self.width}x{self.height} cells)')

    @classmethod
    def from_metres(cls, resolution, origin_x, origin_y, size_x, size_y,
                    origin_z=0.0, size_z=None) -> 'GridSpec':
        depth = 1 if size_z is None else max(1, math.ceil(size_z / resolution))
        return cls(resolution, origin_x, origin_y, origin_z,
                   max(1, math.ceil(size_x / resolution)),
                   max(1, math.ceil(size_y / resolution)), depth)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> 'GridSpec':
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})

    def window(self, xmin, xmax, ymin, ymax):
        """Return (r0, r1, c0, c1) cell slice bounds covering a box, clipped to the grid."""
        res = self.resolution
        c0 = max(0, math.floor((xmin - self.origin_x) / res))
        c1 = min(self.width, math.ceil((xmax - self.origin_x) / res) + 1)
        r0 = max(0, math.floor((ymin - self.origin_y) / res))
        r1 = min(self.height, math.ceil((ymax - self.origin_y) / res) + 1)
        return r0, r1, c0, c1

    def cell_centres(self, r0, r1, c0, c1):
        """Return (X, Y) meshgrids of cell centres for the slice, shape (r1-r0, c1-c0)."""
        res = self.resolution
        xs = self.origin_x + (np.arange(c0, c1) + 0.5) * res
        ys = self.origin_y + (np.arange(r0, r1) + 0.5) * res
        return np.meshgrid(xs, ys)


def _points_in_polygon(x, y, poly) -> np.ndarray:
    """Even-odd test of points (arrays x, y) against a closed (N, 2) polygon."""
    inside = np.zeros(x.shape, dtype=bool)
    x0, y0 = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
    for xa, ya, xb, yb in zip(x0, y0, x1, y1):
        if ya == yb:
            continue
        crosses = (ya > y) != (yb > y)
        x_int = xa + (y - ya) * (xb - xa) / (yb - ya)
        inside ^= crosses & (x < x_int)
    return inside


def _near_segments(x, y, verts, closed, radius) -> np.ndarray:
    """Points within `radius` of any segment of the (N, 2) vertex chain."""
    near = np.zeros(x.shape, dtype=bool)
    segs = list(zip(verts[:-1], verts[1:]))
    if closed:
        segs.append((verts[-1], verts[0]))
    r2 = radius * radius
    for a, b in segs:
        box = ((x >= min(a[0], b[0]) - radius) & (x <= max(a[0], b[0]) + radius)
               & (y >= min(a[1], b[1]) - radius) & (y <= max(a[1], b[1]) + radius))
        if not box.any():
            continue
        px, py = x[box] - a[0], y[box] - a[1]
        ab = b - a
        denom = float(ab @ ab)
        t = np.zeros_like(px) if denom == 0.0 else np.clip((px * ab[0] + py * ab[1]) / denom, 0, 1)
        dx, dy = px - t * ab[0], py - t * ab[1]
        near[box] |= dx * dx + dy * dy <= r2
    return near


def _footprint(spec: GridSpec, ob: Obstacle):
    """Rasterize one obstacle's inflated 2D footprint; returns (mask, (r0, r1, c0, c1))."""
    if ob.kind == 'rigid_body':
        raise NotImplementedError('rigid_body obstacles are not rasterized yet')
    res = spec.resolution
    infl_cells = math.ceil(ob.inflation / res) if ob.inflation > 0 else 0
    pad = res * (infl_cells + 1)
    r0, r1, c0, c1 = spec.window(*ob.xy_bounds(pad=pad))
    if r1 <= r0 or c1 <= c0:
        return None, None
    # Rasterize on a window padded beyond the grid edge so inflation near the
    # border behaves the same as in the interior, then crop.
    wr0, wr1 = r0 - infl_cells, r1 + infl_cells
    wc0, wc1 = c0 - infl_cells, c1 + infl_cells
    x, y = spec.cell_centres(wr0, wr1, wc0, wc1)
    verts = ob.vertices[:, :2]
    if ob.kind == 'polygon':
        # Fill + stroke the boundary so slivers narrower than a cell still show up.
        mask = _points_in_polygon(x, y, verts) | _near_segments(x, y, verts, True, res / 2.0)
    else:
        radius = max(ob.width / 2.0, res / 2.0)
        mask = _near_segments(x, y, verts, ob.kind == 'loop', radius)
    if infl_cells > 0:
        k = np.arange(-infl_cells, infl_cells + 1)
        disk = (k[:, None] ** 2 + k[None, :] ** 2) * res * res <= ob.inflation ** 2 + 1e-12
        mask = ndimage.binary_dilation(mask, structure=disk)
    mask = mask[infl_cells:mask.shape[0] - infl_cells, infl_cells:mask.shape[1] - infl_cells]
    return mask, (r0, r1, c0, c1)


def rasterize_2d(spec: GridSpec, obstacles) -> np.ndarray:
    grid = np.full((spec.height, spec.width), FREE, dtype=np.int8)
    for ob in obstacles:
        if ob.kind == 'rigid_body':
            continue
        mask, win = _footprint(spec, ob)
        if mask is None:
            continue
        r0, r1, c0, c1 = win
        grid[r0:r1, c0:c1][mask] = OCCUPIED
    return grid


def rasterize_3d(spec: GridSpec, obstacles) -> np.ndarray:
    vox = np.zeros((spec.depth, spec.height, spec.width), dtype=bool)
    zc = spec.origin_z + (np.arange(spec.depth) + 0.5) * spec.resolution
    for ob in obstacles:
        if ob.kind == 'rigid_body':
            continue
        mask, win = _footprint(spec, ob)
        if mask is None:
            continue
        layers = (zc >= ob.z_min - ob.inflation) & (zc <= ob.z_max + ob.inflation)
        r0, r1, c0, c1 = win
        vox[layers, r0:r1, c0:c1] |= mask[None, :, :]
    return vox


def fit_to_obstacles(obstacles, resolution: float, margin: float) -> GridSpec:
    """Smallest grid (aligned to `resolution`) covering all obstacles plus `margin`."""
    shapes = [o for o in obstacles if o.kind != 'rigid_body']
    if not shapes:
        raise ValueError('no obstacles to fit the grid to')
    b = np.array([o.xy_bounds(pad=margin) for o in shapes])
    xmin, ymin = b[:, 0].min(), b[:, 2].min()
    xmax, ymax = b[:, 1].max(), b[:, 3].max()
    ox = round(math.floor(xmin / resolution) * resolution, 9)
    oy = round(math.floor(ymin / resolution) * resolution, 9)
    zmin = min(o.z_min for o in shapes)
    zmax = max(o.z_max for o in shapes)
    return GridSpec.from_metres(resolution, ox, oy, xmax - ox, ymax - oy,
                                origin_z=zmin, size_z=max(zmax - zmin, resolution))


def export_pgm_yaml(base_path: str, spec: GridSpec, grid: np.ndarray) -> tuple:
    """
    Write <base>.pgm + <base>.yaml loadable by nav2_map_server.

    PGM rows run top (max y) to bottom, so the grid is flipped vertically.
    Returns (pgm_path, yaml_path).
    """
    base_path = os.path.expanduser(base_path)
    os.makedirs(os.path.dirname(os.path.abspath(base_path)), exist_ok=True)
    pgm_path, yaml_path = base_path + '.pgm', base_path + '.yaml'
    img = np.where(grid >= OCCUPIED, 0, np.where(grid < 0, 205, 254)).astype(np.uint8)[::-1]
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{spec.width} {spec.height}\n255\n'.encode())
        f.write(img.tobytes())
    with open(yaml_path, 'w') as f:
        f.write(f'image: {os.path.basename(pgm_path)}\n'
                'mode: trinary\n'
                f'resolution: {spec.resolution}\n'
                f'origin: [{spec.origin_x}, {spec.origin_y}, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.25\n')
    return pgm_path, yaml_path
