"""
Thread-safe state shared by the ROS node and the Flask web app (rclpy-free).

The node feeds raw mocap data in (`set_markers`, `set_car`); the web app reads
snapshots and edits obstacles / the grid. Every change bumps `version`, which
the node watches to republish the OccupancyGrid; user edits also bump
`edit_version`, so the UI rebuilds its forms only for those and not for every
move of a tracked obstacle. The raster is cached and rebuilt only when
obstacles or the grid change.

Obstacles with `track` set follow their markers (see `tracking.py`): each
marker frame updates a per-vertex estimate, and the published vertices are
replaced once some vertex has moved more than `track_min_move`.

Obstacle requests reference markers by the position the UI last saw, not
only by id: OptiTrack re-labels unlabeled markers when they drop out, so an
id seen in the browser may name a different marker (or none) by the time
the request arrives. Each requested point is matched to the nearest current
marker within `match_tolerance`.
"""

import os
import threading
import time
from typing import Optional

import numpy as np

from . import frames
from .grid import export_pgm_yaml, fit_to_obstacles, GridSpec, rasterize_2d
from .obstacles import load_map_file, make_obstacle, save_map_file, THIN_KINDS
from .tracking import assign, rigid_fill

EDITABLE_FIELDS = ('name', 'inflation', 'width', 'z_min', 'z_max', 'track')


class MapState:

    def __init__(self, grid: GridSpec, up_axis: str = 'z', map_file: Optional[str] = None,
                 car_exclusion_radius: float = 0.3, car_marker_tolerance: float = 0.02,
                 match_tolerance: float = 0.05, z_margin: float = 0.05,
                 body_to_base=(0.0, 0.0, 0.0, 0.0), track_gate: float = 0.1,
                 track_min_move: float = 0.01, clock=time.monotonic):
        self._lock = threading.RLock()
        self.up_axis = frames.check_up_axis(up_axis)
        self.map_file = os.path.expanduser(map_file) if map_file else None
        self.car_exclusion_radius = car_exclusion_radius
        self.car_marker_tolerance = car_marker_tolerance
        self.match_tolerance = match_tolerance
        self.z_margin = z_margin
        self.track_gate = track_gate
        self.track_min_move = track_min_move
        self.body_to_base = tuple(float(v) for v in body_to_base)
        if len(self.body_to_base) != 4:
            raise ValueError('body_to_base must be [x, y, z, yaw]')
        self._clock = clock

        self.grid = grid
        self.obstacles = []
        self.version = 0
        self.edit_version = 0
        self._raster = None
        self._tracks = {}   # obstacle id -> {'pos': (V, 3) estimate, 'matched': int or None}
        self.load_note = ''

        self._marker_ids = np.zeros(0, dtype=int)
        self._marker_raw = np.zeros((0, 3))
        self._marker_map = np.zeros((0, 3))
        self._markers_stamp = None
        self._car = None
        self._car_stamp = None

        if self.map_file and os.path.exists(self.map_file):
            loaded = load_map_file(self.map_file)
            self.obstacles = loaded['obstacles']
            if loaded['grid']:
                self.grid = GridSpec.from_dict(loaded['grid'])
            if loaded['up_axis'] and loaded['up_axis'] != self.up_axis:
                self.load_note = (f'{self.map_file} was saved with up_axis={loaded["up_axis"]} '
                                  f'but the node runs with up_axis={self.up_axis}')
            self.version = self.edit_version = 1
            for ob in self.obstacles:
                self._sync_track(ob)

    # ---- live mocap input -------------------------------------------------

    def set_markers(self, ids, raw_positions) -> None:
        raw = np.asarray(raw_positions, dtype=float).reshape(-1, 3)
        with self._lock:
            self._marker_ids = np.asarray(ids, dtype=int)
            self._marker_raw = raw
            self._marker_map = frames.to_map_frame(raw, self.up_axis)
            self._markers_stamp = self._clock()
            self._track_obstacles()

    def _track_obstacles(self) -> None:
        tracked = [ob for ob in self.obstacles if ob.track]
        if not tracked:
            return
        live = self._marker_map[~self._excluded_mask()]
        prev = np.concatenate([self._tracks[ob.id]['pos'] for ob in tracked])
        idx = assign(prev, live, self.track_gate)
        moved, k = False, 0
        for ob in tracked:
            tr = self._tracks[ob.id]
            n = len(tr['pos'])
            i = idx[k:k + n]
            k += n
            matched = i >= 0
            new = tr['pos'].copy()
            new[matched] = live[i[matched]]
            tr['pos'] = rigid_fill(tr['pos'], new, matched)
            tr['matched'] = int(matched.sum())
            if np.linalg.norm(tr['pos'] - ob.vertices, axis=1).max() > self.track_min_move:
                ob.vertices = tr['pos'].copy()
                moved = True
        if moved:
            self._changed(edit=False)

    def _sync_track(self, ob) -> None:
        if ob.track and ob.id not in self._tracks:
            self._tracks[ob.id] = {'pos': ob.vertices.copy(), 'matched': None}
        elif not ob.track:
            self._tracks.pop(ob.id, None)

    def set_car(self, raw_position, raw_quat, raw_marker_positions=()):
        """
        Store the car pose; returns the base_link (position, quat) in the map frame.

        The rigid-body pivot is moved by `body_to_base` so the returned pose,
        TF and UI marker all refer to the car's base_link (rear axle).
        """
        body_pos = frames.to_map_frame(raw_position, self.up_axis)
        body_quat = frames.quat_to_map_frame(raw_quat, self.up_axis)
        pos, quat = frames.apply_body_offset(body_pos, body_quat, self.body_to_base)
        mk = np.asarray(raw_marker_positions, dtype=float).reshape(-1, 3)
        with self._lock:
            self._car = {
                'position': pos,
                'quat': quat,
                'body_position': body_pos,
                'raw_position': np.asarray(raw_position, dtype=float),
                'markers': frames.to_map_frame(mk, self.up_axis),
            }
            self._car_stamp = self._clock()
        return pos, quat

    def _excluded_mask(self) -> np.ndarray:
        excluded = np.zeros(len(self._marker_map), dtype=bool)
        if self._car is None or len(self._marker_map) == 0:
            return excluded
        d = np.linalg.norm(self._marker_map - self._car['body_position'], axis=1)
        excluded |= d <= self.car_exclusion_radius
        car_mk = self._car['markers']
        if len(car_mk):
            dm = np.linalg.norm(self._marker_map[:, None, :] - car_mk[None, :, :], axis=-1)
            excluded |= dm.min(axis=1) <= self.car_marker_tolerance
        return excluded

    def _age(self, stamp):
        return None if stamp is None else self._clock() - stamp

    def snapshot(self) -> dict:
        """JSON-able view of everything the UI draws."""
        with self._lock:
            excluded = self._excluded_mask()
            markers = [
                {'id': int(i), 'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]),
                 'excluded': bool(e)}
                for i, p, e in zip(self._marker_ids, self._marker_map, excluded)
            ]
            car = None
            if self._car is not None:
                p, q = self._car['position'], self._car['quat']
                car = {'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]),
                       'yaw': frames.yaw_from_quat(q),
                       'raw': [float(v) for v in self._car['raw_position']],
                       'age': self._age(self._car_stamp)}
            raw_range = None
            if len(self._marker_raw) >= 2:
                raw_range = [float(v) for v in np.ptp(self._marker_raw, axis=0)]
            return {
                'version': self.version,
                'edit_version': self.edit_version,
                'up_axis': self.up_axis,
                'markers': markers,
                'markers_age': self._age(self._markers_stamp),
                'raw_range': raw_range,
                'car': car,
                'obstacles': [self._obstacle_view(o) for o in self.obstacles],
                'grid': self.grid.to_dict(),
                'map_file': self.map_file,
                'load_note': self.load_note,
            }

    def _obstacle_view(self, ob) -> dict:
        d = ob.to_dict()
        tr = self._tracks.get(ob.id)
        if tr is not None:
            d['tracking'] = {'matched': tr['matched'], 'total': len(tr['pos'])}
        return d

    def car_pose(self):
        with self._lock:
            if self._car is None:
                return None
            return self._car['position'].copy(), self._car['quat'].copy()

    def live_markers(self):
        """Return (positions (N, 3), excluded (N,)) in the map frame."""
        with self._lock:
            return self._marker_map.copy(), self._excluded_mask()

    # ---- obstacle editing -------------------------------------------------

    def match_points(self, points):
        """Match UI-supplied [{x, y, z, id?}] to current markers; returns (positions, ids)."""
        if not points:
            raise ValueError('no markers selected')
        with self._lock:
            cur, ids, excluded = self._marker_map, self._marker_ids, self._excluded_mask()
            out_pos, out_ids, used = [], [], set()
            for k, p in enumerate(points):
                q = np.array([float(p['x']), float(p['y']), float(p.get('z', 0.0))])
                if len(cur) == 0:
                    raise ValueError('no live markers in the current mocap frame')
                d = np.linalg.norm(cur - q, axis=1)
                j = int(np.argmin(d))
                if d[j] > self.match_tolerance:
                    raise ValueError(f'selected marker #{k + 1} near ({q[0]:.3f}, {q[1]:.3f}) '
                                     'is no longer visible (moved or occluded)')
                if excluded[j]:
                    raise ValueError(f'selected marker #{k + 1} belongs to the car')
                if j in used:
                    raise ValueError(f'selected marker #{k + 1} duplicates another selection')
                used.add(j)
                out_pos.append(cur[j].copy())
                out_ids.append(int(ids[j]))
            return np.array(out_pos), out_ids

    def build_obstacle(self, body: dict):
        positions, ids = self.match_points(body.get('points'))
        kind = body.get('kind', 'polygon')
        width = float(body.get('width') or 0.0)
        if kind in THIN_KINDS and not width > 0.0:
            raise ValueError(f'{kind} needs a positive width')
        return make_obstacle(
            name=str(body.get('name') or ''),
            kind=kind,
            points=positions,
            order=body.get('order'),
            width=width,
            inflation=float(body.get('inflation') or 0.0),
            z_min=_opt_float(body.get('z_min')),
            z_max=_opt_float(body.get('z_max')),
            z_margin=self.z_margin,
            marker_ids=ids,
            track=bool(body.get('track', False)),
        )

    def add_obstacle(self, body: dict):
        ob = self.build_obstacle(body)
        with self._lock:
            if not body.get('name'):
                ob.name = f'{ob.kind}_{len(self.obstacles) + 1}'
            self.obstacles.append(ob)
            self._sync_track(ob)
            self._changed()
        return ob

    def update_obstacle(self, obstacle_id: str, fields: dict):
        with self._lock:
            ob = self._find(obstacle_id)
            unknown = set(fields) - set(EDITABLE_FIELDS)
            if unknown:
                raise ValueError(f'cannot edit fields {sorted(unknown)}')
            backup = ob.to_dict()
            for k, v in fields.items():
                if k == 'track' and not isinstance(v, bool):
                    raise ValueError('track must be true or false')
                setattr(ob, k, str(v) if k == 'name' else v if k == 'track' else float(v))
            try:
                ob.validate()
            except ValueError:
                for k in fields:
                    setattr(ob, k, backup[k])
                raise
            self._sync_track(ob)
            self._changed()
            return ob

    def delete_obstacle(self, obstacle_id: str) -> None:
        with self._lock:
            self.obstacles.remove(self._find(obstacle_id))
            self._tracks.pop(obstacle_id, None)
            self._changed()

    def _find(self, obstacle_id: str):
        for ob in self.obstacles:
            if ob.id == obstacle_id:
                return ob
        raise KeyError(obstacle_id)

    # ---- grid -------------------------------------------------------------

    def set_grid(self, d: dict) -> GridSpec:
        merged = {**self.grid.to_dict(), **d}
        spec = GridSpec.from_dict(merged)
        with self._lock:
            self.grid = spec
            self._changed()
        return spec

    def fit_grid(self, margin: float, resolution: Optional[float] = None) -> GridSpec:
        with self._lock:
            spec = fit_to_obstacles(self.obstacles, resolution or self.grid.resolution, margin)
            self.grid = spec
            self._changed()
            return spec

    def occupancy(self):
        """Return (version, GridSpec, int8[h, w]) -- the exact grid that is published."""
        with self._lock:
            if self._raster is None:
                self._raster = rasterize_2d(self.grid, self.obstacles)
            return self.version, self.grid, self._raster

    def _changed(self, edit: bool = True) -> None:
        self.version += 1
        if edit:
            self.edit_version += 1
        self._raster = None

    # ---- persistence ------------------------------------------------------

    def save(self) -> dict:
        if not self.map_file:
            raise ValueError('no map file configured (parameter map_file)')
        _, spec, grid = self.occupancy()
        with self._lock:
            save_map_file(self.map_file, self.obstacles, spec.to_dict(), self.up_axis)
        pgm, yaml_path = export_pgm_yaml(os.path.splitext(self.map_file)[0], spec, grid)
        return {'json': self.map_file, 'pgm': pgm, 'yaml': yaml_path}


def _opt_float(v):
    return None if v is None or v == '' else float(v)
