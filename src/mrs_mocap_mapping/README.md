# mrs_mocap_mapping

Build a `nav_msgs/OccupancyGrid` from OptiTrack markers. Put markers on or around the obstacles, select them in a browser, and the node publishes the map in the mocap frame for the car.

## Run

```bash
# 1. OptiTrack driver (separate mocap workspace), lifecycle-activated as for sysid.
# 2. Map node + editor:
ros2 launch mrs_roboracer_bringup mocap_map_launch.py car_rigid_body_name:=<Motive asset> up_axis:=y
# 3. Open http://<host>:8080/
```

| Output | Type | Notes |
|---|---|---|
| `/mocap_map` | `nav_msgs/OccupancyGrid` | transient_local (latched), republished on every edit and every `map_publish_period` |
| `/mocap_map/obstacles` | `visualization_msgs/MarkerArray` | obstacle outlines + live markers (car markers grey) |
| `/mocap_odom` | `nav_msgs/Odometry` | car `base_link` pose in `map` (child `mocap_base_link`), twist in the body frame |
| `/mocap_scan` | `sensor_msgs/LaserScan` | `/scan` relayed with `frame_id = mocap_laser` |
| TF `map → mocap_base_link → mocap_laser` | | per mocap frame + static `base_to_laser` |

All parameters are documented in `config/mocap_map_params.yaml`.

## Car pose, TF and scans

```
map ──(mocap, per frame)──> mocap_base_link ──(static base_to_laser)──> mocap_laser
odom ─> base_link ─> laser          (the car's own tree, untouched)
```

The mocap tree runs **parallel** to the car's: the car already publishes `odom → base_link`, and a TF frame can have only one parent, so mocap can't also parent `base_link`. To draw the car's scans in the mocap frame, the node republishes `/scan` as `/mocap_scan` with `frame_id = mocap_laser`. Controllers that want ground truth should subscribe to `/mocap_odom` and not use TF.

- **Calibrate the two offsets on the real car.** `body_to_base` is `[x, y, z, yaw]` from the Motive pivot to the rear axle, in the body frame; its yaw also fixes a body whose +x isn't forward. `base_to_laser` must match the car's real `base_link → laser`; the default is the f1tenth_gym value 0.275 m. Check by parking next to a cone: the `/mocap_scan` points should land on it.
- **Twist** comes from a least-squares line fit over the last `twist_window` mocap samples (default 9). It lags by `(window − 1) / 2` samples (~33 ms at 120 Hz) and resets when tracking drops out. Until the window fills, the twist is zero with a 1e6 covariance.
- **Timing:** TF, `/mocap_odom` and `/mocap_scan` are stamped with this node's clock, because the mocap PC and the car don't share one. That's fine for display and control at mocap rates; it isn't for exact time alignment of scans to poses (sync the hosts with chrony first).
- **Later option:** if something needs the standard ROS (REP-105) chain `map → odom → base_link` (nav2 costmaps, AMCL-style tools), a `map → odom` mode can be added that computes `T_map_odom = T_map_base · inv(T_odom_base)`.

## Workflow

1. **Check the up axis.** The Status panel shows the raw per-axis marker spread. Markers mostly sit near the floor, so the up axis is the one with the smallest spread, and the UI warns if that disagrees with `up_axis`. Everything downstream is a right-handed Z-up `map` frame (`y`: map = (x, −z, y)).
2. **Select markers.** Click to select (numbered in click order) or shift-drag to box-select. Car markers can't be selected: a marker counts as the car's if it is within `car_exclusion_radius` of the car or within `car_marker_tolerance` of one of the car body's markers.
3. **Pick a shape:**
   - **Polygon**: filled area. Order is the convex hull (default) or *as clicked* for non-convex shapes.
   - **Thin wall** (`polyline`): open chain swept by *width*.
   - **Closed loop** (`loop`): closed chain swept by *width* and **not filled**, e.g. cones with markers on top. Order *auto* (default) runs nearest-neighbour + 2-opt, so it untangles any click order. Check the numbered preview before creating.
4. Optionally set **inflation** per obstacle, either when you create it or later in the list.
5. **Fit to obstacles** sizes the grid. **Save map** writes `map_file` (JSON) plus `<name>.pgm/.yaml`, which `nav2_map_server` can load. On startup an existing `map_file` is loaded, and it overrides the `grid.*` parameters.

Obstacles are **static snapshots**. The vertices are the marker positions at creation, so the markers can be removed afterwards. Requests match markers by position (`marker_match_tolerance`), not by id, because OptiTrack re-labels unlabeled markers when they drop out.

## Code layout

`frames.py`, `odometry.py`, `obstacles.py`, `grid.py`, `map_state.py` and `web_app.py` don't use rclpy and are unit-tested (`python3 -m pytest -p no:anyio test -q`). `mocap_map_node.py` is the only ROS module. `test/fake_mocap_publisher.py [--up-axis y]` publishes a fake scene (a car driving a circle, a cone ring, a box and a wall) plus a ray-cast `/scan` (frame `laser`), for trying the UI and the scan relay without the rig.

## Extension points (3D / drones, live obstacles)

- Obstacle vertices are stored in full 3D, with a vertical extent `[z_min, z_max]`. By default the extent runs from the floor to the highest marker + `z_margin`. `grid.rasterize_3d` already turns obstacles into a `bool[depth, height, width]` voxel grid. A drone map needs a publisher for it (e.g. an `OctoMap` or `PointCloud2`) and a `depth` in the grid spec, which `fit_to_obstacles` already fills in.
- `kind: rigid_body` is reserved in the schema for obstacles that follow a Motive rigid body: the shape lives in the body frame and is moved by the live pose. The rasterizers skip it for now. Implementing it means storing a body-frame footprint and having the node re-rasterize when that body's pose changes.

## Known limitations

- `mocap4r2_msgs` field names (`Markers.markers[].marker_index/translation`, `RigidBodies.rigidbodies[].rigid_body_name/pose/markers`) follow upstream mocap4r2. The package was tested against a stand-in built from that layout, not against the lab's mocap workspace.
- A bare `y` is a YAML boolean, and ROS hands it to the node as `True`. The node maps `True` back to `'y'`, so `up_axis:=y` works anyway.
- The web UI has no authentication. It binds to `0.0.0.0` by default, so set `web.host: 127.0.0.1` on untrusted networks.
