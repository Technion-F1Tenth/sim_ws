# mrs_roboracer_sysid

Open-loop motion-noise data collection for the rear-axle kinematic bicycle
model used by the kinodynamic MCTS planner.

## 1. Overview

The kinodynamic MCTS planner rolls out a kinematic-bicycle transition model
`x_{l+1} = x_l + f(x_l, u_l + w_l) * dt`. Nobody has characterized the
process noise `w` for this car. This package drives the car open-loop
through a randomized, feasibility-checked sweep of `(v, kappa)` commands,
spanning the operating envelope (including the high-lateral-accel regime
where the kinematic model is expected to break), and logs everything --
commands, ground-truth pose/twist/odom, and per-step metadata -- to a
`rosbag2`.

**This package only collects data.** It does not compute residuals, fit a
noise model, or evaluate `f(x_l, u_l)`. `motion_model.py` (the exact model
under test, including `jacobian_df_du`) and `bag_to_dataset.py` (offline
extraction into a flat per-step table) exist to make that *future* fitting
task clean and unambiguous.

## 2. Package contents

Two ROS packages, because rosidl message generation requires `ament_cmake`:

- **`mrs_roboracer_sysid_msgs`** (ament_cmake/rosidl): defines
  `SysidMeta.msg`, published on `/sysid/meta` alongside every command.
- **`mrs_roboracer_sysid`** (ament_python): everything else --
  - `motion_model.py` -- pure rear-axle kinematic bicycle model (RK4 step,
    curvature/steering conversions, `jacobian_df_du`, `quaternion_to_yaw`).
    rclpy-free; importable offline.
  - `spline_paths.py` -- random waypoint sampling, spline fitting,
    feasibility checking, speed-profile assignment, and inversion to
    `(v, delta)` command sequences.
  - `coverage.py` -- `(v, kappa)` bin occupancy tracker and path-template
    scheduler (`speed_sweep`, `curvature_sweep`, `figure8`, `corner_probe`).
  - `velocity_estimators.py` -- Savitzky-Golay pose-derivative, mocap-twist,
    and onboard-odom speed estimators (used online only for the SETTLE
    check; reused offline by `bag_to_dataset.py`).
  - `pose_watchdog.py` -- `PoseStalenessWatchdog`, the only runtime safety
    check (no geofence -- out of scope, see §7).
  - `sync_chirp.py` -- steering-step sync-chirp maneuver prepended to every
    generated path.
  - `odom_to_pose_adapter.py` -- **dev-only** node that republishes
    `/ego_racecar/odom` as `PoseStamped`/`TwistStamped` so the node can be
    developed/tested in sim without a mocap system.
  - `sysid_data_collector.py` -- the main node and state machine.
  - `bag_to_dataset.py` -- offline `rosbag2` -> flat-table extraction
    (`sysid_bag_to_dataset` console script).

## 3. Build

```bash
cd /home/mrslab3/sim_ws
colcon build --symlink-install --packages-select mrs_roboracer_sysid_msgs mrs_roboracer_sysid
source install/setup.bash
```

## 4. Quick start (sim/dev)

1. Launch the f1tenth_gym_ros sim separately (not part of this package).
2. Launch this package's sim bringup (adapter + collector + `safety_node`):
   ```bash
   ros2 launch mrs_roboracer_sysid sysid_sim_launch.py
   ```
3. Start a session (generates all `paths.N` paths, starts `ros2 bag record`):
   ```bash
   ros2 service call /sysid/start_session std_srvs/srv/Trigger
   ```
4. Arm and run each path one at a time, with an operator and RC override
   ready (see §7):
   ```bash
   ros2 service call /sysid/arm_path std_srvs/srv/Trigger
   ```
   Each call executes one generated path, then waits in `SETTLE` until the
   car is stationary before becoming `ARMED` for the next path.
5. After the last path, the node writes
   `<io.bag_path>_coverage_report.json` and stays in `DONE`. To start
   another session, **restart the node with a different `io.bag_path`**
   (`ros2 bag record` refuses to write into an existing directory; see §7).

Monitor `/sysid/meta` (`phase` field mirrors the state machine: `IDLE=0,
GENERATE=1, ARMED=2, EXECUTE=3, SETTLE=4, DONE=5`) and `/safe_drive` to
follow progress.

## 5. Configuration

All parameters live in `config/sysid_params.yaml`, grouped by dotted prefix:

- **`model.*`** -- wheelbase `L`, `ref_point` (`"rear_axle"` only --
  `"front_axle"` raises `NotImplementedError`, see §12), steering limits.
- **`limits.*`** -- `v_min`/`v_max`, `a_max` (longitudinal), and two
  lateral-accel ceilings: `a_lat_max_safe` (normal paths) vs.
  `a_lat_probe_max` (allowed only for `corner_probe` paths).
- **`paths.*`** -- `N` paths per session, spline order (5 = quintic,
  falls back to cubic), `rng_seed` (logged prominently at session start
  for reproducibility), and **`workspace_bounds`**: an axis-aligned
  `[x_min, x_max, y_min, y_max]` box used only to *sample waypoints* --
  it is **not** a runtime safety/geofence check (none exists; see §7).
  Tune this to your actual usable space.
- **`coverage.*`** -- the `(v, kappa)` bin edges, `target_revisits_per_bin`,
  and scheduling knobs (`prioritize_corner_first`, `corner_probe_paths`,
  `figure8_period`/`figure8_radius`/`figure8_speed`).
- **`rate.control_rate_hz`** -- main control-loop / command rate.
- **`io.*`** -- topic names. `io.drive_topic` (default `/safe_drive`) is
  the **only** topic this node publishes commands to -- it never publishes
  `/drive` directly (see §7). `io.bag_path` is the `ros2 bag record -o`
  argument; set `io.twist_topic`/`io.odom_topic` to `""` to disable those
  subscriptions/bag topics.
- **`velocity.*`** -- `velocity_source` is currently informational
  (`pose_derivative` is the only one the node runs online, for the SETTLE
  check); `sg_window`/`sg_order` tune the Savitzky-Golay filter used both
  there and in `bag_to_dataset.py`.
- **`sync.*`** -- sync-chirp amplitude/duration/speed, prepended to every
  path when `enable_sync_chirp` is true.
- **`safety.*`** -- `pose_timeout_ms` (watchdog threshold, checked at
  `watchdog_rate_hz` independent of the control loop), `path_timeout_s`
  (abort an in-progress path), `settle_v_eps`/`settle_confirm_s` (SETTLE
  exit condition), `settle_timeout_s` (SETTLE fallback to `IDLE`).

## 6. Mocap integration (deferred)

`sysid_mocap_launch.py` is structurally identical to the sim launch but
drops `odom_to_pose_adapter` (the mocap bridge is expected to publish
`PoseStamped`/`TwistStamped` directly). Before using it:

- Pin `pose_topic`/`twist_topic` to the real mocap bridge's topic names
  (currently placeholders `/mocap/pose`, `/mocap/twist`).
- Confirm the mocap twist frame convention. `bag_to_dataset.py`'s
  `v_meas_l_mocap_twist` column assumes **world-frame** `(vx, vy)`
  projected onto heading (`MocapTwistEstimator`); if the mocap twist is
  body-frame, that column will be wrong until this is corrected.
- Decide whether `io.odom_topic` (onboard VESC odometry) is available and
  worth recording alongside mocap ground truth.
- Re-tune `safety.pose_timeout_ms` for the mocap system's actual rate/latency.

## 7. Safety

- **Cmd-mux only**: this node publishes `AckermannDriveStamped` to
  `io.drive_topic` (`/safe_drive`) **only**. `mrs_roboracer_safety`'s
  `safety_node` arbitrates `/safe_drive -> /drive` (clipping + TTC AEB).
  This node never publishes `/drive`.
- **Stale-pose watchdog**: `PoseStalenessWatchdog` runs on an independent
  timer at `safety.watchdog_rate_hz`. If no pose has been received within
  `safety.pose_timeout_ms`, `/sysid/meta.flag_stale_pose=true`, `arm_path`
  is refused, and an in-progress `EXECUTE` aborts to `IDLE` (publishing a
  zero command).
- **No geofence / capture-volume enforcement.** `paths.workspace_bounds`
  only constrains where waypoints are *sampled* -- it is not checked at
  runtime. This is a research node: **run with an operator and RC override
  present at all times.**
- **Abort/DONE recovery requires a fresh bag path.** Both an aborted
  session (stale pose / path timeout / SETTLE timeout) and a completed
  (`DONE`) session stop the bag recording and leave the node in a state
  where `start_session` either is refused (`DONE`) or would try to
  `ros2 bag record` into the same (now-existing) `io.bag_path` directory,
  which `ros2 bag record` refuses. **Restart the node with a new
  `io.bag_path`** to start a fresh session.

## 8. Sync chirp

Every generated path is prepended with a short steering-step maneuver
(`sync_chirp.py`): hold the path's initial steering angle briefly, step to
`base_delta + sync.chirp_amplitude` (clipped to `model.delta_max`) at
`sync.chirp_speed` for `sync.chirp_duration`, then step back. All chirp
steps have `/sysid/meta.flag_sync_chirp=true`. The distinctive,
steering-step-induced heading-rate transient is intended for offline
cross-correlation between commanded `delta` and measured heading rate, to
estimate and verify a stable command-to-measurement time lag. **That
cross-correlation/lag-estimation step is a future task** -- this package
only guarantees the chirp data is present in every path.

## 9. Offline workflow

```bash
ros2 run mrs_roboracer_sysid sysid_bag_to_dataset \
    <bag_path> <output.csv> \
    --pose-topic /sysid/pose --twist-topic /sysid/twist \
    --odom-topic /ego_racecar/odom --meta-topic /sysid/meta \
    --sg-window 9 --sg-order 2 --format csv
```

Output columns (one row per executed command step):

| Column | Meaning |
|---|---|
| `path_id`, `step_index` | from `/sysid/meta` |
| `t_cmd` | `/sysid/meta.header.stamp` (= intended command time) |
| `t_pose_start`/`t_pose_end` | nearest raw pose samples at-or-before `t_cmd` of this step / the next step (**no interpolation**) |
| `dt` | `t_pose_end - t_pose_start` (actual measured interval; `NaN` for the last step of a session) |
| `x_l`,`y_l`,`theta_l` / `x_lp1`,`y_lp1`,`theta_lp1` | pose at `t_pose_start` / `t_pose_end` |
| `v_meas_l` | Savitzky-Golay pose-derivative speed at `t_pose_start` (`velocity_estimators.PoseDerivativeEstimator`) |
| `v_cmd`,`delta_cmd`,`kappa_cmd` | commanded values from `/sysid/meta` |
| `coverage_bin` | `/sysid/meta.coverage_bin_id` (`-1` = outside the configured `(v,kappa)` grid, e.g. chirp/SETTLE) |
| `lat_accel_est` | **commanded** `v_cmd^2 * kappa_cmd` (not measured) |
| `flag_sync_chirp`,`flag_high_lat_accel`,`flag_stale_pose` | booleans from `/sysid/meta` |
| `v_meas_l_mocap_twist`,`v_meas_l_onboard_odom` | optional, present only if `--twist-topic`/`--odom-topic` resolve to data in the bag |

This tool explicitly does **not** compute `f(x_l, u_l)`, residuals `e_l`,
`B_l = (df/du)*dt`, coverage/binning statistics, or chirp cross-correlation
-- those are future work. `motion_model.jacobian_df_du` exists for the
`B_l` computation when that task starts.

## 10. Acceptance / validation checklist

| Check | How |
|---|---|
| Executed commands match the inverted spline | Compare `v_cmd`/`delta_cmd` from `bag_to_dataset.py` output against the persisted `GeneratedPath.commands` in `<paths.persist_dir>/path_<NNN>.npz` for the same `path_id`/`step_index`. |
| Logged `dt`s uniform at `rate.control_rate_hz`; jitter bounded | `diff(t_cmd)` in the extracted dataset should be `~= 1/control_rate_hz`; histogram `/sysid/meta.loop_jitter_s` and confirm it stays within a few ms. |
| Sync chirp data present every path | Confirm rows with `flag_sync_chirp=true` exist at the start of every `path_id`, with the expected hold/step/step-back `delta_cmd` pattern. (Cross-correlation itself is future work.) |
| Stale-pose watchdog fires | Dry run: stop `odom_to_pose_adapter` (or the mocap bridge) mid-`EXECUTE`. Within `safety.pose_timeout_ms`, confirm `/sysid/meta.flag_stale_pose=true`, `/safe_drive` goes to zero, and the node returns to `IDLE` (check logs). |
| Coverage target met, including corner | After a full `N`-path session, inspect `<io.bag_path>_coverage_report.json`: `bins_met`/`bins_total`/`pct_met`, and confirm the corner bin (`v_idx=n_v-1`, `kappa_idx in {0, n_kappa-1}`) has `visits >= target_revisits_per_bin`. If not, increase `paths.N` or `coverage.corner_probe_paths` and run another session (new `io.bag_path`). |

## 11. Replay / reproducibility

- `paths.rng_seed` is logged at `start_session`. The same seed + config
  reproduces the same generated paths (modulo `scipy`/`numpy` version
  differences in spline fitting).
- All generated paths are persisted to `<paths.persist_dir>/path_<NNN>.npz`
  (default `persist_dir`, when empty, is `<io.bag_path>_paths`) --
  waypoints, spline coefficients, the full command sequence, and the
  coverage target -- for replay/debugging independent of the bag.

## 12. Known limitations / TODOs

- `model.ref_point="front_axle"` is declared but **not implemented**
  (`motion_model.step`/`jacobian_df_du` raise `NotImplementedError`); only
  `"rear_axle"` is supported.
- `paths.check_self_intersection` defaults to `false` (simple O(n^2)
  polyline check, available but off by default).
- `velocity.velocity_source` is not yet pinned to a single source for
  offline fitting -- `bag_to_dataset.py` emits all available sources
  (`v_meas_l` from pose derivative, plus optional `v_meas_l_mocap_twist`/
  `v_meas_l_onboard_odom`) for comparison.
- Mocap integration is deferred (§6): topic names are placeholders and the
  mocap twist frame convention is unverified.
- No geofence/capture-volume enforcement (§7) -- intentional, per current
  requirements; `paths.workspace_bounds` is sampling-domain only.
