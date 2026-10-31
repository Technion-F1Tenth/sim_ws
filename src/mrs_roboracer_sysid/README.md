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
  - `odom_to_pose_adapter.py` -- republishes a ground-truth `Odometry`
    (`/ego_racecar/odom` in sim, `/mocap_odom` with mocap) as
    `/sysid/pose` `PoseStamped` (+ `/sysid/twist` in sim only).
  - `mocap_pose_adapter.py` -- republishes one named
    `mocap4r2_msgs/RigidBodies` entry from `mocap4r2_optitrack_driver` as
    `PoseStamped`. No longer launched by `mrs_roboracer_bringup` (see §6).
  - `sysid_data_collector.py` -- the main node and state machine.
  - `bag_to_dataset.py` -- offline `rosbag2` -> flat-table extraction
    (`sysid_bag_to_dataset` console script).

## 3. Build

```bash
cd /home/mrslab3/sim_ws
colcon build --symlink-install --packages-select mrs_roboracer_sysid_msgs mrs_roboracer_sysid mrs_roboracer_bringup
source install/setup.bash
```

## 4. Quick start (sim/dev)

1. Launch the sim, adapter, collector and `safety_node` from `mrs_roboracer_bringup`
   (`mode:=sim` includes the f1tenth_gym_ros bridge):
   ```bash
   ros2 launch mrs_roboracer_bringup sysid_launch.py mode:=sim
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
   To stop a program early (e.g. the car is heading for a wall), call
   `/sysid/abort_path`. The car is commanded to zero and the bag keeps
   recording. Once the car is stationary, the node goes to `ARMED` for the
   next path (see §7).
5. After the last path, or after calling `/sysid/end_session` to stop early,
   the node stops the bag, writes `<bag_path>_coverage_report.json` and goes
   to `DONE`. Calling `start_session` again starts a new session with a new
   bag, so the node does not need a restart.

With `io.bag_path` empty (the default), each session records to a fresh
`<io.bag_dir>/sysid_<YYYYmmdd_HHMMSS>` (`io.bag_dir` defaults to
`~/sysid_bags`), and the `start_session` reply names it. An explicit
`io.bag_path` must not exist yet, because `ros2 bag record` refuses to write
into an existing directory. It only works for one session per node.

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
  argument (empty: auto-named under `io.bag_dir`, see §4); set `io.twist_topic`/`io.odom_topic` to `""` to disable those
  subscriptions/bag topics.
- **`velocity.*`** -- `velocity_source` is currently informational
  (`pose_derivative` is the only one the node runs online, for the SETTLE
  check); `sg_window`/`sg_order` tune the Savitzky-Golay filter used both
  there and in `bag_to_dataset.py`.
- **`sync.*`** -- sync-chirp amplitude/duration/speed, prepended to every
  path when `enable_sync_chirp` is true.
- **`visualization.*`** -- active-path visualization. When
  `visualization.publish_path` is true, the currently armed spline is
  published as `nav_msgs/Path` on `visualization.path_topic` (default
  `/sysid/path`) in `visualization.path_frame` (default `map`).
- **`safety.*`** -- `pose_timeout_ms` (watchdog threshold, checked at
  `watchdog_rate_hz` independent of the control loop), `path_timeout_s`
  (abort an in-progress path), `settle_v_eps`/`settle_confirm_s` (SETTLE
  exit condition), `settle_timeout_s` (after this long in SETTLE without
  coming to rest, log a warning; SETTLE keeps waiting and never ends the
  session, so the operator can drive the car back before it stops).

## 6. Mocap integration

Ground truth comes from a lab OptiTrack system via
[`mocap4ros2_optitrack`](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack)
(`mocap4r2_optitrack_driver`). The driver:

- Is a **lifecycle node** (`mocap4r2_optitrack_driver_node`).
  `mrs_roboracer_bringup/launch/optitrack_launch.py` starts, configures and
  activates it, and `mocap_map_launch.py` includes that file, so no manual
  `ros2 lifecycle set` is needed.
- Publishes `rigid_bodies` (`mocap4r2_msgs/msg/RigidBodies`): one message
  per mocap frame, `header` + `RigidBody[]`, each with a `rigid_body_name`
  (the numeric Motive Streaming ID as a string, e.g. `"4"`, **not** the
  asset name) and a `geometry_msgs/Pose`. `header.frame_id`
  is hardcoded `"map"` by the driver. There is **no native PoseStamped
  output and no twist/velocity output**.

The pit PC runs the driver plus `mrs_mocap_mapping`'s `mocap_map_node`,
which publishes the car as `/mocap_odom` (`nav_msgs/Odometry` of
`base_link` in `map`, with the `up_axis` and `body_to_base` corrections
applied). That is the mocap pose source for every `mode:=mocap` launch,
sysid included. On the car, `sysid_launch.py mode:=mocap` starts the car
hardware (f1tenth_stack), `odom_to_pose_adapter` (`/mocap_odom` ->
`/sysid/pose`), `sysid_data_collector` and `safety_node`:

```bash
# pit PC: OptiTrack driver (auto configured + activated) + mocap_map_node
ros2 launch mrs_roboracer_bringup mocap_map_launch.py car_rigid_body_name:=<streaming ID> up_axis:=y
# car: f1tenth_stack + adapter + collector + safety_node; the bag is recorded on the car
ros2 launch mrs_roboracer_bringup sysid_launch.py mode:=mocap \
    params_file:=<.../sysid_params_3x3.yaml>
```

Pass `launch_hardware:=false` if f1tenth_stack already runs on the car.
`sysid_launch.py` never starts the OptiTrack driver or `mocap_map_node`
itself -- `mocap_map_launch.py` on the pit PC is the only place that runs.

Both machines need the same `ROS_DOMAIN_ID`, and their clocks must be
synchronized (chrony, with the car as a client of the pit PC). `/mocap_odom`
is stamped with the pit clock (at receipt by `mocap_map_node`), while
`/sysid/meta` is stamped with the car clock, and `sysid_bag_to_dataset`
aligns them by header stamp. Any clock offset therefore shows up as fake
dead time. Check `chronyc tracking` on the car before each session.

Bags recorded before 2026-10-26 (`sysid_pit_launch.py` + `sysid_car_launch.py`)
have the raw rigid-body pose on `/mocap/pose` instead of `/sysid/pose`: pass
`--pose-topic /mocap/pose` to `sysid_bag_to_dataset` for those.

Notes:

- **No twist is bridged.** Since the driver has no velocity output,
  `twist_topic` defaults to `""`. Velocity comes from
  `velocity_estimators.PoseDerivativeEstimator` (pose-derivative), the same
  source already used for the online SETTLE check and as the primary
  offline `v_meas_l` column -- no new differentiation logic was added.
- Decide whether `io.odom_topic` (onboard VESC odometry) is available and
  worth recording alongside mocap ground truth.
- Re-tune `safety.pose_timeout_ms` for the mocap system's actual rate/latency.
- **Open/unverified: axis convention.** The driver does no axis conversion
  in code, so whether the rigid body's pose matches the ROS REP103
  (Z-up, right-handed) convention depends entirely on how the coordinate
  system is calibrated in Motive. Verify this empirically (e.g. check a
  known-stationary orientation) before trusting `theta_l`/yaw downstream.

## 7. Safety

- **Cmd-mux only**: this node publishes `AckermannDriveStamped` to
  `io.drive_topic` (`/safe_drive`) **only**. `mrs_roboracer_safety`'s
  `safety_node` arbitrates `/safe_drive -> /drive` (clipping + TTC AEB).
  This node never publishes `/drive`.
- **Stale-pose watchdog**: `PoseStalenessWatchdog` runs on an independent
  timer at `safety.watchdog_rate_hz`. If no pose has been received within
  `safety.pose_timeout_ms`, `/sysid/meta.flag_stale_pose=true`, `arm_path`
  is refused, and an in-progress `EXECUTE` aborts that program (see below).
- **No geofence / capture-volume enforcement.** `paths.workspace_bounds`
  only constrains where waypoints are *sampled* -- it is not checked at
  runtime. This is a research node: **run with an operator and RC override
  present at all times.**
- **Aborts end one program, not the session.** A stale pose,
  `safety.path_timeout_s` or `/sysid/abort_path` publishes zero and moves to
  `SETTLE`. The bag keeps recording, and once the car is stationary the
  node arms the next path. An abort on the last path ends the session
  (`DONE`). Each abort is appended to `<bag_path>_aborted.json` (also listed
  under `aborted` in the coverage report), naming the path and the segment
  that was cut short. `sysid_bag_to_dataset` reads that file and sets
  `flag_segment_aborted` on that segment, and `sysid_fit_input_noise` drops
  it. Segments that finished before the abort are kept. The aborted program
  is not retried, so its remaining segments are missing from the
  session.

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
| `path_id`, `step_index`, `phase` | from `/sysid/meta` (`phase`: 3 = EXECUTE) |
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
| Stale-pose watchdog fires | Dry run: stop `odom_to_pose_adapter` (or the mocap bridge) mid-`EXECUTE`. Within `safety.pose_timeout_ms`, confirm `/sysid/meta.flag_stale_pose=true`, `/safe_drive` goes to zero, and the node moves to `SETTLE` and then `ARMED` for the next path, with the bag still recording (check logs and `<bag_path>_aborted.json`). |
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

## 13. Rate-input noise identification (`paths.mode: rate_steps`)

For the rate-input kinematic bicycle used by the planner,

```
v_dot = u1 + w1     (acceleration, m/s^2)
delta_dot = u2 + w2 (steering rate, rad/s)
theta_dot = v tan(delta) / L,  x_dot = v cos(theta),  y_dot = v sin(theta)
```

the collector can instead run **held-rate-step programs** (`rate_steps.py`,
enabled by `paths.mode: "rate_steps"`, which is the default in
`config/sysid_params.yaml`). Each program is: sync chirp, then up to
`steps.segments_per_path` segments that each hold `(u1, u2)` constant for
`steps.min_segment_s`..`max_segment_s`, then a straight brake to a stop. The
car only takes speed/angle setpoints, so the held rates are integrated into
ramped `(v, delta)` setpoints at `rate.control_rate_hz`. Consecutive segments
are continuous in `(v, delta)`.

How segments are chosen: the least-visited `(u1, u2)` cell for the current
speed band (`steps.v_band_edges`) that is feasible under a perfect-tracking
rollout. Feasible means:
- speed stays within the limits,
- `|delta|` stays at least `steps.delta_margin` below `delta_max`,
- lateral acceleration stays within `limits.a_lat_max_safe`,
- the car stays inside `steps.max_excursion_m` of the **arm pose**, counting
  stopping distance.

Steering segments (`u2 != 0`) are only scheduled at `v >= steps.v_min_steer`
and when `v_mean * T >= steps.min_arc_m`. The session log and
`<bag_path>_coverage_report.json` (`input_coverage`) report planned segment
counts per `(u1, u2, speed band)` cell.

`/sysid/meta` carries `segment_id` (-1 outside measurement segments),
`u1_cmd` and `u2_cmd`.

**Operation:** excursion is measured from the pose at each `arm_path`, so
drive the car back to the centre of the capture volume (RC) before each arm.
The rollout assumes perfect setpoint tracking, so leave margin in
`max_excursion_m`. Disable or relax the TTC AEB only if the volume is clear,
because any `/drive` override invalidates the rest of that program (see
below).

**Small volumes:** `config/sysid_params_3x3.yaml` is tuned for a 3 x 3 m
space. It uses `max_excursion_m` 1.2, `v_max` 2.0, a shortened chirp and 60
programs, which comes to about 115 segments and about 70 arc-gated steering
segments. Pass it with `params_file:=`. In that space `u1 = +3` never fits,
and `u2 = +/-3` only fits while braking. After each program, SETTLE waits
until the car is stationary for `settle_confirm_s` and then goes back to
ARMED. It never ends the session, so you can drive the car back to the
centre at any point and then call `arm_path`.

### Offline

```bash
ros2 run mrs_roboracer_sysid sysid_bag_to_dataset <bag> steps.csv \
    --pose-topic /sysid/pose --twist-topic "" --odom-topic "" \
    --segments-output segments.csv --wheelbase 0.33 --lag-s 0.0 --min-arc-m 0.2
ros2 run mrs_roboracer_sysid sysid_fit_input_noise segments.csv [more.csv ...] \
    --output fit.json --plot-dir plots/
```

Per-segment residuals (`segments.py`) are one-segment predictions starting
from the **measured** state at the segment start (`a`), evaluated at its end
(`b`):

| Channel | Residual | Propagated to input level |
|---|---|---|
| Acceleration | `r_v = v_b - (v_a + u1 T)` | `accel_resid = r_v / T` |
| Steering rate | `r_yawrate = yawrate_b - v_b tan(delta_a + u2 T) / L`, with `delta_a = atan(L yawrate_a / v_a)` | `steer_rate_resid = L cos^2(delta_b) / (v_b T) * r_yawrate` |

Notes:
- Steering angle is never measured. By default it is inferred from the
  measured yaw rate at the segment start. The `*_cmd` columns use the
  commanded angle instead, which picks up error from earlier segments and
  any steering trim offset.
- `gate_arc` is `0.5 (v_a + v_b) T >= min_arc_m`.
- `flag_safety_override` marks a segment where a `/drive` message differed
  from `/safe_drive` (clipping or AEB). `flag_after_override` extends that
  mark to the rest of the program.
- `sysid_fit_input_noise` drops stale or overridden segments and requires
  both endpoint speeds `>= --min-speed`. The steering channel also requires
  `gate_arc` and no steering saturation. Each channel is then fit with a
  zero-mean Gaussian MLE (`sigma^2 = mean(r^2)`) truncated at `+/- k sigma`.
- **Check the bias diagnostics before trusting the zero-mean fit.** Each
  channel reports `mean` (constant bias, e.g. steering trim) and
  `corr_with_u` / `slope_vs_u` (the residual against that channel's
  commanded rate). Uncompensated first-order lag gives a residual of
  roughly `-(tau/T) * (u - u_prev)`, where `u_prev` is the previous
  segment's rate. Since `u_prev` is roughly independent of `u`, the slope
  against `u` comes out near `-tau/T`: speeding up reads low and braking
  reads high. Over a
  balanced set of segments those errors cancel in `mean`, so look at
  `corr_with_u`. A clearly negative correlation means lag compensation is
  needed (next subsection).

### First-order lag compensation

The planner assumes `v_dot = u1` and `delta_dot = u2`. A real speed loop
(VESC) and steering servo lag their setpoints. Model each one as first
order:

```
v_dot = (v_sp - v) / tau_v          delta_dot = (delta_sp - delta) / tau_delta
```

With `steps.tau_v` / `steps.tau_delta` set, the generator publishes led
setpoints, which are the exact inverse of that lag for a held rate:

```
v_sp = v_ref + tau_v * u1           delta_sp = delta_ref + tau_delta * u2
```

The actual `(v, delta)` then follows the reference ramps, so the residuals
measure noise around the planner's own model. Details:
- The setpoint jumps by `tau * (change in u)` at each segment boundary.
  That jump is intended.
- `v_sp` is clamped at 0, so the car never reverses.
- The feasibility checks also apply to the led setpoints, so a steering
  lead never pushes past `delta_max - delta_margin`.
- `/sysid/meta` logs both the published setpoints (`v_cmd`/`delta_cmd`)
  and the reference (`v_ref`/`delta_ref`). The steps CSV has both, and the
  segment table's `*_cmd_start/end` columns use the reference.
- **Whatever executes planner trajectories on the car must apply the same
  lead with the same tau.** Otherwise sigma was identified for a different
  actuator chain than the one being driven.

Pure dead time is not removed by this lead. Account for it offline with
`--lag-s`.

#### Workflow

1. **Uncompensated session** (`tau_v = tau_delta = 0`). Extract it with
   `sysid_bag_to_dataset <bag> steps.csv --segments-output segments.csv`.
2. **Fit tau offline from `steps.csv`.** One row per 50 Hz control tick,
   with columns `t_cmd`, published setpoints `v_cmd`/`delta_cmd`, measured
   `v_meas_l` and `thetadot_meas_l` (pose-derivative values at the tick),
   and `path_id`/`segment_id`.
   - **Speed:** within each program (`path_id`), simulate
     `v_hat_{k+1} = v_hat_k + dt / tau_v * (v_cmd_{k-d} - v_hat_k)`,
     starting from `v_hat = v_meas_l` at the first row with
     `segment_id >= 0`. Minimise `sum (v_meas_l - v_hat)^2` over `tau_v`,
     plus an integer dead time `d` in ticks if needed (that dead time is
     your `--lag-s`). Use only rows with `segment_id >= 0`; the sync chirp
     is a pure setpoint step and is also a good isolated step response to
     check tau against.
   - **Steering:** same procedure, using the angle measured from yaw rate,
     `delta_meas = atan(L * thetadot_meas_l / v_meas_l)`, on rows with
     `v_meas_l >= ~0.5 m/s` (the inversion is ill-conditioned near
     standstill). Compare it against `delta_cmd`.
   - **Sanity check:** `segments.csv` from this session should show
     `slope_vs_u` close to `-tau / T_mean` in each channel.
3. **Compensated session** with the fitted `steps.tau_v` / `steps.tau_delta`
   (new `bag_path`). Extract it and run `sysid_fit_input_noise`. Each
   channel's `corr_with_u` should now be near 0 and `mean` small compared
   with `sigma`. Report sigma from this session.
