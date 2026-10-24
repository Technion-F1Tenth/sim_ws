# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace overview

ROS 2 Humble colcon workspace (`/home/mrslab3/sim_ws`) for an F1TENTH / RoboRacer car, used both in the f1tenth_gym simulator and on the physical car. All first-party packages are `ament_python` except `mrs_roboracer_sysid_msgs` (ament_cmake, needed for rosidl message generation).

- `src/f1tenth_gym_ros/` is an **upstream clone** (`f1tenth/f1tenth_gym_ros`, branch `dev-humble`) with its own `.git`, and it contains `f1tenth_gym/` (pip-installed editable into `.venv`). Treat it as third-party code. It is untracked in this repo.
- `src/MPPI_CONTROLLER_SPEC.md` is a planning spec for a future MPPI controller (not implemented yet). It refers to a `ReactivePlannerBase` interface that **does not exist in this workspace**. Flag that rather than inventing it.

## Environment & build

```bash
source .venv/bin/activate            # venv created with --system-site-packages (f1tenth_gym, scipy, pandas, numba)
source /opt/ros/humble/setup.bash
colcon build --symlink-install       # or: --packages-select <pkg> ...
source install/setup.bash
```

- `/usr/bin/python3` is the interpreter ROS uses. Anaconda is also installed on this machine, so make sure conda's python isn't the one on PATH when building or running.
- `mrs_roboracer_inference` needs PyTorch, which is **not currently installed** in the system python or in `.venv`.
- `mrs_roboracer_sysid` exec-depends on `mocap4r2_msgs`, which comes from a separate mocap workspace (not vendored here).

## Tests

```bash
colcon test --packages-select mrs_roboracer_sysid && colcon test-result --verbose
# Single test file / test, run directly from the package dir:
cd src/mrs_roboracer_sysid && python3 -m pytest test/test_spline_paths.py -q
python3 -m pytest test/test_motion_model.py -k <name> -q
```

Every package has the ament `test_flake8.py` / `test_pep257.py` / `test_copyright.py` linters, so flake8 and pep257 docstring violations show up as test failures. Real unit tests exist in `mrs_roboracer_sysid` and `mrs_roboracer_inference/test/test_policy_model.py`.

## Architecture: the cmd-mux contract

The key cross-package invariant: **controllers publish `AckermannDriveStamped` to `/safe_drive` only, and `mrs_roboracer_safety/safety_node` is the sole writer of `/drive`.** The safety node clips speed/steering (`max_speed`, `max_steering_angle`) and runs a TTC-based AEB from `/scan` + odom, latched for `aeb_hold_duration`. Any new controller must follow this pattern and be launched alongside `safety_node`.

Controllers (each publishes to `/safe_drive`):
- `mrs_reactive_driver`: simple gap-following driver, toggled through the `/reactive_driver/activate` (`std_srvs/SetBool`) service. `driver_commands_config.bash` adds `start_driver`/`stop_driver` aliases.
- `mrs_roboracer_inference`: RL policy (MinGRU, trained in PufferLib) running at 20 Hz. `policy_model.py` is copied verbatim from PufferLib. **`model/MODEL_INFO.md` is the source of truth** for the observation layout (4 stacked 64-beam scans + normalized speed = 257 floats), normalization constants (`MAX_RANGE=30`, `V_MAX=10`: training constants that must not be changed), and actions (steering *rate* + acceleration, which must be clamped by the integrator, not by the network). The node resamples any LaserScan onto the policy's 270° beam fan by interpolation.
- `mrs_roboracer_sysid`: open-loop system-ID data collector with two program generators selected by `paths.mode`: `spline` (smooth `(v, κ)`-coverage paths) or `rate_steps` (held `(u1=accel, u2=steering-rate)` segments for fitting rate-input noise; `rate_steps.py` → `segments.py` residuals → `sysid_fit_input_noise`) (state machine IDLE→GENERATE→ARMED→EXECUTE→SETTLE→DONE, driven by `/sysid/start_session` and `/sysid/arm_path` Trigger services). It starts and stops `ros2 bag record` itself, and publishes `SysidMeta` on `/sysid/meta` every tick. `SysidMeta.phase` constants must mirror `sysid_data_collector.State`. `motion_model.py`, `spline_paths.py`, `coverage.py`, and `velocity_estimators.py` are rclpy-free and unit-testable. `bag_to_dataset.py` (`sysid_bag_to_dataset`) is the offline extractor. See `src/mrs_roboracer_sysid/README.md` for the full workflow, config (`config/sysid_params.yaml`), and known limitations. Notable gotcha: a session that finished or aborted requires restarting the node with a new `io.bag_path`.

Map building (not a controller): `mrs_mocap_mapping/mocap_map_node` turns OptiTrack markers into obstacles through a Flask web editor (port 8080). It publishes a latched `/mocap_map` OccupancyGrid, `/mocap_map/obstacles` MarkerArray, `/mocap_odom` (car ground truth), and `/mocap_scan` (`/scan` re-framed). Its TF subtree `map → mocap_base_link → mocap_laser` is deliberately parallel to the car's `odom → base_link` (one TF parent per frame), so don't make it publish `base_link`. `frames.py`, `odometry.py`, `obstacles.py`, `grid.py`, `map_state.py` and `web_app.py` are rclpy-free and unit-tested. Obstacle kinds are `polygon`, `polyline` and `loop` (thin walls swept by a width), plus `rigid_body`, which is reserved. Vertices are kept in 3D and `rasterize_3d` exists for the future drone extension. See `src/mrs_mocap_mapping/README.md`. Its pytest run needs `-p no:anyio`, because a user-site anyio plugin breaks the system pytest.

`mrs_roboracer_bringup/launch/` wires controller + safety node together:
- `reactive_driver_safety_launch.py` (safety `max_speed` 3.0)
- `inference_launch.py` (safety `max_speed` 8.5, because the policy averages ~7.6 m/s)
- `sysid_sim_launch.py` (uses `odom_to_pose_adapter` to fake ground truth from `/ego_racecar/odom`)
- `sysid_mocap_launch.py` (needs `rigid_body_name:=<Motive asset>`; the OptiTrack driver must be launched and lifecycle-activated separately)
- `mocap_map_launch.py` (map editor only; needs `car_rigid_body_name:=<Motive asset>`, and `up_axis:=y|z`)

None of these launch the simulator. Start it separately:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py   # config: src/f1tenth_gym_ros/config/sim.yaml (map, num_agent, async_mode)
ros2 launch mrs_roboracer_bringup inference_launch.py
```

Sim topics: `/scan`, `/ego_racecar/odom`, `/drive`. Visualization is through Foxglove (`ws://localhost:8765`).
