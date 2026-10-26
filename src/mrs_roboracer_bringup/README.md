# mrs_roboracer_bringup

One launch per workflow, and each starts everything that workflow needs.
Every driving launch takes a required `mode`:

| mode | runs on | platform started | `odom` (safety TTC, speed) | `pose` (map frame) |
|---|---|---|---|---|
| `sim` | sim PC | f1tenth_gym_ros `gym_bridge_launch.py` | `/ego_racecar/odom` | `/ego_racecar/odom` (ground truth) |
| `onboard` | car | f1tenth_stack `bringup_launch.py` (+ map_server + particle_filter if the controller needs a pose) | `/odom` (VESC) | `/pf/pose/odom` |
| `mocap` | car + pit PC | f1tenth_stack on the car; `mocap_map_launch.py` on the pit PC | `/odom` (VESC) | `/mocap_odom` |

`/scan` and `/drive` are the same in every mode. Controllers publish
`/safe_drive`, and `safety_node` is the only writer of `/drive`. On the car,
`/drive` is the `ackermann_mux` navigation input, so the joystick (priority
100) still overrides it. The table is in `mrs_roboracer_bringup/modes.py`.
Each launch declares `scan_topic:=`, `odom_topic:=` (and `pose_topic:=` where used) to override a single entry.

`mode` has no default on purpose, so a launch on the car can never start the
sim (whose fake `/scan` would feed the AEB), and vice versa.

## Workflows

```bash
# Reactive gap follower (needs only /scan)
ros2 launch mrs_roboracer_bringup reactive_driver_safety_launch.py mode:=sim|onboard|mocap

# RL policy (needs /scan + odom speed)
ros2 launch mrs_roboracer_bringup inference_launch.py mode:=sim|onboard|mocap

# System ID (needs ground truth: no onboard mode)
ros2 launch mrs_roboracer_bringup sysid_launch.py mode:=sim|mocap [params_file:=...]

# Obstacle mapping / mocap station (pit PC); also the pit side of every mode:=mocap launch
ros2 launch mrs_roboracer_bringup mocap_map_launch.py car_rigid_body_name:=<streaming ID> up_axis:=y
```

Common platform arguments, accepted by every driving launch:

- `launch_hardware:=false`: don't start f1tenth_stack (it already runs on the car).
- `map:=<map .yaml>`: map for onboard localization. Required when the
  controller needs a pose in `mode:=onboard`. `mocap_map_node` exports
  (`~/mocap_maps/*.yaml`) work.
- `launch_mocap:=true car_rigid_body_name:=<id>`: in `mode:=mocap`, run the
  OptiTrack driver and `mocap_map_node` on this machine instead of the pit PC.

For mocap, both machines must share `ROS_DOMAIN_ID` and be chrony-synced:
`/mocap_odom` is stamped with the pit clock.

## Building blocks

- `platform_launch.py mode:=...`: just the platform (e.g. `mode:=onboard`
  for joystick-only driving).
- `optitrack_launch.py`: the OptiTrack driver, configured and activated
  automatically.
- `localization_launch.py map:=...`: nav2 `map_server`, `lifecycle_manager`
  and f1tenth `particle_filter` (`config/particle_filter.yaml`). Set the
  initial pose on `/initialpose` (Foxglove/RViz pose estimate).

Packages are looked up only for the selected mode, so the sim PC doesn't need
f1tenth_stack and the car doesn't need f1tenth_gym_ros. Prerequisites that
are **not** in this workspace:

- f1tenth_stack (`~/roboracer_ws`)
- `particle_filter` + range_libc (https://github.com/f1tenth/particle_filter),
  plus `ros-humble-nav2-map-server` and `ros-humble-nav2-lifecycle-manager`,
  for onboard localization.
- f1tenth_gym_ros for sim.

## Adding a controller (e.g. pure pursuit)

Copy `reactive_driver_safety_launch.py`:

1. Declare `mode` (`MODE_DESCRIPTION`), `topic_arguments(...)` for the
   topics it uses, and `safety_arguments(max_speed=...)`.
2. In an `OpaqueFunction`, call `resolve_topics(context)` (pass
   `allowed=(...)` to restrict modes), build the controller node from
   `topics[...]` publishing to `safe_drive_topic`, and return it with
   `safety_node(topics)`.
3. Add `include_platform(localization='true')` if it reads `topics['pose']`,
   otherwise `'false'`.
