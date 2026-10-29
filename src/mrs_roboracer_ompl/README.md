# MRS RoboRacer OMPL Planner 
This is the package for the MRS RoboRacer OMPL planner, which integrates the Open Motion Planning Library (OMPL) into the MRS RoboRacer framework. This planner is designed to provide efficient and reliable path planning capabilities for autonomous racing vehicles.
## Dependencies
- ROS (Robot Operating System)
- OMPL (Open Motion Planning Library)
- tf2

See all dependencies in the `package.xml` file.

## Warnings
This package is still under development and may contain bugs or incomplete features. Use it at your own risk. 
The package was primarily developed and tested on Ubuntu 22.04 with ROS2 Humble. The simulation environment used for testing is [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros).

## Installation
OMPL comes from `ros-humble-ompl` (`rosdep install --from-paths src -y --ignore-src`).
To build against a fork with your own planners, clone it into `src/` (OMPL ships a
`package.xml`); colcon builds it and it overlays `/opt/ros/humble`.

This package only plans: it publishes a latched `nav_msgs/Path` and never a drive
command. Path tracking is left to the controller packages (e.g. `mrs_roboracer_pp_cpp`).
Launch it through `mrs_roboracer_bringup`:
```
ros2 launch mrs_roboracer_bringup ompl_pure_pursuit_launch.py mode:=sim
```

Planners (`planner_type`):
- `dubins` (default): geometric RRT* on Dubins curves, minimizing path length. Paths are
  forward-only with turns no tighter than `min_turning_radius`, which pure pursuit can follow.
- `rrt`: kinodynamic RRT over the (accel, steering-rate) car model. Not optimizing.
- `rcs`: RCSStar, built only when the external RCS project is found (`HAS_RCS`, see
  `CMakeLists.txt`).

## RCSStar Parameter Sweep
The planning node supports a built-in sweep mode that evaluates RCSStar across a parameter grid and fixed start/goal cases.

### Config
Edit the YAML file at `mrs_roboracer_ompl/config/rcs_sweep.yaml` to define:
- `grid`: parameter ranges (cartesian product)
- `cases`: fixed start/goal poses (x, y, yaw)
- `output_csv`: where to save results

### Run
Example for simulation:
```
ros2 launch mrs_roboracer_bringup ompl_pure_pursuit_launch.py mode:=sim planner_type:=rcs rcs_sweep:=true
```

Notes:
- Sweep waits for the map to be available before starting.
- While sweep runs, live goal topics are ignored.


