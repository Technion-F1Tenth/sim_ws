# SIM WS Technion F1Tenth 
## Setup 
To be able to use the repository you need to have the following dependencies installed 

1. [F1Tenth Gym](https://github.com/f1tenth/f1tenth_gym)
2. [ROS Bridge for F1Tenth Gym](https://github.com/f1tenth/f1tenth_gym_ros)
3. ROS2 Foxy

The setup is known to work on a Ubuntu 20.04 VM in macOS (Intel). It folllows that it should work on native Ubuntu as well as probably on Windows VMs. The bridge to Foxglove directly from the VM is not yet working. It is probably a networking issue. Will update as soon as it is resolved.
## Usage 
After installing the dependencies.

    git clone https://github.com/andykamin3/sim_ws
    cd sim_ws 
    colcon build --symlink-install
    cd install/setup.bash

Then you can run any of the nodes using 

    ros2 run [package name] [node_name]

Or its corresponding launch file 

    ros2 launch [package_name] [launch_file]

### Important note
The current configuration is intended for the behavior node to manage the top level behavior of the car. Although not yet completed, in its current form is the intermediary between the `/drive` topic and the different nodes. It is recommended to run it in the background all the time. It also allows to pause the car by running the following line in any Terminal window
    ros2 topic pub /behavior_command std_msgs/msg/String "{data: 'BRAKE'}"
You can resume driving 
    ros2 topic pub /behavior_command std_msgs/msg/String "{data: 'REACTIVE'}"

It is recommended to use the launch file as it will launch both the behavior and reactive_node simultaneously.



## Available Nodes 

| Node Name      | Package Name | Has launch file
| ----------- | ----------- | ----------- |
| reactive_node      | driving       |  yes |
| occupancy_grid_node      | occupancy_grid       |  yes |
| pure_pursuit | pure_pursuit | yes 
| safety | safety_node | no 
| waypoint_logger | waypoint_logger | no


