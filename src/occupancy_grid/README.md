# Occupancy Grid Node
## Background
Creates an [occupancy grid](https://en.wikipedia.org/wiki/Occupancy_grid_mapping) of the surroundings of the car. 
## Usage 
Launch with parameters in `config/params.yaml` using 

    ros2 launch occupancy_grid occupancy_grid.launch.py

Or run with default parameters 

    ros2 run occupancy_grid occupancy_grid_node

## Available parameters 

| Parameter      | Explanation                                      |
|----------------|--------------------------------------------------|
| map_width      | Width of the occupancy grid map in grid cells.   |
| map_height     | Height of the occupancy grid map in grid cells.  |
| resolution     | Size of each grid cell in meters.                |
| laser_frame    | Frame ID of the laser sensor on the racecar.     |
| laser_topic    | Topic for receiving laser scan data.             |
| odom_topic     | Topic for receiving odometry data from the racecar. |
| map_topic      | Topic for publishing the occupancy grid map.    |


**THE PARAMETERS FOR THE TOPICS NEED TO BE CHANGED WHEN MOVING FROM THE CAR TO THE SIMULATOR AND VICEVERSA**