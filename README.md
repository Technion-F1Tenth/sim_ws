## Usage 
After running in your terminal the following command (adjusting the path might be required):

```bash
chmod +x /sim_ws/scripts/simulation/*.sh
```

You can now run the following commands:
Note that all scripts launch a tmux and the gym from scratch. If you want to restart the scripts you must run the kill_processes.sh script


| Script Name | Description |
|-------------|-------------|
| reactive_drive_single_car.sh  | Launches a single car driving in a reactive node in the simulator alongside a foxglove instance  |
| reactive_drive_two_cars.sh  | Launches two cars driving with a reactive node in the simulator alongside a foxglove instance  |
| kill_processes.sh  | Kills all active processes. Required before restarting any other script. |

# Notes
Currently you need to manually modify the sim configuration to switch between one and two cars. This is especially important in the case of a single car script running for a two car sim since it will just not work. 
