source /opt/ros/foxy/setup.bash
source ~/f1tenth_ws/install/setup.bash

# Create a new tmux session
tmux new-session -d -s my_session

# Create a new window and run the first command
tmux new-window -t my_session:1
tmux send-keys -t my_session:1 'ros2 launch f1tenth_gym_ros gym_bridge_launch.py' ENTER

# Create another new window and run the second command
tmux new-window -t my_session:2
tmux send-keys -t my_session:2 'ros2 launch driving reactive_node.launch.py' ENTER

# Attach to the tmux session
tmux attach -t my_session
