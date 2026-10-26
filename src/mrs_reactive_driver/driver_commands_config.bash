echo 'alias start_driver="ros2 service call /reactive_driver/activate std_srvs/srv/SetBool \"{data: true}\""' >> ~/.bashrc
echo 'alias stop_driver="ros2 service call /reactive_driver/activate std_srvs/srv/SetBool \"{data: false}\""' >> ~/.bashrc
source ~/.bashrc