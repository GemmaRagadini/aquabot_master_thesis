# Aquabot – ROS 2 Workspace

ROS 2 workspace for the Aquabot Master Thesis.

## Packages
- aquabot_bringup
- arduino_reader
- dynamixel_controller
- master

## Build
- colcon build
- source install/setup.bash 

## Launch
ros2 launch aquabot_bringup system_launch.py

# write csv 
ros2 service call /trial std_srvs/srv/SetBool "{data: true}"

# topic  
ros2 topic echo /target_position
ros2 topic echo /sensor_reading