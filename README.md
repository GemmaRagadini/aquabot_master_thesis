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

Per vedere i valori dei giunti : ros2 topic echo /joint_states
Per pubblicare un sensor reading finto: ros2 topic pub /sensor_reading std_msgs/msg/Float32MultiArray "{data: [0.8]}"
Per vedere target posizione dynamixel:ros2 topic echo /aquabot/dynamixel/target_position