# Security Robot ROS2 Project
Fionn O Muiri

scp -r sec_bot_control_ros2 fionn@192.168.211.55:~/sec_bot_ws/src

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/sec_bot_ros2/config/mapper_params_online_async.yaml

sudo apt install ros-humble-slam-toolbox