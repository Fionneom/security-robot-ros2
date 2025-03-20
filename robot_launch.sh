#!/bin/bash

cd sec_bot_ws
colcon build
source install/setup.bash
ros2 launch sec_bot_ros2 robot_bringup.launch.py