#!/bin/bash

if [ -z "$IN_TERMINAL" ]; then
    export IN_TERMINAL=1
    gnome-terminal -- bash -c "$(realpath "$0"); exec bash"
    exit
fi

cd /home/fionn/sec_bot_ws || exit
colcon build
source install/setup.bash

export DISPLAY=:0
export XAUTHORITY=/home/yourusername/.Xauthority

sleep 5

ros2 launch sec_bot_ros2 robot_bringup.launch.py