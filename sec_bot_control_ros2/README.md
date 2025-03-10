# ROS 2 Car Project - Control
This package contains all the custom nodes created for the Car Project. \
 A lot of these nodes could be replaced by implementing the [ros2_control](https://control.ros.org/rolling/index.html) package. This package provides a framework for realtime robot control. Custom nodes were chosen for ease of development, however, ros2_control would provide a much more flexible and robust system for motor control.

## Todo
- Publish Odom topic for fusion with IMU
- Use sim_time when running in simulation to keep things in sync
- Tune PIDs for simulated motor control (especially steering)

- Simulation is not particularly accurate in general, wheels don't always turn when changing drive direction. Should probably look into this

- Joint state publisher stopped working for some reason. Wheel angle publisher is being used as a work around until we can get joint feedback from the Pico