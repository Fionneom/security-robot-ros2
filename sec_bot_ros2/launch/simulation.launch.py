"""\
Launches Gazebo Ignition (ROS2's simultion tool), base car, and ROS-Gazebo bridge
It would work really well on a good computer, but struggles to run on the jetson which is a pity.
There are also a lot of different versions of gazebo. This one is specially for ros humble and has very poor documentation.
It would be really cool if you could get it working properly, but its also a pain.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = 'true'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        # launch_arguments={
        #     'gz_args': '-r /carProject_ws/src/sec_bot_ros2/description/onshapeTest/world.sdf'
        # }.items(),
    )

    base = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('sec_bot_ros2'),'launch','base.launch.py'
                )]),
                launch_arguments = {'use_sim_time': use_sim_time}.items()
    )

    # Gazebo Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(get_package_share_directory('sec_bot_ros2'), 'config', 'simulation_bridge.yaml'),
    #         'qos_overrides./tf_static.publisher.durability': 'transient_local'
    #     }],
    #     output='screen'
    # )


    return LaunchDescription([
        gz_sim,
        base,
        # bridge,
    ])