"""\
Launches Gazebo Ignition (ROS2's simulation tool), base robot, and ROS-Gazebo bridge
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
        launch_arguments={
            'gz_args': '-r /home/fionn/sec_bot_ws/src/sec_bot_ros2/urdf/world.sdf'
        }.items(),
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