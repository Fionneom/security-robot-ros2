"""\
Launch basic selection of nodes used in most other launch files
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default = False)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('sec_bot_ros2'),'launch','rsp.launch.py'
                )]),
                launch_arguments = {'use_sim_time': use_sim_time}.items()
    )

    # control = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('sec_bot_ros2_control'),'launch','control.launch.py'
    #             )]),
    #             launch_arguments = {'use_sim_time': use_sim_time}.items()
    # )

    # foxglove = IncludeLaunchDescription(
    #             XMLLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('foxglove_bridge'),'launch','foxglove_bridge_launch.xml'
    #             )]),
    #             launch_arguments = {'use_sim_time': use_sim_time}.items()
    # )


    return LaunchDescription([
        rsp,
        # control,
        # foxglove,
    ])