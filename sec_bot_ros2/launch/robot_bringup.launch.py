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

    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('sec_bot_ros2'),'launch','lidar.launch.py'
                )]),
                launch_arguments = {'use_sim_time': use_sim_time}.items()
    )

    camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('sec_bot_detection_ros2'),'launch','camera.launch.py'
                )]),
                launch_arguments = {'use_sim_time': use_sim_time}.items()
    )

    detection = Node(
        package='sec_bot_detection_ros2',
        namespace='',
        executable='human_detection',
        name='human_detection',
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory('sec_bot_ros2'), 'rviz', 'rviz_config.rviz')]]
    )

    face = Node(
        package='articubot_one_ui',
        namespace='',
        executable='play_face',
        name='play_face',
    )

    control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('sec_bot_control_ros2'),'launch','control.launch.py'
                )]),
                launch_arguments = {'use_sim_time': use_sim_time}.items()
    )


    return LaunchDescription([
        rsp,
        # rviz,
        camera,
        lidar,
        # detection,
        control,  
        # foxglove,
        face
    ])