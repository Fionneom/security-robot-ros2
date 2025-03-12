import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    simulation_mode = LaunchConfiguration('use_sim_time', default = True)
    
  
    # Create a robot_state_publisher node
    params = {"source_list": ["simulation_feedback/joint_states", "robot_control/joint_states"]}
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_odom_publisher = Node(
        package='sec_bot_control_ros2',
        executable='odom_publisher',
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    node_robot_controller = Node(
        package='sec_bot_control_ros2',
        executable='robot_controller',
        output='screen',
        parameters=[{'simulation_mode': simulation_mode}]
    )

    # Launch!
    return LaunchDescription([
        node_joint_state_publisher,
        node_odom_publisher,
        node_robot_controller,
    ])