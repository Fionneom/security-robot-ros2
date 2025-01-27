import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default = False)
  
    # Create a robot_state_publisher node
    params = {"source_list": ["simulation_feedback/joint_states", "car_control/joint_states"], 'use_sim_time': use_sim_time}
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_odom_publisher = Node(
        package='ros2_car_project_control',
        executable='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    node_car_controller = Node(
        package='ros2_car_project_control',
        executable='car_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    node_twist_to_ackermann = Node(
        package='twist_to_ackermann',
        executable='twist_to_ackermann',
        output='screen',
        parameters=[{"wheelbase": 0.76}],
        remappings=[
            ('/nav_vel', '/cmd_vel'),
        ]
    )


    # Launch!
    return LaunchDescription([
        node_joint_state_publisher,
        node_odom_publisher,
        node_twist_to_ackermann,
        node_car_controller,
    ])