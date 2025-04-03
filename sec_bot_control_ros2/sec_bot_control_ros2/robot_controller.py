"""\
ROS 2 node responsible for passing messages to and from the hardware interface/simulation.
Takes in Velocity values, calculates wheel speeds
"""

import rclpy
from rclpy.node import Node

import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

import numpy as np

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        self.command_subscription = self.create_subscription(geometry_msgs.msg.Twist, 'cmd_vel', self.command_callback, 10)
        
        if self.simulation_mode:
            timer_period = 0.05  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)

        else:
            self.motor_write_publisher = self.create_publisher(std_msgs.msg.Float32MultiArray, 'robot_control/wheel_speeds_set', 10)
            self.motor_feedback_subscriber = self.create_subscription(std_msgs.msg.Float32MultiArray, 'robot_control/motor_feedback', self.motor_feedback_callback, 10)

        self.joint_publisher = self.create_publisher(sensor_msgs.msg.JointState, 'robot_control/joint_states', 10)

        self.wheel_diameter = 0.13
        self.wheel_track = 0.2

        self.right_wheel_position = 0
        self.left_wheel_position = 0

        self.right_wheel_speed = 0
        self.left_wheel_speed = 0

        self.t1 = self.get_clock().now().nanoseconds * 1e-9

    def timer_callback(self):
        # Only used when real hardware not available. Updates wheel speed based on current target
        msg_out = std_msgs.msg.Float32MultiArray()
        msg_out.data = [float(self.right_wheel_speed), float(self.left_wheel_speed)]

        self.motor_feedback_callback(msg_out)


    def command_callback(self, msg):
        # Updates target wheel speed based on command input
        linear_velocity = msg.linear
        angular_velocity = msg.angular

        wheel_speeds = self.twist_to_wheel_velocity(linear_velocity.x, angular_velocity.z)

        self.right_wheel_speed = wheel_speeds[0]
        self.left_wheel_speed = wheel_speeds[1]

        if not self.simulation_mode:
            msg_out = std_msgs.msg.Float32MultiArray()
            msg_out.data = [self.right_wheel_speed, self.left_wheel_speed]

            self.motor_write_publisher.publish(msg_out)


    def motor_feedback_callback(self, msg):
        self.right_wheel_speed = msg.data[0]
        self.left_wheel_speed = msg.data[1]

        t2 = self.get_clock().now().nanoseconds * 1e-9
        self.right_wheel_position += self.right_wheel_speed * (t2 - self.t1)
        self.left_wheel_position += self.left_wheel_speed * (t2 - self.t1)
        self.t1 = t2

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["right_wheel_joint", "left_wheel_joint"]
        msg.position = [float(self.right_wheel_position), float(self.left_wheel_position)]
        
        msg.velocity = [float(self.right_wheel_speed), float(self.left_wheel_speed)]

        self.joint_publisher.publish(msg)
     
        
    def twist_to_wheel_velocity(self, linear_velocity, angular_velocity):
        coefficients = np.array([[1, 1], [1, -1]])
        constants = np.array([(4 * linear_velocity) / self.wheel_diameter, (2 * self.wheel_track * angular_velocity) / self.wheel_diameter])

        wheel_speeds = np.linalg.solve(coefficients, constants)

        return(wheel_speeds)



def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()