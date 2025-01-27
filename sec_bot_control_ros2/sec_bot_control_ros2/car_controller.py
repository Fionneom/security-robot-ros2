"""\
ROS 2 node responsible for passing messages to and from the hardware interface/simulation.
Takes in Velocity values, calculates wheel speeds and forwards to the pico.
Also, when not in simulation mode, takes in wheel speeds and angles from pico and updates the kinematic model (joint states)
"""

import rclpy
from rclpy.node import Node

import std_msgs.msg
import sensor_msgs.msg
import ackermann_msgs.msg

class CarController(Node):

    def __init__(self):
        super().__init__('car_controller')

        # We need simulation mode 1) to make sure calculations use simulation time when necesarry and 2) because in simulation mode the joint states are update automatically, but not when using the picos
        self.simulation_mode = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.command_subscription = self.create_subscription(ackermann_msgs.msg.AckermannDrive, 'ack_vel', self.command_callback, 10)
        
        if self.simulation_mode:
            self.simulation_steering_angle_publisher = self.create_publisher(std_msgs.msg.Float64, 'car_control/steering_angle', 10)
            self.simulation_wheel_speed_publisher = self.create_publisher(std_msgs.msg.Float64, 'car_control/wheel_speed', 10)
        else:
            self.pico_writes_publisher = self.create_publisher(std_msgs.msg.Float32MultiArray, 'car_control/pico_write', 10)
            self.pico_read_subscription = self.create_subscription(std_msgs.msg.Float32MultiArray, 'car_control/pico_read', self.pico_read_callback, 10)

            self.joint_publisher = self.create_publisher(sensor_msgs.msg.JointState, 'car_control/joint_states', 10)

        self.wheel_diameter = 0.305

        self.wheel_position = 0
        self.t1 = self.get_clock().now().nanoseconds * 1e-9

    # Callback called when a new speed is set (message received on ack_vel topic), calculates wheel speed and steering angle, and sends these to pico or to Gazebo simulation 
    def command_callback(self, msg):
        velocity = msg.speed
        wheel_speed = (2 * velocity) / self.wheel_diameter
        steering_angle = msg.steering_angle

        if self.simulation_mode:
            msg_out = std_msgs.msg.Float64()

            msg_out.data = wheel_speed
            self.simulation_wheel_speed_publisher.publish(msg_out)

            msg_out.data = steering_angle
            self.simulation_steering_angle_publisher.publish(msg_out)
        else:
            msg_out = std_msgs.msg.Float32MultiArray()
            msg_out.data = [wheel_speed, steering_angle]

            self.pico_writes_publisher.publish(msg_out)

    # Callback called when the hardware interface publishes the current speed and angle of the car.
    # Uses wheel speed to update wheel position and updates joint states (used to update cars position)
    def pico_read_callback(self, msg):
        wheel_speed = msg.data[0]
        steering_angle = msg.data[1]

        t2 = self.get_clock().now().nanoseconds * 1e-9
        self.wheel_position += wheel_speed * (t2 - self.t1)
        self.t1 = t2

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["left_steering_joint", "right_steering_joint", "back_right_wheel_joint", "back_left_wheel_joint", "front_right_wheel_joint", "front_left_wheel_joint"]
        msg.position = [float(steering_angle), float(steering_angle), float(self.wheel_position), float(-self.wheel_position), float(self.wheel_position), float(-self.wheel_position)]
        
        msg.velocity = [0.0, 0.0, float(wheel_speed), float(wheel_speed), float(wheel_speed), float(wheel_speed)]

        self.joint_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)

    car_controller = CarController()

    try:
        rclpy.spin(car_controller)
    except KeyboardInterrupt:
        pass
    finally:
        car_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()