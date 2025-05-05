"""\
ROS 2 node responsible for controlling the motor driver via GPIO pins
"""
import lgpio
import pigpio

import rclpy
from rclpy.node import Node

import std_msgs.msg
import std_srvs.srv

import math as m

CW = 1
CCW = 0

RIGHT = 0
LEFT = 1

class HardwareInterface(Node):

    def __init__(self):
        super().__init__('HardwareInterface')

        # Motor Driver
        self.PWMA_PIN = 18
        self.AIN2_PIN = 23
        self.AIN1_PIN = 24
        self.BIN1_PIN = 8
        self.BIN2_PIN = 7
        self.PWMB_PIN = 13

        self.h = lgpio.gpiochip_open(0)

        lgpio.gpio_claim_output(self.h, self.AIN1_PIN)
        lgpio.gpio_claim_output(self.h, self.AIN2_PIN)
        lgpio.gpio_claim_output(self.h, self.BIN1_PIN)
        lgpio.gpio_claim_output(self.h, self.BIN2_PIN)

        # lgpio.gpio_claim_output(self.h, self.PWMA_PIN)
        # lgpio.gpio_claim_output(self.h, self.PWMB_PIN)
        self.pi = pigpio.pi()  # Open pigpio connection
        if not self.pi.connected:
            raise Exception("Failed to connect to pigpio daemon")

        self.pi.set_mode(self.PWMA_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.PWMB_PIN, pigpio.OUTPUT)

        # Initialises motor brake
        lgpio.gpio_write(self.h, self.AIN1_PIN, 1)
        lgpio.gpio_write(self.h, self.AIN2_PIN, 0)
        lgpio.gpio_write(self.h, self.BIN1_PIN, 0)
        lgpio.gpio_write(self.h, self.BIN2_PIN, 1)

        self.max_acceleration = 3
        self.max_rpm = 120

        self.right_target_speed_rpm = 0
        self.right_current_speed_rpm = 0
        self.right_target_speed_per = 0
        self.right_current_speed_per = 0
        self.right_current_acceleration = 0
        self.old_speed_left = 0

        self.left_target_speed_rpm = 0
        self.left_current_speed_rpm = 0
        self.left_target_speed_per = 0
        self.left_current_speed_per = 0
        self.left_current_acceleration = 0
        self.old_speed_right = 0

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.motor_speed_subscription = self.create_subscription(std_msgs.msg.Float32MultiArray, 'robot_control/wheel_speeds_set', self.motor_speed_set_callback, 10) 

        self.wheel_speed_publisher = self.create_publisher(std_msgs.msg.Float32MultiArray, 'robot_control/motor_feedback', 10)


    def timer_callback(self):
        # Sets motor speed and direction based on target

        if self.right_target_speed_per > self.right_current_speed_per:
            if self.right_target_speed_per - self.right_current_speed_per > self.right_current_acceleration:
                self.set_direction(CW, RIGHT)
                self.right_current_speed_per += self.right_current_acceleration
            else:
                self.right_current_speed_per = self.right_target_speed_per

        elif self.right_target_speed_per < self.right_current_speed_per:
            if abs(self.right_target_speed_per - self.right_current_speed_per) > self.right_current_acceleration:
                self.set_direction(CCW, RIGHT)
                self.right_current_speed_per -= self.right_current_acceleration
            else:
                self.right_current_speed_per = self.right_target_speed_per

        # Brake motors when stationary
        if self.right_current_speed_per == 0 and self.left_target_speed_per == 0:
            lgpio.gpio_write(self.h, self.AIN1_PIN, 1)
            lgpio.gpio_write(self.h, self.AIN2_PIN, 1)


        if self.left_target_speed_per > self.left_current_speed_per:
            if self.left_target_speed_per - self.left_current_speed_per > self.left_current_acceleration:
                self.set_direction(CW, LEFT)
                self.left_current_speed_per += self.left_current_acceleration
            else:
                self.left_current_speed_per = self.left_target_speed_per

        elif self.left_target_speed_per < self.left_current_speed_per:
            if abs(self.left_target_speed_per - self.left_current_speed_per) > self.left_current_acceleration:
                self.set_direction(CCW, LEFT)
                self.left_current_speed_per -= self.left_current_acceleration
            else:
                self.left_current_speed_per = self.left_target_speed_per

        if self.left_current_speed_per == 0 and self.left_target_speed_per == 0:
            lgpio.gpio_write(self.h, self.BIN1_PIN, 1)
            lgpio.gpio_write(self.h, self.BIN2_PIN, 1)

        # Set PWM value to speed percentage

        if self.right_current_speed_per != self.old_speed_right:
            self.pi.set_PWM_dutycycle(self.PWMA_PIN, round(abs((self.right_current_speed_per/100)*255)))
            self.old_speed_right = self.right_current_speed_per
            self.get_logger().info("PWM" + str(self.right_current_speed_per))
        
        if self.left_current_speed_per != self.old_speed_left:
            self.pi.set_PWM_dutycycle(self.PWMB_PIN, round(abs((self.left_current_speed_per/100)*255)))
            self.old_speed_left = self.left_current_speed_per

        self.publish_wheel_speeds()


    def motor_speed_set_callback(self, msg):
        # Update target speed based on comand velocity

        right_target_speed_rads = msg.data[0]
        left_target_speed_rads = msg.data[1]

        self.left_target_speed_rpm = (left_target_speed_rads / (2 * m.pi)) * 60
        self.right_target_speed_rpm = (right_target_speed_rads / (2 * m.pi)) * 60

        # Speed percentage: Percentage of the max speed

        left_old_speed_per = self.left_current_speed_per
        right_old_speed_per = self.right_current_speed_per

        self.left_target_speed_per = (self.left_target_speed_rpm / self.max_rpm) * 100
        self.right_target_speed_per = (self.right_target_speed_rpm / self.max_rpm) * 100

        # Ensures wheels reach target speed in the same amount of time for predictable movement

        left_difference = abs(self.left_target_speed_per - left_old_speed_per)
        right_difference = abs(self.right_target_speed_per - right_old_speed_per)

        max_difference = max(left_difference, right_difference)

        steps = max_difference / self.max_acceleration

        if(steps > 0):
            self.left_current_acceleration = left_difference / steps
            self.right_current_acceleration = right_difference / steps

        # self.get_logger().info("Left Diff: " + str(left_difference))
        # self.get_logger().info("Right Diff: " + str(right_difference))

        # self.get_logger().info("Steps: " + str(steps))

    
    def set_direction(self, direction, motor):
        # Called every time a motor speed is updated, ensures motors are stationary before reversing direction to reduce back emf spikes

        if motor == RIGHT:
            if abs(self.right_current_speed_per) <= self.max_acceleration:
                self.direction_flag_r = True
                if direction == CW:
                    lgpio.gpio_write(self.h, self.AIN1_PIN, 1)
                    lgpio.gpio_write(self.h, self.AIN2_PIN, 0)
                    # self.get_logger().info("Right Motor CW")
                elif direction == CCW:
                    lgpio.gpio_write(self.h, self.AIN1_PIN, 0)
                    lgpio.gpio_write(self.h, self.AIN2_PIN, 1)
                    # self.get_logger().info("Right Motor CCW")

        elif motor == LEFT:
            if abs(self.left_current_speed_per) <= self.max_acceleration:
                self.direction_flag_l = True
                if direction == CW:
                    lgpio.gpio_write(self.h, self.BIN1_PIN, 1)
                    lgpio.gpio_write(self.h, self.BIN2_PIN, 0)
                    # self.get_logger().info("Left Motor CW")
                elif direction == CCW:
                    lgpio.gpio_write(self.h, self.BIN1_PIN, 0)
                    lgpio.gpio_write(self.h, self.BIN2_PIN, 1)
                    # self.get_logger().info("Left Motor CCW")


    def publish_wheel_speeds(self):
        # Updates controller with wheel speed. Should use encoders but currently sets to current predicted speed

        self.right_current_speed_rpm = (self.right_current_speed_per / 100) * self.max_rpm
        self.left_current_speed_rpm = (self.left_current_speed_per / 100) * self.max_rpm

        right_current_speed_rads = (self.right_current_speed_rpm / 60) * 2 * m.pi
        left_current_speed_rads = (self.left_current_speed_rpm / 60) * 2 * m.pi

        msg_out = std_msgs.msg.Float32MultiArray()
        msg_out.data = [right_current_speed_rads, left_current_speed_rads]

        self.wheel_speed_publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)

    hardware_interface = HardwareInterface()

    # This code prevents an error when the system is stopped with ctrl - c
    try:
        rclpy.spin(hardware_interface)
    except KeyboardInterrupt:
        pass
    finally:
        hardware_interface.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
