"""\
ROS 2 node responsible for reading and writing to the Pico.
Uses the MainController python class (see submodules folder)
Also subscribes to lidar and radar safety nodes to stop when and object is detected
"""
import lgpio

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

        # ME = Motor Encoder
        self.ME1A_PIN = 21
        self.ME1B_PIN = 20
        self.ME2A_PIN = 16
        self.ME2B_PIN = 25

        # Motor Driver
        self.PWMA_PIN = 18
        self.AIN2_PIN = 23
        self.AIN1_PIN = 24
        self.BIN1_PIN = 8
        self.BIN2_PIN = 7
        self.PWMB_PIN = 13

        self.h = lgpio.gpiochip_open(0)

        lgpio.gpio_claim_input(self.h, self.ME1A_PIN)
        lgpio.gpio_claim_input(self.h, self.ME1B_PIN)
        lgpio.gpio_claim_input(self.h, self.ME2A_PIN)
        lgpio.gpio_claim_input(self.h, self.ME2B_PIN)

        lgpio.gpio_claim_output(self.h, self.AIN1_PIN)
        lgpio.gpio_claim_output(self.h, self.AIN2_PIN)
        lgpio.gpio_claim_output(self.h, self.BIN1_PIN)
        lgpio.gpio_claim_output(self.h, self.BIN2_PIN)

        lgpio.gpio_claim_output(self.h, self.PWMA_PIN)
        lgpio.gpio_claim_output(self.h, self.PWMB_PIN)

        lgpio.gpio_write(self.h, self.AIN1_PIN, 1)
        lgpio.gpio_write(self.h, self.AIN2_PIN, 1)
        lgpio.gpio_write(self.h, self.BIN1_PIN, 1)
        lgpio.gpio_write(self.h, self.BIN2_PIN, 1)

        self.acceleration = 2
        self.max_rpm = 150

        self.right_target_speed_rpm = 0
        self.right_current_speed_rpm = 0
        self.right_target_speed_per = 0
        self.right_current_speed_per = 0

        self.left_target_speed_rpm = 0
        self.left_current_speed_rpm = 0
        self.left_target_speed_per = 0
        self.left_current_speed_per = 0

        self.direction_flag_r = False
        self.direction_flag_l = False

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.motor_speed_subscription = self.create_subscription(std_msgs.msg.Float32MultiArray, 'robot_control/wheel_speeds_set', self.motor_speed_set_callback, 10) 

        self.wheel_speed_publisher = self.create_publisher(std_msgs.msg.Float32MultiArray, 'robot_control/motor_feedback', 10)


    def timer_callback(self):
        m1a = lgpio.gpio_read(self.h, self.ME1A_PIN)
        m1b = lgpio.gpio_read(self.h, self.ME1B_PIN)
        m2a = lgpio.gpio_read(self.h, self.ME2A_PIN)
        m2b = lgpio.gpio_read(self.h, self.ME2B_PIN)
        
        lgpio.tx_pwm(self.h, self.PWMA_PIN, 1000, abs(self.right_current_speed_per))
        lgpio.tx_pwm(self.h, self.PWMB_PIN, 1000, abs(self.left_current_speed_per))

        if self.right_target_speed_per > self.right_current_speed_per:
            if self.right_target_speed_per - self.right_current_speed_per > self.acceleration:
                self.set_direction(CW, RIGHT)
                self.right_current_speed_per += self.acceleration
            else:
                self.right_current_speed_per = self.right_target_speed_per

        elif self.right_target_speed_per < self.right_current_speed_per:
            if abs(self.right_target_speed_per - self.right_current_speed_per) > self.acceleration:
                self.set_direction(CCW, RIGHT)
                self.right_current_speed_per -= self.acceleration
            else:
                self.right_current_speed_per = self.right_target_speed_per

        if self.right_current_speed_per == 0 and self.left_target_speed_per == 0:
                lgpio.gpio_write(self.h, self.AIN1_PIN, 1)
                lgpio.gpio_write(self.h, self.AIN2_PIN, 1)
                if self.direction_flag_r:
                    self.get_logger().info("Right Motor Brake")
                    self.direction_flag_r = False

        if self.left_target_speed_per > self.left_current_speed_per:
            if self.left_target_speed_per - self.left_current_speed_per > self.acceleration:
                self.set_direction(CW, LEFT)
                self.left_current_speed_per += self.acceleration
            else:
                self.left_current_speed_per = self.left_target_speed_per

        elif self.left_target_speed_per < self.left_current_speed_per:
            if abs(self.left_target_speed_per - self.left_current_speed_per) > self.acceleration:
                self.set_direction(CCW, LEFT)
                self.left_current_speed_per -= self.acceleration
            else:
                self.left_current_speed_per = self.left_target_speed_per

        if self.left_current_speed_per == 0 and self.left_target_speed_per == 0:
                lgpio.gpio_write(self.h, self.BIN1_PIN, 1)
                lgpio.gpio_write(self.h, self.BIN2_PIN, 1)
                if self.direction_flag_l:
                    self.get_logger().info("Left Motor Brake")
                    self.direction_flag_l = False

        self.publish_wheel_speeds()

        # self.get_logger().info(str(m1a) + " " +str(m1b) + " " +str(m2a) + " " +str(m2b) + " ")
        # self.get_logger().info("Current Speed: " + str(self.right_current_speed_per) + " Target Speed: " + str(self.right_target_speed_per))


    def motor_speed_set_callback(self, msg):
        right_target_speed_rads = msg.data[0]
        left_target_speed_rads = msg.data[1]

        self.left_target_speed_rpm = (left_target_speed_rads / (2 * m.pi)) * 60
        self.right_target_speed_rpm = (right_target_speed_rads / (2 * m.pi)) * 60

        self.left_target_speed_per = (self.left_target_speed_rpm / self.max_rpm) * 100
        self.right_target_speed_per = (self.right_target_speed_rpm / self.max_rpm) * 100

        
    def set_direction(self, direction, motor):
        if motor == RIGHT:
            if self.right_current_speed_per < self.acceleration and self.right_current_speed_per > -self.acceleration:
                self.direction_flag_r = True
                if direction == CW:
                    lgpio.gpio_write(self.h, self.AIN1_PIN, 1)
                    lgpio.gpio_write(self.h, self.AIN2_PIN, 0)
                    self.get_logger().info("Right Motor CW")
                elif direction == CCW:
                    lgpio.gpio_write(self.h, self.AIN1_PIN, 0)
                    lgpio.gpio_write(self.h, self.AIN2_PIN, 1)
                    self.get_logger().info("Right Motor CCW")

        elif motor == LEFT:
            if self.left_current_speed_per < self.acceleration and self.left_current_speed_per > -self.acceleration:
                self.direction_flag_l = True
                if direction == CW:
                    lgpio.gpio_write(self.h, self.BIN1_PIN, 1)
                    lgpio.gpio_write(self.h, self.BIN2_PIN, 0)
                    self.get_logger().info("Left Motor CW")
                elif direction == CCW:
                    lgpio.gpio_write(self.h, self.BIN1_PIN, 0)
                    lgpio.gpio_write(self.h, self.BIN2_PIN, 1)
                    self.get_logger().info("Left Motor CCW")


    def publish_wheel_speeds(self):
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
        while hardware_interface.motorspeed > 0:
            hardware_interface.motorspeed -= 0.01
    finally:
        hardware_interface.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
