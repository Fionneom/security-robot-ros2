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

class HardwareInterface(Node):

    def __init__(self):
        super().__init__('HardwareInterface')

        # ME = Motor Encoder
        self.ME1A_PIN = 21
        self.ME1B_PIN = 20
        self.ME2A_PIN = 16
        self.ME2B_PIN = 25

        self.h = lgpio.gpiochip_open(0)

        lgpio.gpio_claim_input(self.h, self.ME1A_PIN)
        lgpio.gpio_claim_input(self.h, self.ME1B_PIN)
        lgpio.gpio_claim_input(self.h, self.ME2A_PIN)
        lgpio.gpio_claim_input(self.h, self.ME2B_PIN)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        


    # As mentioned above, continuosly write wheel speed and angle to the pico amd reads the actual speed and steering angle
    def timer_callback(self):
        m1a = lgpio.gpio_read(self.h, self.ME1A_PIN)
        m1b = lgpio.gpio_read(self.h, self.ME1B_PIN)
        m2a = lgpio.gpio_read(self.h, self.ME2A_PIN)
        m2b = lgpio.gpio_read(self.h, self.ME2B_PIN)

        self.get_logger().info(str(m1a + " " +m1b + " " +m2a + " " +m2b + " "))



def main(args=None):
    rclpy.init(args=args)

    hardware_interface = HardwareInterface()

    # This code prevents an error when th system is stopped with ctrl - c
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
