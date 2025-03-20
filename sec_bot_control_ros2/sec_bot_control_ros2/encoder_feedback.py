import lgpio

import rclpy
from rclpy.node import Node

import std_msgs.msg
import std_srvs.srv

import math as m

class EncoderFeedback(Node):

    def __init__(self):
        super().__init__('EncoderFeedback')

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


def main(args=None):
    rclpy.init(args=args)

    encoder_feedback = EncoderFeedback()

    # This code prevents an error when the system is stopped with ctrl - c
    try:
        rclpy.spin(encoder_feedback)
    except KeyboardInterrupt:
        pass
    finally:
        encoder_feedback.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()