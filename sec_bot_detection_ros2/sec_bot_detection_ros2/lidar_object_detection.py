import rclpy
from rclpy.node import Node

import sensor_msgs.msg

class LidarDetection(Node):
    def __init__(self):
        super().__init__('Lidar_Object_Detection')

        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.lidar_subscription = self.create_subscription(sensor_msgs.msg.LaserScan, 'scan', self.lidar_callback, 10) 

        self.old_ranges = []
        self.filter_size = 5
    # def timer_callback(self):
    #     pass

    def lidar_callback(self, msg):
        ranges = msg.ranges

        self.old_ranges.append(ranges)

        if len(self.old_ranges) > self.filter_size:
            self.old_ranges.pop(0)

        




def main(args=None):
    rclpy.init(args=args)

    lidar_detection = LidarDetection()

    try:
        rclpy.spin(lidar_detection)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_detection.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()