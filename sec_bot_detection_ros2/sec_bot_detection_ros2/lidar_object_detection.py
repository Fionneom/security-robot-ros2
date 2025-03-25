import rclpy
from rclpy.node import Node

import sensor_msgs.msg

import numpy as np

class LidarDetection(Node):
    def __init__(self):
        super().__init__('Lidar_Object_Detection')

        self.lidar_subscription = self.create_subscription(sensor_msgs.msg.LaserScan, 'scan', self.lidar_callback, 10) 

        self.range_history = []
        self.old_average = []
        self.filter_size = 7
        self.cluster_size = 3

    def lidar_callback(self, msg):
        ranges = msg.ranges

        self.range_history.append(ranges)

        if len(self.range_history) > self.filter_size:
            self.range_history.pop(0)

        average = []

        for i in range(len(self.range_history[0])):
            row = []
            for j in range(len(self.range_history)):
                row.append(round(self.range_history[j][i], 1))
            average.append(np.median(row))

        
        if self.old_average != []:
            for i in range(len(average)):
                if(self.old_average[i] - average[i] > 0.2):
                    if all(self.old_average[j] - average[j] > 0.2 for j in range(i - self.cluster_size, i + self.cluster_size)):
                        self.get_logger().info("Object at: " + str(i))
                        i += self.cluster_size
                        

        self.old_average = average


        

        




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