import rclpy
from rclpy.node import Node

import sensor_msgs.msg

import csv
from datetime import datetime

class LidarTest(Node):
    def __init__(self):
        super().__init__('Lidar_Test')

        self.lidar_subscription = self.create_subscription(sensor_msgs.msg.LaserScan, 'scan', self.lidar_callback, 10) 
        
        self.sample_count = 0
        self.sample_count_target = 100

        self.sample_point = 0 # Straight Ahead
        self.distances = []

        self.get_logger().info("Lidar Test Beginning...")


    def lidar_callback(self, msg):
        distance = msg.ranges[self.sample_point]
        self.distances.append(distance)
        self.sample_count += 1

        if self.sample_count >= self.sample_count_target:
            self.publish()

    def publish(self):
        with open('lidar_test_' + str(datetime.now()) +'.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Lidar Test"])
            for val in self.distances:
                writer.writerow([val])
                

        file.close()
        self.destroy_node()
        raise KeyboardInterrupt




def main(args=None):
    rclpy.init(args=args)

    lidar_test = LidarTest()

    try:
        rclpy.spin(lidar_test)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_test.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()