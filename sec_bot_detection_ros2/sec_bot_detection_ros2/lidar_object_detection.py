import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener

import sensor_msgs.msg
from geometry_msgs.msg import Twist

import numpy as np
import math as m

RIGHT = 0
LEFT = 1

class LidarDetection(Node):
    def __init__(self):
        super().__init__('Lidar_Object_Detection')

        self.lidar_subscription = self.create_subscription(sensor_msgs.msg.LaserScan, 'scan', self.lidar_callback, 10) 

        self.range_history = []
        self.old_average = []
        self.filter_size = 5
        self.cluster_size = 7
        self.point_count = 454

        self.target_angle = 0
        self.current_angle = 0
        self.target_tolerance = 0.2

        self.detection_enabled = True
        self.detection_timer = 0

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lidar_callback(self, msg):
        if self.detection_enabled and self.detection_timer >= 20:
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
                    if(self.old_average[i] - average[i] > 0.15):
                        if all(self.old_average[j] - average[j] > 0.2 for j in range(i - self.cluster_size, i + self.cluster_size)):
                            # i += self.cluster_size
                            
                            if i < self.point_count / 2:
                                lidar_to_angle = (i / self.point_count) * m.pi * 2
                            else:
                                lidar_to_angle = m.pi * (((2 * i - self.point_count) / self.point_count) - 1)

                            self.target_angle = self.current_angle + lidar_to_angle

                            if self.target_angle < - m.pi:
                                self.target_angle += 2 * m.pi
                            elif self.target_angle > m.pi:
                                self.target_angle -= 2 * m.pi

                            self.detection_enabled = False
                            
                            self.get_logger().info("Object at: " + str(lidar_to_angle) + "  Current Angle: " + str(self.current_angle))

            self.old_average = average

    def timer_callback(self):
        self.detection_timer += 1

        try:
            pose = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            quat = pose.transform.rotation
            _, _, angle = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            self.current_angle = angle
        except:
            self.get_logger().info("Waiting on Odom")

        if abs(self.target_angle - self.current_angle) <= self.target_tolerance:
            msg = Twist()
            msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg)

            self.detection_enabled = True

            # self.get_logger().info("Target Found")

        elif self.target_angle < self.current_angle:
            if self.target_angle < self.current_angle - m.pi:
                self.turn(LEFT)
            else:
                self.turn(RIGHT)
            self.detection_timer = 0

        elif self.target_angle > self.current_angle:
            if self.target_angle > self.current_angle + m.pi:
                self.turn(RIGHT)
            else:
                self.turn(LEFT)
            self.detection_timer = 0
            
            


    def turn(self, direction):
        if direction == RIGHT:
            msg = Twist()
            msg.angular.z = -1.2
            self.cmd_vel_publisher.publish(msg)

            self.get_logger().info("Object at: " + str(self.target_angle) + "  Current Angle: " + str(self.current_angle))
        
        else:
            msg = Twist()
            msg.angular.z = 1.2
            self.cmd_vel_publisher.publish(msg)

            self.get_logger().info("Object at: " + str(self.target_angle) + "  Current Angle: " + str(self.current_angle))


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