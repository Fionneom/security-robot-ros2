"""\
ROS 2 node which detects changes in range based on the lidar and turn the robot to the detected movement
"""

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

        # History stores previous points to filter out noise
        self.range_history = []
        self.old_average = []
        self.filter_size = 8
        self.cluster_size = 2
        self.point_count = 454

        self.target_angle = 0
        self.current_angle = 0
        # Robot only needs to turn to within 0.2 radians of the target to prevent overshoot
        self.target_tolerance = 0.2

        # Detection is disabled for a number of seconds after reaching target to allow scans to settle
        self.detection_enabled = True
        self.detection_timer = 0
        self.detection_timer_threshold = 30

        # The minimum distance a range must change to trigger the system is 0.25m, the max distance the object can be from the lidar is 3m to reduce distant errors 
        self.range_change_threshold = 0.25
        self.max_distance = 3

        self.rotation_speed = 1.2

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Used to access the current angle of the robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lidar_callback(self, msg):
        # Only detect objects once lidar has settled
        if self.detection_enabled and self.detection_timer >= self.detection_timer_threshold:
            ranges = msg.ranges

            self.range_history.append(ranges)

            # Remove oldest range data
            if len(self.range_history) > self.filter_size:
                self.range_history.pop(0)

            average = []

            # Calculate the median value of each point to filter out noise
            for i in range(len(self.range_history[0])):
                row = []
                for j in range(len(self.range_history)):
                    row.append(round(self.range_history[j][i], 1))
                average.append(np.median(row))

            
            if self.old_average != []:

                # Loops through all 454 Lidar points
                for i in range(len(average)):

                    # Checks if any point has gotten changed by a threshold value eg. 15cm
                    if(self.old_average[i] - average[i] > self.range_change_threshold) and self.detection_enabled and average[i] <= self.max_distance:

                        # Checks a cluster of points around the point to ensure it is not a single point error
                        if all(self.old_average[j] - average[j] > self.range_change_threshold for j in range(i - self.cluster_size, i + self.cluster_size)):
                            
                            self.get_logger().info("old_average " + str(self.old_average[i]) + "  average: " + str(average[i]))

                            # Converts lidar point value to randian value between -pi and pi (Used for robot angle)
                            if i < self.point_count / 2:
                                lidar_to_angle = (i / self.point_count) * m.pi * 2
                            else:
                                lidar_to_angle = m.pi * (((2 * i - self.point_count) / self.point_count) - 1)

                            self.target_angle = self.current_angle + lidar_to_angle

                            # Keeps target within range -pi to pi
                            if self.target_angle < - m.pi:
                                self.target_angle += 2 * m.pi
                            elif self.target_angle > m.pi:
                                self.target_angle -= 2 * m.pi

                            # Disables detection of new points until target reached
                            self.detection_enabled = False
                            
                            # self.get_logger().info("Object at: " + str(lidar_to_angle) + "  Current Angle: " + str(self.current_angle))

            self.old_average = average

    def timer_callback(self):
        self.detection_timer += 1

        # Get robot angle and convert quaternions to radians
        try:
            pose = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            quat = pose.transform.rotation
            _, _, angle = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            self.current_angle = angle
        except:
            self.get_logger().info("Waiting on Odom")

        # If current angle is within tolerance of target stop moving and enable detection
        if abs(self.target_angle - self.current_angle) <= self.target_tolerance or abs(self.target_angle - self.current_angle) > 2 * m.pi - self.target_tolerance:
            msg = Twist()
            msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg)

            self.range_history = []

            self.detection_enabled = True

        # Turn towards target while minimising rotation distance
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
            
            

    # Write command velocity
    def turn(self, direction):
        msg = Twist()

        if direction == RIGHT:
            msg.angular.z = -self.rotation_speed
        else:
            msg.angular.z = self.rotation_speed
        
        self.cmd_vel_publisher.publish(msg)

        # self.get_logger().info("Object at: " + str(self.target_angle) + "  Current Angle: " + str(self.current_angle))


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