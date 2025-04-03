"""\
ROS 2 node to update the robot's position in the world based on wheel speed.
"""

import rclpy
from rclpy.node import Node

import sensor_msgs.msg
import geometry_msgs.msg

from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

import math as m
import numpy as np


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        
        self.transform_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        self.joint_state_subscription = self.create_subscription(sensor_msgs.msg.JointState, 'joint_states', self.joint_callback, 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Position of the robot in x and y and angle (radians) relative to the world origin
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # Linear velocity in x and y (m/s) and rotational velocity (rad/s)
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0

        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0

        # Distance between wheels
        self.wheel_track = 0.2
        self.wheel_diameter = 0.13

        self.t1 = self.get_clock().now().nanoseconds * 1e-9

    # Updates the robot's position based on the speed of each wheel every 0.1 seconds
    def timer_callback(self):
        self.y_vel = ((self.wheel_diameter * m.cos(self.odom_theta) / 4) * (self.right_wheel_speed + self.left_wheel_speed))
        self.x_vel = ((self.wheel_diameter * m.sin(self.odom_theta) / 4) * (self.right_wheel_speed + self.left_wheel_speed))

        self.theta_vel = (self.wheel_diameter / (2 * self.wheel_track)) * (self.left_wheel_speed - self.right_wheel_speed)

        t2 = self.get_clock().now().nanoseconds * 1e-9

        self.odom_x += self.x_vel * (t2 - self.t1)
        self.odom_y += self.y_vel * (t2 - self.t1)
        self.odom_theta += self.theta_vel * (t2 - self.t1)

        self.t1 = t2

        # Publishes a transform, position and rotation of the robot relative to the origin
        self.publish_transform()
        
        # Can also publish a specific odom message for use when merging odometry with the IMU (not necesarry right now)
        # self.publish_odom()
        
    # Updates wheel speed based on the joint_states topic
    def joint_callback(self, msg):
        # Prevents the code from crashing if wheel speeds haven't been received yet
        try:
            self.right_wheel_speed = msg.velocity[msg.name.index("right_wheel_joint")]
            self.left_wheel_speed = msg.velocity[msg.name.index("left_wheel_joint")]
        except:
            self.get_logger().info('Joint data not available')
        
    # Formats information for a transform message
    def publish_transform(self):
        odom_transform = geometry_msgs.msg.TransformStamped()

        odom_transform.header.stamp = self.get_clock().now().to_msg()
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id = 'base_link'

        odom_transform.transform.translation.x = float(self.odom_x)
        odom_transform.transform.translation.y = float(self.odom_y)

        # Converts roll pitch and yaw into quaternions
        q = quaternion_from_euler(0, 0, -self.odom_theta)
        odom_transform.transform.rotation.z = q[2]
        odom_transform.transform.rotation.w = q[3]

        self.transform_broadcaster.sendTransform(odom_transform)

    # Not currently used
    def publish_odom(self):
        odom_message = Odometry()

        odom_message.header.stamp = self.get_clock().now().to_msg()

        odom_message.pose.pose.position.x = self.odom_x
        odom_message.pose.pose.position.y = self.odom_y
        odom_message.pose.pose.position.z = 0.0
        odom_message.pose.pose.orientation.x = 0.0
        odom_message.pose.pose.orientation.y = 0.0

        q = quaternion_from_euler(0, 0, -self.odom_theta)
        odom_message.pose.pose.orientation.z = q[2]
        odom_message.pose.pose.orientation.w = q[3]

        odom_message.twist.twist.linear.x = self.x_vel
        odom_message.twist.twist.linear.y = self.y_vel

        odom_message.twist.twist.angular.z = -self.theta_vel

        self.odom_publisher.publish(odom_message)

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = m.cos(ai)
    si = m.sin(ai)
    cj = m.cos(aj)
    sj = m.sin(aj)
    ck = m.cos(ak)
    sk = m.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    # Prevents errors when shutting down
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()