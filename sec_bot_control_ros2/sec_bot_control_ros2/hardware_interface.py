"""\
ROS 2 node responsible for reading and writing to the Pico.
Uses the MainController python class (see submodules folder)
Also subscribes to lidar and radar safety nodes to stop when and object is detected
"""


import rclpy
from rclpy.node import Node

import std_msgs.msg
import std_srvs.srv

# import CarControllers.MainControllers as PicoController
from .submodules.MainController import Main_Controller

class HardwareInterface(Node):

    def __init__(self):
        super().__init__('HardwareInterface')

        # Individual controller for both speed and steering picos
        self.speed_controller = Main_Controller('/dev/serial/by-id/usb-Raspberry_Pi_Picoprobe__CMSIS-DAP__E6614103E74C5C36-if01')
        self.steering_controller = Main_Controller('/dev/serial/by-id/usb-Raspberry_Pi_Picoprobe__CMSIS-DAP__E6609CB2D32B5728-if01')

        # Calibration service allows car to be calibrated on command
        self.calibration_service = self.create_service(std_srvs.srv.Trigger, 'calibrate_steering', self.calibration_service_callback)

        # Write PID values for speed control
        self.speed_controller.Write_KP(1200)
        self.speed_controller.Write_KI(1000) #112
        self.speed_controller.Write_KD(100) #15

        # Trigger steering calibration
        self.steering_controller.Write_Calibration(0)

        self.pico_writes_subscriber = self.create_subscription(std_msgs.msg.Float32MultiArray, 'car_control/pico_write', self.pico_write_callback, 10)
        
        self.lidar_blocking_subscriber = self.create_subscription(std_msgs.msg.Int32MultiArray, 'lidar_blocking', self.lidar_blocking_callback, 10)
        self.radar_blocking_subscriber = self.create_subscription(std_msgs.msg.Int32MultiArray, 'radar_blocking', self.radar_blocking_callback, 10)

        self.pico_read_publisher = self.create_publisher(std_msgs.msg.Float32MultiArray, 'car_control/pico_read', 10)

        # Turbo mode is redundant now and could probably be removed but I don't want to break anything
        self.speed_service = self.create_service(std_srvs.srv.SetBool, 'turbo_mode', self.turbo_mode_callback)

        # Timer callback used to continuously update speed and steering angle and read back results every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.target_wheel_speed_counts = 0
        # There are 100 segments on the back wheel
        self.counts_per_revolution = 100

        self.lidar_forward_blocking = False
        self.lidar_reverse_blocking = False
        self.radar_forward_blocking = False

        # Might need to update counts to 15, but again, I don't want to break anything. This might slightly improve car position estimate
        self.steering_counts = 14
        # The car has a maximum steering angle of approx 0.456 radians to the right and left
        self.total_steering_angle_rad = 0.912
        self.target_steering_angle_counts = 0


    # Callback updates wheel speed and angle when a message is received on the pico write topic
    def pico_write_callback(self, msg):
        wheel_speed_rad = msg.data[0]
        # Set to negative to correct the wheels turning the wrong way. Would be better to update in the URDF file but it works for now
        target_steering_angle_rad = -msg.data[1]

        # Wheel speed from rad to count per 0.25 seconds (Blame James and Frank for weird unit lol)
        wheel_speed_counts = int((25 / (2 * 3.1415926535)) * wheel_speed_rad)

        # Converts steering angle in radians to counts based on the maximum steering angle and total number of counts
        self.target_steering_angle_counts = int(((target_steering_angle_rad / self.total_steering_angle_rad) + 0.5) * self.steering_counts)

        # Checking value close to zero should prevent rounding errors
        if wheel_speed_counts > -0.01 and wheel_speed_counts < 0.01:
            self.target_wheel_speed_counts = 0
            self.speed_controller.EMERGENCY_STOP()
        else:
            self.target_wheel_speed_counts = wheel_speed_counts

    # Ros service to stop car and trigger calibration
    def calibration_service_callback(self, request, response):
        self.get_logger().info('Calibration triggered')

        self.target_wheel_speed_counts = 0
        self.speed_controller.EMERGENCY_STOP()
        self.steering_controller.Write_Calibration(0)

        response.success = True
        return response

    # Again, could probably be removed
    def turbo_mode_callback(self, request, response):
        self.turbo_mode = request.data
        response.success = True

        if self.turbo_mode:
            self.speed_controller.Write_KP(500)
            self.speed_controller.Write_KI(500) #112
            self.speed_controller.Write_KD(500) #15 #15
        else:
            self.speed_controller.Write_KP(500)
            self.speed_controller.Write_KI(500) #112
            self.speed_controller.Write_KD(500) #15

        response.message = str(self.turbo_mode)

        return response

    # Updates when the lidar safety system updates
    def lidar_blocking_callback(self, msg):
        self.lidar_forward_blocking = msg.data[0]
        self.lidar_reverse_blocking = msg.data[1]

    # Updates when radar safety system updates
    def radar_blocking_callback(self, msg):
        self.radar_forward_blocking = msg.data[0]

    # As mentioned above, continuosly write wheel speed and angle to the pico amd reads the actual speed and steering angle
    def timer_callback(self):
        if (self.lidar_forward_blocking or self.radar_forward_blocking) and self.target_wheel_speed_counts > 0:
            self.target_wheel_speed_counts = 0
            self.speed_controller.EMERGENCY_STOP()

        if self.lidar_reverse_blocking and self.target_wheel_speed_counts < 0:
            self.target_wheel_speed_counts = 0
            self.speed_controller.EMERGENCY_STOP()
        
        # See MainController.py for the source code for reads and writes
        self.speed_controller.Write_SetSpeed(self.target_wheel_speed_counts)
        self.steering_controller.Write_SetTarget(self.target_steering_angle_counts)

        actual_wheel_speed_counts = self.speed_controller.Read_Current_Count_Speed()
        actual_steering_angle_counts = self.steering_controller.Read_Current_Count()

        # Conversion from counts per 0.25 seconds to radians per second
        actual_wheel_speed_rad = actual_wheel_speed_counts * ((8 * 3.141592653) / self.counts_per_revolution)
        # Conversion from counts to radians
        actual_steering_angle_rad = ((actual_steering_angle_counts * self.total_steering_angle_rad) / self.steering_counts) - (self.total_steering_angle_rad / 2)

        self.get_logger().info('Current speed: %f' % (actual_wheel_speed_rad))
        self.get_logger().info('Current angle: %f' % (actual_steering_angle_rad))

        # Publish output to Car Controller
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = [actual_wheel_speed_rad, -actual_steering_angle_rad]

        self.pico_read_publisher.publish(msg)



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
