#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header, Int16

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

import numpy as np
import math
import argparse
from collections import deque


def quaternion_to_rpy(w, x, y, z):
    """
    Convert quaternion to roll, pitch, and yaw (RPY) angles.
    
    Parameters:
    - x: float, x component of the quaternion
    - y: float, y component of the quaternion
    - z: float, z component of the quaternion
    - w: float, w component of the quaternion
    
    Returns:
    - roll: float, roll angle in radians
    - pitch: float, pitch angle in radians
    - yaw: float, yaw angle in radians
    """
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class CommandPublisherNode(Node):
    def __init__(self, args):
        super().__init__('command_publisher_node')
        self.logger = self.get_logger()
        self.controller_command_received = np.zeros(4)
        self.controller_command__to_send = np.zeros(4)
        self.command_values = [0, 0, 0, 0]
        self.motor_state_vector = np.ones(4)
        self.motor_command_pub = self.create_publisher(Actuators, '/x500/command/motor_speed', 10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscription with proper QoS
        self.subscription = self.create_subscription(
            Odometry,
            '/x500/odometry',
            self.odometry_callback,
            qos_profile)
        
        self.actuators_msg = Actuators()
        self.actuators_msg.header = Header()
        self.actuators_msg.header.stamp = self.get_clock().now().to_msg()
        self.actuators_msg.header.frame_id = ''
        self.actuators_msg.velocity = [float(0), float(0), float(0), float(0)]
        
        # Control and Detection
        self.rate = self.create_rate(5)  # 5 Hz means a 200ms interval
        
        # Initialize class attributes
        self.timestamp = None
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.ax = self.ay = self.az = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        
        # Previous velocities for acceleration calculation
        self.prev_vx = self.prev_vy = self.prev_vz = 0.0
        self.prev_timestamp = None
        
        # Store command values from args
        if len(args) == 1:
            self.command_values = [float(args[0]) for i in range(4)]
        elif len(args) == 4:
            self.command_values = [float(arg) for arg in args]
        
        # Create a timer to call command_pub every 1 second (adjust as needed)
        self.frequency = 100
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)  # 1 second interval
        
        # for odometry info
        self.last_secs_nsecs = [0, 0]
        self.last_wz = 0.0
        self.alpha_z = 0.0
        
        self.queue_len = 10
        self.queue_alpha = deque(maxlen=self.queue_len)
        self.counter = 0

    def timer_callback(self):
        """Callback function for the timer to publish commands."""
        self.command_pub(self.command_values)

    def calculate_accelerations(self, timestamp):
        """Calculate accelerations from velocity changes"""
        if self.prev_timestamp is None:
            self.ax = self.ay = self.az = 0.0
            return

        dt = (timestamp - self.prev_timestamp) * 1e-6  # Convert microseconds to seconds
        if dt > 0:
            self.ax = (self.vx - self.prev_vx) / dt
            self.ay = (self.vy - self.prev_vy) / dt
            self.az = (self.vz - self.prev_vz) / dt
            
            
    def odometry_callback(self, msg):
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        # Orientation
        _q = msg.pose.pose.orientation
        self.rpy = list(quaternion_to_rpy(_q.w, _q.x, _q.y, _q.z))
        # Linear velocities
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        # Angular velocities
        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z
        
        self.logger.info(f'wz: {self.wz}')
        
        # curr_time = self.get_clock().now()
        # del_t = curr_time.seconds_nanoseconds()[0]-self.last_secs_nsecs[0] + 1e-9 * (curr_time.seconds_nanoseconds()[1]-self.last_secs_nsecs[1])
        # # print('time: ', curr_time.seconds_nanoseconds())
        # del_wz = self.wz - self.last_wz
        # try:
        #     self.alpha_z = del_wz/del_t
        # except ZeroDivisionError:
        #     print('Zero Division Error')
        #     self.alpha_z = 0.0
        # self.last_secs_nsecs[0], self.last_secs_nsecs[1] = curr_time.seconds_nanoseconds()[0], curr_time.seconds_nanoseconds()[1]
        # self.last_wz = self.wz
        # # print('del_wz: ', del_wz, 'del_t: ', del_t, 'omega about z: ', self.wz, 'alpha_z: ', self.alpha_z)
        # if self.alpha_z!=0.0: 
        #     self.queue_alpha.append(self.alpha_z)
        #     self.counter+=1
        #     if self.counter % self.queue_len == 0:
        #         print('alpha_z: ', sum(self.queue_alpha)/self.queue_len)
        #         self.counter = 0
        
    
    def command_pub(self, msg: list):
        self.controller_command_received[0] = msg[0]
        self.controller_command_received[1] = msg[1]
        # self.controller_command_received[2] = msg[2]
        # self.controller_command_received[3] = msg[3]
        self.actuators_msg.header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info(f'Recieved Motor speed from PX4: {self.controller_command_received[0]} {self.controller_command_received[1]} {self.controller_command_received[2]} {self.controller_command_received[3]}')
        ## dot product
        self.actuators_msg.velocity = [float(v) for v in self.controller_command_received]
        # self.motor_command_pub.publish(self.actuators_msg)
        return 0
        



def main(args=None):
    rclpy.init()
    parser = argparse.ArgumentParser(description='Command Publisher Node')
    # parser.add_argument('--command', nargs=4, type=float, default=[100, 100, 100, 100],
    #                     help='Four command values to publish')
    parser.add_argument('--command', nargs=1, type=float, default=[100],
                        help='Four command values to publish')
    args = parser.parse_args()
    command_publisher_node = CommandPublisherNode(args.command)
    try:
        rclpy.spin(command_publisher_node)
    except KeyboardInterrupt:
        command_publisher_node.destroy_node()
        # rclpy.shutdown()
    finally:
        print('Exited!!')
        

if __name__ == '__main__':
    main()