#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header, Int16

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry

import numpy as np
import math

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

class ControlBridgeNode(Node):
    def __init__(self):
        super().__init__('control_bridge_node')
        self.subscription = self.create_subscription(
            Actuators,
            '/x500_0/command/motor_speed',
            self.listener_callback,
            10)
        self.subscription = self.create_subscription(
            Int16,
            '/drone_control/toggle_motor_fail_state',
            self.toggle_motor_fail_state,
            10)
        self.subscription
        self.logger = self.get_logger()
        self.controller_command_received = np.zeros(4)
        self.controller_command__to_send = np.zeros(4)
        self.motor_state_vector = np.ones(4)
        self.motor_command_pub = self.create_publisher(Actuators, '/x500_0/command/motor_vel', 10)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscription with proper QoS
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile)
        
        self.actuators_msg = Actuators()
        self.actuators_msg.header = Header()
        self.actuators_msg.header.stamp = self.get_clock().now().to_msg()
        self.actuators_msg.header.frame_id = ''
        self.actuators_msg.velocity = [float(0), float(0), float(0), float(0)]
        
        # Control and Detection
        self.rate = self.create_rate(5)  # 5 Hz means a 200ms interval
        self.detected_motor_index = None
        self.switch_contol = False
        
        # Initialize class attributes
        self.timestamp = None
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.ax = self.ay = self.az = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        
        # Previous velocities for acceleration calculation
        self.prev_vx = self.prev_vy = self.prev_vz = 0.0
        self.prev_timestamp = None
        
        self.motor_toggle_timestamp = 0.0



    def listener_callback(self, msg: Actuators):
        if self.switch_contol and self.detected_motor_index is not None:
            for i in range(4):
                self.controller_command_received[i] = self.control_function()[i]
        else: 
            self.controller_command_received[0] = msg.velocity[0]
            self.controller_command_received[1] = msg.velocity[1]
            self.controller_command_received[2] = msg.velocity[2]
            self.controller_command_received[3] = msg.velocity[3]

        # self.get_logger().info(f'Recieved Motor speed from PX4: {self.controller_command_received[0]} {self.controller_command_received[1]} {self.controller_command_received[2]} {self.controller_command_received[3]}')
        ## dot product
        self.controller_command__to_send = self.controller_command_received * self.motor_state_vector
        self.actuators_msg.velocity = [float(v) for v in self.controller_command__to_send]
        self.motor_command_pub.publish(self.actuators_msg)
        # self.get_logger().info(f'Sent Motor vel to Gazebo: {self.controller_command__to_send[0]} {self.controller_command__to_send[1]} {self.controller_command__to_send[2]} {self.controller_command__to_send[3]}')
        
        
    def toggle_motor_fail_state(self, msg) -> bool:
        current_time = self.get_clock().now().seconds_nanoseconds()
        seconds = current_time[0] + current_time[1] * 1e-9
        # self.logger.info(f'{seconds}, {self.motor_toggle_timestamp + 1}')
        if seconds < self.motor_toggle_timestamp + 1:
            self.motor_toggle_timestamp = seconds
            return False
        
        index = msg.data
        self.logger.info('-----------')
        self.logger.info('header')
        fail_status = "motor status failed for index:"
        for i in range(len(self.motor_state_vector)):
            if self.motor_state_vector[i] == 0:
                fail_status+=f' {i}'
        self.logger.info(fail_status)
        self.motor_state_vector[index] = 1 - self.motor_state_vector[index]
        self.logger.info(f"motor status failed for index {index}" if self.motor_state_vector[index] == 0 else f"motor status working for index {index}")
        self.logger.info('-----------\n\n')
        
        # TODO: impelment detection
        # self.rate.sleep()
        # Assuming that the detection has been done
        # if not self.switch_contol or self.detected_motor_index is None:
        #     self.switch_contol = True
        #     self.detected_motor_index = index
        
        
        # save time of recent toggle
        self.motor_toggle_timestamp = seconds
        return True
        
        
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
        """Process incoming odometry messages"""
        # Store timestamp
        self.timestamp = msg.timestamp

        # Store position
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = msg.position[2]

        # Store velocity
        self.vx = msg.velocity[0]
        self.vy = msg.velocity[1]
        self.vz = msg.velocity[2]

        # Calculate accelerations
        self.calculate_accelerations(msg.timestamp)

        # Convert quaternion to euler angles (roll, pitch, yaw)
        self.roll, self.pitch, self.yaw = quaternion_to_rpy(msg.q[0], msg.q[1], msg.q[2], msg.q[3])

        # Convert angles to degrees
        self.roll = np.degrees(self.roll)
        self.pitch = np.degrees(self.pitch)
        self.yaw = np.degrees(self.yaw)
        
        # Update previous values for next acceleration calculation
        self.prev_vx = self.vx
        self.prev_vy = self.vy
        self.prev_vz = self.vz
        
        # self.get_logger().info(f'{[self.timestamp, self.x, self.y, self.z, self.vx, self.vy, self.vz, self.ax, self.ay, self.az, self.roll, self.pitch, self.yaw]}')
        self.prev_timestamp = self.timestamp
        
    def control_function(self):
        t = 700
        return t, t, t, t
        



def main(args=None):
    rclpy.init()
    control_bridge_node = ControlBridgeNode()
    rclpy.spin(control_bridge_node)
    control_bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()