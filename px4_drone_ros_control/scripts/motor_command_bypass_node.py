#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from actuator_msgs.msg import Actuators
import numpy as np
from std_msgs.msg import Header, Int16, Float64



class MotorCommandBypass(Node):
    def __init__(self):
        super().__init__('motor_command_bypass_node')
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
        
        self.actuators_msg = Actuators()
        self.actuators_msg.header = Header()
        self.actuators_msg.header.stamp = self.get_clock().now().to_msg()
        self.actuators_msg.header.frame_id = ''
        self.actuators_msg.velocity = [float(0), float(0), float(0), float(0)]


    def listener_callback(self, msg: Actuators):
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
        
        
    def toggle_motor_fail_state(self, msg):
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



def main(args=None):
    rclpy.init()
    motor_command_bypass_node = MotorCommandBypass()
    rclpy.spin(motor_command_bypass_node)
    motor_command_bypass_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
