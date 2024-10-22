#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
import numpy as np
import csv
import signal
import sys, math, os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


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



class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        
        # Initialize class attributes
        self.timestamp = None
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.ax = self.ay = self.az = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        
        # Previous velocities for acceleration calculation
        self.prev_vx = self.prev_vy = self.prev_vz = 0.0
        self.prev_timestamp = None
        
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
        
        # Initialize CSV file
        self.log_directory = '/home/rajeev-gupta/ros2/inter-iit_ws/src/Inter-IIT_IdeaForge-PS/px4_drone_ros_control/logs'
        self.csv_filename = '/odometry_data_0.csv'
        if os.path.exists(self.log_directory + self.csv_filename):
            while os.path.exists(self.log_directory + self.csv_filename):
                self.csv_filename = self.csv_filename[:-5] + str(int(self.csv_filename[-5])+1) + self.csv_filename[-4:]
        self.csv_file = open(self.log_directory+self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'z', 
                                'vx', 'vy', 'vz', 
                                'ax', 'ay', 'az', 
                                'roll', 'pitch', 'yaw'])
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('Odometry subscriber started. Recording data to: ' + self.csv_filename)

    def signal_handler(self, sig, frame):
        """Handle cleanup on shutdown"""
        self.get_logger().info('Closing CSV file and shutting down...')
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        sys.exit(0)

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

        # Write data to CSV
        self.csv_writer.writerow([
            self.timestamp,
            self.x, self.y, self.z,
            self.vx, self.vy, self.vz,
            self.ax, self.ay, self.az,
            self.roll, self.pitch, self.yaw
        ])
        
        self.get_logger().info(f'{[self.timestamp, self.x, self.y, self.z, self.vx, self.vy, self.vz, self.ax, self.ay, self.az, self.roll, self.pitch, self.yaw]} - written to file')
        
        # Update previous values for next acceleration calculation
        self.prev_vx = self.vx
        self.prev_vy = self.vy
        self.prev_vz = self.vz
        self.prev_timestamp = self.timestamp

def main(args=None):
    rclpy.init(args=args)
    
    odometry_subscriber = OdometrySubscriber()
    
    try:
        rclpy.spin(odometry_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        odometry_subscriber.csv_file.close()
        odometry_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()