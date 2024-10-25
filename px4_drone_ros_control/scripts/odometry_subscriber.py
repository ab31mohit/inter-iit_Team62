#!/usr/bin/env python3

import rclpy
import rclpy.wait_for_message
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Int16
import numpy as np
import pandas as pd
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
        super().__init__("odometry_subscriber")

        # Initialize class attributes
        self.timestamp = None
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.ax = self.ay = self.az = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.roll_rate = self.pitch_rate = self.yaw_rate = 0.0
        self.failed_motor = -1
        self.detected_motor_failure = -1
        self.motor_failed_timestamp = None
        self.failure_detected_timestamp = None

        # Previous values for rate and acceleration calculations (store in radians)
        self.prev_roll_rad = self.prev_pitch_rad = self.prev_yaw_rad = 0.0
        self.prev_timestamp = None

        # Initialize a list to store the last 20 az values for failure detection
        self.last_20 = []
        self.threshold_acceleration = 4.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Create subscriptions
        self.subscription = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos_profile,
        )

        self.subscription = self.create_subscription(
            Int16,
            "/drone_control/toggle_motor_fail_state",
            self.motor_failure_callback,
            10,
        )

        # Initialize CSV file
        self.log_directory = "/home/shravan/inter-iit_ws/src/Inter-IIT_IdeaForge-PS/px4_drone_ros_control/logs"
        self.csv_filename = "/odometry_data_0.csv"
        if os.path.exists(self.log_directory + self.csv_filename):
            while os.path.exists(self.log_directory + self.csv_filename):
                self.csv_filename = (
                    self.csv_filename[:-5]
                    + str(int(self.csv_filename[-5]) + 1)
                    + self.csv_filename[-4:]
                )
        self.csv_file = open(self.log_directory + self.csv_filename, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # Write header with new angular acceleration columns
        self.csv_writer.writerow(
            [
                "timestamp",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "ax",
                "ay",
                "az",
                "roll",
                "pitch",
                "yaw",
                "roll_rate",
                "pitch_rate",
                "yaw_rate",
                "failed_motor",
                "detected_motor_failure",
            ]
        )

        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info(
            "Odometry subscriber started. Recording data to: " + self.csv_filename
        )

    def signal_handler(self, sig, frame):
        self.get_logger().info("Closing CSV file and shutting down...")
        if hasattr(self, "csv_file"):
            self.csv_file.close()
        sys.exit(0)

    def calculate_accelerations(self, timestamp):
        if self.prev_timestamp is None:
            self.ax = self.ay = self.az = 0.0
            return

        dt = (timestamp - self.prev_timestamp) * 1e-6
        if dt > 0:
            self.ax = (self.vx - self.prev_vx) / dt
            self.ay = (self.vy - self.prev_vy) / dt
            self.az = (self.vz - self.prev_vz) / dt

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_angular_rate(self, current, previous, dt):
        """Calculate angular rate accounting for wraparound"""
        diff = self.normalize_angle(current - previous)
        return diff / dt if dt > 0 else 0.0

    def calculate_attitude_rates(self, timestamp, roll_rad, pitch_rad, yaw_rad):
        """Calculate attitude rates from attitude changes in radians"""
        if self.prev_timestamp is None:
            self.roll_rate = self.pitch_rate = self.yaw_rate = 0.0
            return

        dt = (timestamp - self.prev_timestamp) * 1e-6  # Convert microseconds to seconds
        if dt > 0:
            # Calculate rates in radians/sec, handling wraparound
            roll_rate_rad = self.calculate_angular_rate(roll_rad, self.prev_roll_rad, dt)
            pitch_rate_rad = self.calculate_angular_rate(pitch_rad, self.prev_pitch_rad, dt)
            yaw_rate_rad = self.calculate_angular_rate(yaw_rad, self.prev_yaw_rad, dt)

            # Convert to degrees/sec for output
            self.roll_rate = np.degrees(roll_rate_rad)
            self.pitch_rate = np.degrees(pitch_rate_rad)
            self.yaw_rate = np.degrees(yaw_rate_rad)

    def detect_failure(self):
        # Append the new acceleration to last_20_az
        if len(self.last_20) < 20:
            self.last_20.append([self.az, self.pitch_rate, self.yaw_rate])
        else:
            self.last_20.pop(0)
            self.last_20.append([self.az, self.pitch_rate, self.yaw_rate])
        
        # Calculate the average of the last 20 az values
        avg_az = np.mean(self.last_20, axis=0)[0]
        avg_pitch_rate = np.mean(self.last_20, axis=0)[1]
        avg_yaw_rate = np.mean(self.last_20, axis=0)[2]
        if avg_az > self.threshold_acceleration:
            self.failure_detected_timestamp = self.timestamp
            if avg_pitch_rate < 0 and avg_yaw_rate < 0:
                self.detected_motor_failure = 0
            if avg_pitch_rate > 0 and avg_yaw_rate < 0:
                self.detected_motor_failure = 1
            if avg_pitch_rate < 0 and avg_yaw_rate > 0:
                self.detected_motor_failure = 2
            if avg_pitch_rate > 0 and avg_yaw_rate > 0:
                self.detected_motor_failure = 3
            self.get_logger().info(f"Motor failure detected: {self.detected_motor_failure}")
            self.get_logger().info(f"Delay in detection: {(self.failure_detected_timestamp - self.motor_failed_timestamp) * 1e-6:.3f}s")

    def motor_failure_callback(self, msg):
        self.failed_motor = msg.data
        self.motor_failed_timestamp = self.timestamp
        self.get_logger().info(f"Motor {self.failed_motor} failed")

    def odometry_callback(self, msg):
        self.timestamp = msg.timestamp

        # Store position and velocity
        self.x, self.y, self.z = msg.position
        self.vx, self.vy, self.vz = msg.velocity

        # Calculate accelerations
        self.calculate_accelerations(msg.timestamp)

        # Get current angles in radians
        roll_rad, pitch_rad, yaw_rad = quaternion_to_rpy(
            msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        )

        # Calculate attitude rates (using radians)
        self.calculate_attitude_rates(msg.timestamp, roll_rad, pitch_rad, yaw_rad)
        
        # Store angles in degrees for output
        self.roll = np.degrees(roll_rad)
        self.pitch = np.degrees(pitch_rad)
        self.yaw = np.degrees(yaw_rad)

        # Update previous values
        self.prev_vx = self.vx
        self.prev_vy = self.vy
        self.prev_vz = self.vz
        self.prev_roll_rad = roll_rad
        self.prev_pitch_rad = pitch_rad
        self.prev_yaw_rad = yaw_rad
        self.prev_timestamp = self.timestamp

        # Write data to CSV
        self.csv_writer.writerow(
            [
                self.timestamp,
                self.x,
                self.y,
                self.z,
                self.vx,
                self.vy,
                self.vz,
                self.ax,
                self.ay,
                self.az,
                self.roll,
                self.pitch,
                self.yaw,
                self.roll_rate,
                self.pitch_rate,
                self.yaw_rate,
                self.failed_motor,
                self.detected_motor_failure,
            ]
        )

        self.get_logger().info(
            f"{[self.timestamp, self.x, self.y, self.z, self.vx, self.vy, self.vz, self.ax, self.ay, self.az, self.roll, self.pitch, self.yaw, self.roll_rate, self.pitch_rate, self.yaw_rate, self.failed_motor, self.detected_motor_failure]} - written to file"
        )

        if self.detected_motor_failure == -1:
            self.detect_failure()   

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


if __name__ == "__main__":
    main()
