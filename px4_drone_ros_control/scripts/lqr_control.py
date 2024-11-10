#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from scipy import linalg

class DroneReducedAttitudeController(Node):
    def __init__(self):
        super().__init__('drone_reduced_attitude_controller')
        
        # System parameters from the paper
        self.mass = 0.50  # kg
        self.l = 0.17     # m (distance from center to propeller)
        self.g = 9.81     # m/s^2
        self.I_xx = 3.2e-3  # kg*m^2
        self.I_zz = 5.5e-3  # kg*m^2
        self.I_Pzz = 1.5e-5  # kg*m^2 (propeller inertia)
        self.kf = 6.41e-6    # thrust coefficient
        self.ktau = 1.69e-2  # torque coefficient
        self.gamma = 2.75e-3 # drag coefficient
        self.motor_time_constant = 0.015  # seconds
        
        # Controller parameters
        self.hover_height = 1.0
        self.failed_motor = -1
        
        # Subscribers
        self.create_subscription(Int32, 'failed_motor', self.failed_motor_callback, 10)
        self.create_subscription(Odometry, 'drone/odom', self.state_callback, 10)
        
        # Publishers for motor commands
        self.motor_pubs = []
        for i in range(4):
            pub = self.create_publisher(Float64, f'motor{i+1}_cmd', 10)
            self.motor_pubs.append(pub)
            
        # Cascade control parameters
        self.zeta = 0.7  # damping ratio for position control
        self.wn = 2.0    # natural frequency for position control
        
        # Initialize state
        self.p = 0.0  # roll rate
        self.q = 0.0  # pitch rate
        self.r = 0.0  # yaw rate
        self.nx = 0.0  # primary axis x component
        self.ny = 0.0  # primary axis y component
        self.nz = 1.0  # primary axis z component
        
    def failed_motor_callback(self, msg):
        """Handle motor failure updates."""
        if self.failed_motor != msg.data:
            self.failed_motor = msg.data
            if self.failed_motor != -1:
                self.get_logger().info(f'Motor {self.failed_motor} failed, updating control strategy')
                # Recalculate LQR gains for the new configuration
                self.update_lqr_gains()
    
    def update_lqr_gains(self):
        """Update LQR gains based on reduced attitude dynamics."""
        # State vector: [p, q, nx, ny, f1_dev, f2_dev, f3_dev]
        # For one motor failure (assuming motor 4 failed)
        
        # System matrices including motor dynamics
        A = np.zeros((7, 7))
        
        # Angular velocity coupling terms from paper
        a_bar = ((self.I_xx - self.I_zz)/self.I_xx) * self.r - (self.I_Pzz/self.I_xx) * self.omega_sum
        
        # Basic attitude dynamics
        A[0:4, 0:4] = np.array([
            [0, a_bar, 0, 0],
            [-a_bar, 0, 0, 0],
            [0, -self.nz, 0, self.r],
            [self.nz, 0, -self.r, 0]
        ])
        
        # Motor dynamics
        A[4:7, 4:7] = -1/self.motor_time_constant * np.eye(3)
        
        # Input matrix
        B = np.zeros((7, 2))
        B[0:2, 0:2] = np.array([[0, 1], [1, 0]]) * self.l/self.I_xx
        B[4:7, 0:2] = 1/self.motor_time_constant * np.eye(2, 3).T
        
        # LQR weights
        Q = np.diag([100, 100, 50, 50, 1, 1, 1])  # State cost
        R = np.diag([1, 1])  # Input cost
        
        # Calculate LQR gains
        self.K = self.calculate_lqr_gain(A, B, Q, R)
    
    def calculate_lqr_gain(self, A, B, Q, R):
        """Calculate the LQR gain matrix."""
        P = linalg.solve_continuous_are(A, B, Q, R)
        return np.linalg.inv(R) @ B.T @ P
    
    def compute_desired_thrust_direction(self, pos_error, vel_error):
        """Compute desired thrust direction using cascaded control law."""
        # Desired acceleration from position controller
        acc_des = -2 * self.zeta * self.wn * vel_error - self.wn**2 * pos_error
        
        # Add gravity compensation
        acc_des[2] += self.g
        
        # Compute desired thrust direction
        thrust_dir = acc_des / np.linalg.norm(acc_des)
        
        return thrust_dir
    
    def state_callback(self, msg):
        """Update state and compute control inputs."""
        if self.failed_motor == -1:
            return
            
        # Extract current state
        pos = np.array([msg.pose.pose.position.x, 
                       msg.pose.pose.position.y, 
                       msg.pose.pose.position.z])
        vel = np.array([msg.twist.twist.linear.x,
                       msg.twist.twist.linear.y,
                       msg.twist.twist.linear.z])
        
        # Compute errors
        pos_error = pos - np.array([0, 0, self.hover_height])
        vel_error = vel
        
        # Get desired thrust direction
        n_des = self.compute_desired_thrust_direction(pos_error, vel_error)
        
        # Current reduced attitude state
        state = np.array([self.p, self.q, self.nx, self.ny])
        
        # Compute control input using LQR
        u = -self.K @ state
        
        # Convert control input to motor commands
        self.send_motor_commands(u, n_des)
    
    def send_motor_commands(self, u, n_des):
        """Convert control inputs to motor commands with one failed motor."""
        # Assuming motor 4 has failed
        if self.failed_motor == 4:
            # Base hover thrust
            hover_thrust = self.mass * self.g / 3.0
            
            # Compute individual motor thrusts
            f1 = hover_thrust + u[0]/2
            f2 = hover_thrust + u[1]
            f3 = hover_thrust - u[0]/2
            f4 = 0.0  # Failed motor
            
            # Convert forces to motor speeds
            w1 = np.sqrt(f1 / self.kf)
            w2 = np.sqrt(f2 / self.kf)
            w3 = np.sqrt(f3 / self.kf)
            w4 = 0.0
            
            # Publish commands
            for i, w in enumerate([w1, w2, w3, w4]):
                msg = Float64()
                msg.data = float(w)
                self.motor_pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = DroneReducedAttitudeController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()