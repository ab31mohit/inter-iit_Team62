#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def find_package_directory(package_name):
    # Get the AMENT_PREFIX_PATH environment variable
    ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')

    if not ament_prefix_path:
        return "AMENT_PREFIX_PATH is not set. ROS 2 might not be sourced."

    # Split the AMENT_PREFIX_PATH if it contains multiple paths
    package_directories = ament_prefix_path.split(':')

    # Search for the package in each directory
    for package_dir in package_directories:
        # Typically, packages are located under `share/package_name`
        possible_path = os.path.join(package_dir, 'share', package_name)
        if os.path.isdir(possible_path):
            return package_dir[:-(len(package_name)+9)]

    return f"Package '{package_name}' not found in AMENT_PREFIX_PATH."

class DataPlotterNode(Node):
    def __init__(self):
        super().__init__('data_plotter_node')
        
        # Declare parameters
        self.workspace_dir = find_package_directory('px4_drone_ros_control')
        self.declare_parameter('data_directory', self.workspace_dir + '/src/Inter-IIT_IdeaForge-PS/px4_drone_ros_control/logs')
        
        # Get parameters
        self.directory_path = self.get_parameter('data_directory').value
        if not self.directory_path:
            self.directory_path = os.getcwd()
            self.get_logger().info(f'No directory specified, using current directory: {self.directory_path}')
        
        self.csv_files = []
        self.selected_file = None
        self.data = None
        
        self.get_logger().info('Data Plotter Node has been started')

    def list_csv_files(self):
        """List all CSV files in the directory and select the one with max integer suffix"""
        # Get all csv files
        self.csv_files = [f for f in os.listdir(self.directory_path) if f.endswith('.csv')]
        
        if not self.csv_files:
            self.get_logger().error(f"No CSV files found in {self.directory_path}")
            return False

        # Remove extension and find file with max integer suffix
        file_numbers = []
        for file in self.csv_files:
            # Remove extension
            base_name = os.path.splitext(file)[0]
            # Get last character and convert to int if possible
            try:
                num = int(base_name[-1])
                file_numbers.append((file, num))
            except ValueError:
                continue

        if not file_numbers:
            self.get_logger().error("No files with numeric suffixes found")
            return False

        # Sort by number and get the file with max number
        self.selected_file = max(file_numbers, key=lambda x: x[1])[0]
        self.get_logger().info(f"Selected file: {self.selected_file}")
        return True

    def read_data(self):
        """Read the selected CSV file"""
        if self.selected_file is None:
            self.get_logger().error("No file selected")
            return False

        file_path = os.path.join(self.directory_path, self.selected_file)
        try:
            self.data = pd.read_csv(file_path)
            self.get_logger().info(f"Data shape: {self.data.shape}")
            self.get_logger().info(f"Columns: {self.data.columns.tolist()}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error reading file: {str(e)}")
            return False

    def plot_data(self):
        """Plot each column against the first column (timestamp)"""
        if self.data is None:
            self.get_logger().error("No data loaded")
            return

        # Get the timestamp column (first column)
        timestamp_col = self.data.columns[0]
        
        # Convert microseconds to seconds for better readability
        timestamp_seconds = (self.data[timestamp_col] - self.data[timestamp_col].iloc[0]) * 1e-6

        # Create subplots for each measurement group
        fig, axes = plt.subplots(4, 1, figsize=(15, 20))
        fig.suptitle(f'Data from {self.selected_file}', fontsize=16)

        # Position plots
        axes[0].plot(timestamp_seconds, self.data[['x', 'y', 'z']])
        axes[0].set_title('Position')
        axes[0].set_ylabel('Position (m)')
        axes[0].legend(['x', 'y', 'z'])
        axes[0].grid(True)

        # Velocity plots
        axes[1].plot(timestamp_seconds, self.data[['vx', 'vy', 'vz']])
        axes[1].set_title('Velocity')
        axes[1].set_ylabel('Velocity (m/s)')
        axes[1].legend(['vx', 'vy', 'vz'])
        axes[1].grid(True)

        # Acceleration plots
        axes[2].plot(timestamp_seconds, self.data[['ax', 'ay', 'az']])
        axes[2].set_title('Acceleration')
        axes[2].set_ylabel('Acceleration (m/s²)')
        axes[2].legend(['ax', 'ay', 'az'])
        axes[2].grid(True)

        # Attitude plots
        axes[3].plot(timestamp_seconds, self.data[['roll', 'pitch', 'yaw']])
        axes[3].set_title('Attitude')
        axes[3].set_ylabel('Angle (rad)')
        axes[3].legend(['roll', 'pitch', 'yaw'])
        axes[3].grid(True)

        # Set common x-label
        fig.text(0.5, 0.04, 'Time (seconds)', ha='center', fontsize=12)

        # Adjust layout
        plt.tight_layout()
        
        # Save the plot
        plot_filename = f"/odometry_plots_{os.path.splitext(self.selected_file)[0]}.png"
        plt.savefig(self.directory_path + plot_filename)
        self.get_logger().info(f"Plots saved as {plot_filename}")
        
        # Show the plot
        plt.show()

    def execute(self):
        if self.list_csv_files() and self.read_data():
            self.plot_data()
            # Shutdown node after plotting
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    data_plotter = DataPlotterNode()
    try:
        data_plotter.execute()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            data_plotter.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()