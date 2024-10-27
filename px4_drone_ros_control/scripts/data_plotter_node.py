#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

class DataPlotterNode(Node):
    def __init__(self):
        super().__init__("data_plotter_node")

        # Declare parameters
        self.declare_parameter(
            "data_directory",
            "/home/kuldeep/inter-iit_ws/src/Inter-IIT_IdeaForge-PS/px4_drone_ros_control/logs",
        )

        # Get parameters
        self.directory_path = self.get_parameter("data_directory").value
        if not self.directory_path:
            self.directory_path = os.getcwd()
            self.get_logger().info(
                f"No directory specified, using current directory: {self.directory_path}"
            )

        self.csv_files = []
        self.selected_file = None
        self.data = None

        self.get_logger().info("Data Plotter Node has been started")

    def list_csv_files(self):
        """List all CSV files in the directory and select the one with max integer suffix"""
        # Get all csv files
        self.csv_files = [
            f for f in os.listdir(self.directory_path) if f.endswith(".csv")
        ]

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
        timestamp_seconds = (
            self.data[timestamp_col] - self.data[timestamp_col].iloc[0]
        ) * 1e-6

        # Find the index where vz reaches its maximum value
        max_vz_index = self.data["vz"].idxmax()

        # Limit the data to the range up to and including the max vz
        data_limited = self.data.loc[:max_vz_index]
        timestamp_seconds_limited = timestamp_seconds.loc[:max_vz_index]

        # Identify the indices where the failed_motor changes from 0 to non-zero
        failed_motor_changes = data_limited["failed_motor"].ne(0).astype(int).diff().eq(1)
        failed_motor_indices = data_limited.index[failed_motor_changes].tolist()

        # Create subplots for each measurement group
        fig, axes = plt.subplots(6, 1, figsize=(15, 20))

        # Position plots
        axes[0].plot(timestamp_seconds_limited, data_limited[["x", "y", "z"]])
        axes[0].set_title("Position")
        axes[0].set_ylabel("Position (m)")
        axes[0].legend(["x", "y", "z"])
        axes[0].grid(True)
        for idx in failed_motor_indices:
            axes[0].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        # Velocity plots
        axes[1].plot(timestamp_seconds_limited, data_limited[["vx", "vy", "vz"]])
        axes[1].set_title("Velocity")
        axes[1].set_ylabel("Velocity (m/s)")
        axes[1].legend(["vx", "vy", "vz"])
        axes[1].grid(True)
        for idx in failed_motor_indices:
            axes[1].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        # Acceleration plots
        # axes[2].plot(timestamp_seconds_limited, data_limited[["ax", "ay", "az"]])
        # axes[2].set_title("Acceleration")
        # axes[2].set_ylabel("Acceleration (m/s²)")
        # axes[2].legend(["ax", "ay", "az"])
        # axes[2].grid(True)
        # for idx in failed_motor_indices:
        #     axes[2].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        # Averaged acceleration plots
        acc_avg = data_limited[["ax", "ay", "az"]].rolling(window=20).mean()
        axes[2].plot(timestamp_seconds_limited, acc_avg)
        axes[2].set_title("Acceleration (Averaged)")
        axes[2].set_ylabel("Acceleration (m/s²)")
        axes[2].legend(["ax", "ay", "az"])
        axes[2].grid(True)
        for idx in failed_motor_indices:
            axes[2].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        # Attitude plots
        axes[3].plot(timestamp_seconds_limited, data_limited[["roll", "pitch", "yaw"]])
        axes[3].set_title("Attitude")
        axes[3].set_ylabel("Angle (rad)")
        axes[3].legend(["roll", "pitch", "yaw"])
        axes[3].grid(True)
        for idx in failed_motor_indices:
            axes[3].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        # Attitude rate plots
        # axes[4].plot(timestamp_seconds_limited, data_limited[["roll_rate", "pitch_rate", "yaw_rate"]])
        # axes[4].set_title("Attitude Rates")
        # axes[4].set_ylabel("Rate (rad/s)")
        # axes[4].legend(["roll_rate", "pitch_rate", "yaw_rate"])
        # axes[4].grid(True)
        # for idx in failed_motor_indices:
        #     axes[4].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        # Averaged attitude rate plots
        rate_avg = data_limited[["roll_rate", "pitch_rate", "yaw_rate"]].rolling(window=20).mean()
        axes[4].plot(timestamp_seconds_limited, rate_avg)
        axes[4].set_title("Attitude Rates (Averaged)")
        axes[4].set_ylabel("Rate (rad/s)")
        axes[4].legend(["roll_rate", "pitch_rate", "yaw_rate"])
        axes[4].grid(True)
        for idx in failed_motor_indices:
            axes[4].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        #Averaged attitude acceleration plots
        att_acc_avg = data_limited[["roll_acc", "pitch_acc", "yaw_acc"]].rolling(window=100).mean()
        axes[5].plot(timestamp_seconds_limited, att_acc_avg)
        axes[5].set_title("Attitude Acceleration (Averaged)")
        axes[5].set_ylabel("Acceleration (rad/s²)")
        axes[5].legend(["roll_acc", "pitch_acc", "yaw_acc"])
        axes[5].grid(True)
        for idx in failed_motor_indices:
            axes[5].axvline(x=timestamp_seconds_limited[idx], color="black", linestyle="dotted")

        # Set common x-label
        fig.text(0.5, 0.02, "Time (seconds)", ha="center", fontsize=12)


        # Adjust layout: Add space between subplots
        plt.tight_layout(rect=[0, 0.03, 1, 0.97])

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


if __name__ == "__main__":
    main()