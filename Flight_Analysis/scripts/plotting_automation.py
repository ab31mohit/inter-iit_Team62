#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import csv
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

true_positives = 0
false_positives = 0
true_negatives = 0
false_negatives = 0
Correct_1 = 0
Correct_2 = 0
Correct_3 = 0
Correct_4 = 0
Total = 0
Latency = []
true_accuracy = 0


class DataPlotterNode(Node):
    def __init__(self,odometry_file_path, px4_log_file_path, plot_file_path):
        super().__init__("data_plotter_node")

        self.odometry_file_path = odometry_file_path
        self.px4_log_file_path = px4_log_file_path
        self.plot_file_path = plot_file_path

        self.csv_files = []
        self.data = None
        self.failure_data = None
        self.max_index = -1
        self.injection_time = -1
        self.detection_time = -1
        self.injected_motor = -1
        self.detected_motor = -1
        self.latency = -1
        self.injection_unix = -1
        self.detection_unix = -1
        self.get_logger().info("Data Plotter Node has been started")
            

    def read_data(self):
        """Read the selected CSV file"""
        if not os.path.exists(self.odometry_file_path):
            self.get_logger().error(f"Failure data file '{self.odometry_file_path}' not found.")
            return None
        
        if not os.path.exists(self.px4_log_file_path):
                self.get_logger().error(f"Failure data file '{self.px4_log_file_path}' not found.")
                return None
        
        try:
            self.data = pd.read_csv(self.odometry_file_path)
            self.data = self.data[self.data["timestamp"] >= self.data["timestamp"].iloc[0]]
            self.get_logger().info(f"Data shape: {self.data.shape}")
            self.get_logger().info(f"Columns: {self.data.columns.tolist()}")

            self.failure_data = pd.read_csv(self.px4_log_file_path, header=None)
            self.get_logger().info(f"Loaded failure data file: {self.px4_log_file_path}")
            self.detection_time = self.failure_data.iloc[0, 0]  
            self.injection_time = self.failure_data.iloc[0, 1] 
            self.latency = self.failure_data.iloc[0, 2]  
            self.detected_motor = self.failure_data.iloc[0, 3]
            self.injected_motor = self.failure_data.iloc[0, 4]
            # self.detection_unix = self.failure_data.iloc[0,5]
            # self.injection_unix = self.failure_data.iloc[0,6]
            return True
        except Exception as e:
            self.get_logger().error(f"Error reading file: {str(e)}")
            return False


    def plot_data(self):
        global Correct_1, Correct_2, Correct_3, Correct_4, Total, Latency, true_positives, true_negatives, false_negatives, false_positives
        """Plot each column against the first column (timestamp)"""
        if self.data is None:
            self.get_logger().error("No data loaded")
            return
        timestamp_col = self.data.columns[0]
        detection_index = (self.data['timestamp'] - self.detection_unix).abs().idxmin()
        print(detection_index)
        timestamp_seconds = (
            self.data[timestamp_col] - self.data[timestamp_col].iloc[0]
        ) * 1e-6

        # Find the index where vz reaches its maximum value
        max_vz_index = self.data["vz"].idxmax()

        # Limit the data to the range up to and including the max vz
        data_limited = self.data.loc[:max_vz_index]
        timestamp_seconds_limited = timestamp_seconds.loc[:max_vz_index]

        # print(timestamp_seconds_limited)
        
        fig, axes = plt.subplots(5, 1, figsize=(15, 20))
        self.latency *= 1e-6
        # Set common title
        title = ""
        if(self.latency!=-1):
            title += f"Injection time = {self.injection_time} micro-sec    Detection time = {self.detection_time} micro-sec   latency = {self.latency} sec\n"
            title += f"Failure Injected in Motor: {self.injected_motor}     Failure Detected in Motor: {self.detected_motor}\n"
        else:
            title += "NO Failure NO detection"
        fig.suptitle(title, fontsize=16)

        # Position plots
        axes[0].plot(timestamp_seconds_limited, data_limited[["x", "y", "z"]])
        axes[0].set_title("Position")
        axes[0].set_ylabel("Position (m)")
        axes[0].legend(["x", "y", "z"])
        axes[0].grid(True)
        # for idx in injected_failure_indices:
        # axes[0].axvline(x=timestamp_seconds_limited[detection_index], color="red", linestyle="solid")

        # Velocity plots
        axes[1].plot(timestamp_seconds_limited, data_limited[["vx", "vy", "vz"]])
        axes[1].set_title("Velocity")
        axes[1].set_ylabel("Velocity (m/s)")
        axes[1].legend(["vx", "vy", "vz"])
        axes[1].grid(True)
        # axes[1].axvline(x=timestamp_seconds_limited[detection_index], color="red", linestyle="solid")

        # Averaged acceleration plots
        acc_avg = data_limited[["ax", "ay", "az"]].rolling(window=20).mean().fillna(0)
        axes[2].plot(timestamp_seconds_limited, acc_avg)
        axes[2].set_title("Acceleration (Averaged)")
        axes[2].set_ylabel("Acceleration (m/s²)")
        axes[2].legend(["ax", "ay", "az"])
        axes[2].grid(True)
        # axes[2].axvline(x=timestamp_seconds_limited[detection_index], color="red", linestyle="solid")
        # Attitude plots
        axes[3].plot(timestamp_seconds_limited, data_limited[["roll", "pitch", "yaw"]])
        axes[3].set_title("Attitude")
        axes[3].set_ylabel("Angle (rad)")
        axes[3].legend(["roll", "pitch", "yaw"])
        axes[3].grid(True)
        # axes[3].axvline(x=timestamp_seconds_limited[detection_index], color="red", linestyle="solid")

        # Averaged attitude rate plots
        rate_avg = data_limited[["roll_rate", "pitch_rate", "yaw_rate"]].rolling(window=20).mean().fillna(0)
        axes[4].plot(timestamp_seconds_limited, rate_avg)
        axes[4].set_title("Attitude Rates (Averaged)")
        axes[4].set_ylabel("Rate (rad/s)")
        axes[4].legend(["roll_rate", "pitch_rate", "yaw_rate"])
        axes[4].grid(True)
        # axes[4].axvline(x=timestamp_seconds_limited[detection_index], color="red", linestyle="solid")
        # Set common x-label
        fig.text(0.5, 0.02, "Time (seconds)", ha="center", fontsize=12)


        # Adjust layout: Add space between subplots
        plt.tight_layout(rect=[0, 0.03, 1, 0.97])

        # Save the plot
        plot_filename = self.plot_file_path.split("/")[-1]
        plt.savefig(self.plot_file_path)
        self.get_logger().info(f"Plots saved as {plot_filename}")
        if(self.injected_motor*self.detected_motor > 0): 
            if(self.injected_motor==self.detected_motor): true_positives += 1
            else: false_positives +=1 
        elif(self.injected_motor > 0 and self.detected_motor < 0): false_negatives += 1
        elif(self.injected_motor < 0 and self.detected_motor > 0): false_positives += 1
        elif(self.injected_motor < 0 and self.detected_motor < 0): true_negatives += 1
    
        Total += 1

        if(self.injected_motor == self.detected_motor):
            if(self.injected_motor == 1):
                Correct_1 += 1
            if(self.injected_motor == 2):
                Correct_2 += 1
            if(self.injected_motor == 3):
                Correct_3 += 1
            if(self.injected_motor == 4):
                Correct_4 += 1 
        

        Latency.append(self.latency)  
        # Show the plot
        # plt.show()

    def execute(self):
        if self.read_data():
            self.plot_data()
            # Shutdown node after plotting
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):

    rclpy.init(args=None)

    try:
        csv_directory_path = os.path.join(os.path.expanduser("~"), "inter-iit_Team62", "Flight_Analysis", "detection_logs", "csv")
        px4_logs_directory_path = os.path.join(os.path.expanduser("~"), "inter-iit_Team62", "Flight_Analysis", "detection_logs", "px4_logs")
        plots_directory_path = os.path.join(os.path.expanduser("~"), "inter-iit_Team62", "Flight_Analysis", "detection_logs", "plots")

        odometry_files = sorted([f for f in os.listdir(csv_directory_path) if f.startswith("odometry_data_") and f.endswith(".csv")])
        px4_log_files = sorted([f for f in os.listdir(px4_logs_directory_path) if f.startswith("px4_logs_") and f.endswith(".csv")])

        for odometry_file, px4_log_file in zip(odometry_files, px4_log_files):
            index = odometry_file.split("_")[-1].split(".")[0]
            odometry_file_path = os.path.join(csv_directory_path, odometry_file)
            px4_log_file_path = os.path.join(px4_logs_directory_path, px4_log_file)
            plot_file_path = os.path.join(plots_directory_path, f"odometry_plot_{index}.png")

            data_plotter = DataPlotterNode(odometry_file_path, px4_log_file_path, plot_file_path)

            try:
                data_plotter.execute()
            except KeyboardInterrupt:
                break
            finally:
                data_plotter.destroy_node()
                rclpy.init(args=None)

        # Compiling results File:
        # Correct = Correct_1 + Correct_2 + Correct_3 + Correct_4
        Accuracy = (true_positives + true_negatives) * 100 / Total if Total > 0 else 0
        # true_accuracy = (Correct_1+Correct_2+Correct_3+Correct_4)*100/Total if Total>0 else 0
        data = {
            "Total_Sessions": Total,
            "Accuracy": Accuracy,
            "True_Positives": true_positives,
            "True_Negatives": true_negatives,
            "False_positives": false_negatives,
            'False_Negatives': false_negatives,
            "Correct_1": Correct_1,
            "Correct_2": Correct_2,
            "Correct_3": Correct_3,
            "Correct_4": Correct_4,
            "Average_Latancy": np.mean(Latency),
        }
        
        results_file_path = os.path.join(os.path.expanduser("~"), "inter-iit_Team62", "Flight_Analysis", "detection_logs", "results.csv")

        with open(results_file_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Metric", "Value"])
            for key, value in data.items():
                writer.writerow([key, value])

    finally:
        # Shutdown rclpy
        rclpy.shutdown()


if __name__ == "__main__":
    main()