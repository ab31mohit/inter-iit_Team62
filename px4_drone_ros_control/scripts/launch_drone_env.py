#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time

class TerminalLauncherNode(Node):
    def __init__(self):
        super().__init__('drone_launcher_node')
        
        # Create a timer that triggers once after 2 seconds
        self.get_logger().info('Terminal Launcher Node initialized')
        
        # Store the commands to be executed in different tabs
        self.commands = [
            "cd ~/ros2/PX4-Autopilot && make px4_sitl gz_x500",   # Tab 2
            "MicroXRCEAgent udp4 -p 8888",  # Tab 1
            "cd ~/ros2 && ./QGroundControl.AppImage",              # Tab 3
            "ros2 launch px4_drone_ros_control gz_px4_bridge.launch.py"              # Tab 4
        ]

    def launch_terminals_callback(self):
        for cmds in self.commands:
            subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", cmds])
            time.sleep(1)
        self.get_logger().info('Launched terminal with commands')

def main(args=None):
    rclpy.init(args=args)
    terminal_launcher = TerminalLauncherNode()
    try:
        terminal_launcher.launch_terminals_callback()
    except KeyboardInterrupt:
        pass
    finally:
        terminal_launcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()