#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import os

class TerminalLauncherNode(Node):
    def __init__(self):
        super().__init__('drone_launcher_node')
        
        # Create a timer that triggers once after 2 seconds
        self.get_logger().info('Terminal Launcher Node initialized')
        
        # Store the commands to be executed in different tabs
        self.commands = [
            "MicroXRCEAgent udp4 -p 8888",  # Tab 1
        ]

    def launch_terminals_callback(self):
        for command in self.commands:
            self.get_logger().info(f"\nExecuting: {command}")
            with os.popen(command) as stream:
                while True:
                    output = stream.readline()
                    if output == '' and stream.closed:
                        break
                    if output:
                        self.get_logger().info(output.strip())
                self.get_logger().info("Done!!")


def main(args=None):
    rclpy.init(args=args)
    terminal_launcher = TerminalLauncherNode()
    try:
        terminal_launcher.launch_terminals_callback()
        terminal_launcher.destroy_node()
    except:
        pass
    finally:
        pass

if __name__ == '__main__':
    main()