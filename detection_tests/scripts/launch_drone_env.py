#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time

class TerminalLauncherNode(Node):
    def __init__(self):
        super().__init__('drone_launcher_node')
        
        self.get_logger().info('Terminal Launcher Node initialized')
        
        self.commands = [
            "MicroXRCEAgent udp4 -p 8888",  # Tab 1
            "cd  && ./QGroundControl.AppImage",              # Tab 3
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