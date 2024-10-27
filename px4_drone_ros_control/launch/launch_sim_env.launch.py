from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    px4_gz_bridge = os.path.join(
    get_package_share_directory('px4_drone_ros_control'),
    'launch',
    'gz_px4_bridge.launch.py'
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(px4_gz_bridge)
        ),
        Node(
            package='px4_drone_ros_control',  # Replace with the name of your package
            executable='MicroXRCEAgent_launcher.py',  # The name of the Python node executable
            output='screen',       # Output will be printed to the terminal
        ),
        Node(
            package='px4_drone_ros_control',  # Replace with the name of your package
            executable='QGCS_launcher.py',  # The name of the Python node executable
            output='screen',       # Output will be printed to the terminal
        ),
    ])
