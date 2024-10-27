from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the path to the PX4 source directory (adjust as necessary)
    px4_src_path = os.path.join(os.environ['HOME'], 'ros2', 'PX4-Autopilot')

    return LaunchDescription([
        Node(
            package='px4',
            executable='px4',
            name='px4_sitl',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://:14540'},  # UDP connection for PX4 SITL
                {'model': 'x500'},             # Specify the model for the drone
                {'sitl': True},                # Indicate SITL mode
                # Add other necessary parameters here as needed
            ],
            remappings=[
                # Add any topic remappings if necessary
            ],
        ),
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=['--verbose', os.path.join(px4_src_path, 'Tools', 'gzs', 'worlds', 'x500.world')],  # Path to the world file if needed
        ),
    ])
