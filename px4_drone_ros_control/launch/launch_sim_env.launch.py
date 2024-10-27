from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

import os

def generate_launch_description():
    px4_gz_bridge = os.path.join(
    get_package_share_directory('px4_drone_ros_control'),
    'launch',
    'control_bridge_node.launch.py'
    )
    # get_package_share_directory('px4_drone_ros_control'),
    # 'launch',
    # 'gz_px4_bridge.launch.py'
    # )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(px4_gz_bridge)
        ),
        Node(
            package='px4_drone_ros_control',          
            executable='MicroXRCEAgent_launcher.py',  
            output='screen',
        ),
        Node(
            package='px4_drone_ros_control',  
            executable='QGCS_launcher.py',    
            output='screen',
        ),
    ])
