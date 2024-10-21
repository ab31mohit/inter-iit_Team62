from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for the quadcopter."""
    pkg_project_bringup = get_package_share_directory("px4_drone_ros_control")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    
    ros2_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    bridge = Node(
        name='parameter_bridge_clock',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    motor_command_bypass_node = Node(
        name='motor_command_bypass_node', 
        package='px4_drone_ros_control', 
        executable='motor_command_bypass_node.py',
        output='screen'
    )

    return LaunchDescription(
        [
            ros2_gz_bridge,
            bridge,
            motor_command_bypass_node
        ]
    )
