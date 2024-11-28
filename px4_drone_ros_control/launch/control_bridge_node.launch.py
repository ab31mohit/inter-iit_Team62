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
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    """Generate a launch description for the quadcopter."""
    pkg_project_bringup = get_package_share_directory("px4_drone_ros_control")    
    ros2_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    clock_bridge = Node(
        name='parameter_bridge_clock',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    control_bridge_node = Node(
        name='control_bridge_node', 
        package='px4_drone_ros_control', 
        executable='control_bridge_node.py',
        output='screen'
    )

    return LaunchDescription(
        [
            ros2_gz_bridge,
            clock_bridge,
            control_bridge_node
        ]
    )











#       (2)         (0)  
#         \         /
#          \   *   /
#           \ _*_ /
#            |   |
#            |___|                                            
#            /   \
#           /     \
#          /       \
#         /         \
#       (1)         (3)


#                (X)      
#                 |
#                 |
#      (Y) _______|
#                /
#               /
#              /
#           (Z)

# X-Y = cw