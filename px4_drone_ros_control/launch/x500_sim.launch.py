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
    pkg_project_drone = get_package_share_directory("px4_drone_ros_control")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Retrieve the sdf file name from the launch argument
    sdf_file_name = LaunchConfiguration("sdf")

    # Use sdf_file_name in your code as needed
    world_file = PathJoinSubstitution([pkg_project_drone, 'worlds', sdf_file_name])

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],  # Use the world_file with the sdf argument
        # cmd=['gz', 'sim', world_file],  # Use the world_file with the sdf argument
        output='screen'
    )

    ros2_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_drone, 'config', 'ros_gz_bridge_direct.yaml'),
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sdf", default_value='default.sdf', description="sdf file name to open"
            ),
            gz_sim,
            ros2_gz_bridge,
            bridge,
        ]
    )
    