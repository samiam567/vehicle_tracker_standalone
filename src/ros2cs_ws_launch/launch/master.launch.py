from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
import os

def generate_launch_description():


   

    ros2cs_talker = Node(
        package="dotnet_hello_world",
        executable="ros2cs_talker",
        output="screen",
    )

    ros2cs_listener = Node(
        package="dotnet_hello_world",
        executable="ros2cs_listener",
        output="screen",
    )

    vehicle_tracker = Node(
        package='multivehicle_awareness',
        executable='vehicle_tracker',
        output='screen',
    )

    launch_description = [
        ros2cs_talker,
        ros2cs_listener,
        vehicle_tracker,
    ]

    return LaunchDescription(launch_description)
