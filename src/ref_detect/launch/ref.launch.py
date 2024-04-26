from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    ref_detect = Node(
        package="ref_detect",
        executable="ref_detect",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )
    ld = LaunchDescription()
    ld.add_action(ref_detect)
    return ld
