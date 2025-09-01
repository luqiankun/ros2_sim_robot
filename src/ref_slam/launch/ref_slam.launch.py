from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    description = get_package_share_directory('ref_slam')
    param_file=os.path.join(description,"config","config.yaml")
    # print(param_file)
    return LaunchDescription([
        Node(
            package='ref_slam',
            executable='ref_slam_node',
            name='ref_slam_node',
            output='screen',
            parameters=[param_file]
        )
    ])