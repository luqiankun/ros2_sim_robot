from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    car_description = get_package_share_directory('car_description')
    xacro_f = os.path.join(car_description,
                           'urdf',
                           'buggy3.urdf.xacro')
    doc = xacro.parse(open(xacro_f))
    xacro.process_doc(doc)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[
            {'use_sim_time': True}, {'robot_description': doc.toxml()},
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        # output='screen',
        # parameters=[{'use_sim_time': True}],
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(car_description, 'rviz', 'rviz.rviz')]
    )
    st = RegisterEventHandler(  # type: ignore
        OnProcessStart(
            target_action=joint_state_publisher,
            on_start=[
                robot_state_publisher
            ]
        )
    )
    ld = LaunchDescription()
    ld.add_action(joint_state_publisher)
    ld.add_action(st)

    # ld.add_action(robot_state_publisher)

    # Add rviz2 node

    ld.add_action(rviz2_node)

    return ld
