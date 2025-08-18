from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    car_description = get_package_share_directory('car_description')
    xacro_f = os.path.join(car_description,
                           'urdf',
                           'buggy3.urdf.xacro')
    world = os.path.join(car_description, 'world', 'empty.sdf')
    doc = xacro.parse(open(xacro_f))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    xacro_file = PathJoinSubstitution(
        [car_description, 'urdf', 'car.urdf.xacro'])
    config_file = PathJoinSubstitution(
        [car_description, 'config', 'car.yaml'])
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[
            {'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()},
        ]
    )
    # joint_state_publisher_viz = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    ref_detect = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ref_detect'), 'launch', 'ref.launch.py')]))

    ign_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output="screen",
        arguments=['-string', doc.toxml(), '-name', "car",
                   '-allow-renaming', 'true', '-z', '0.3'],
    )
    driver = Node(
        package='car_description',
        executable='driver_node',
        name='driver_motor_node',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}, {"model": world}],
    )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        output='screen'
    )
    # controller = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[config_file],
    #     remappings=[
    #         ("~/robot_description", "/robot_description"),
    #     ]
    # )
    # event
    spawn_driver = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[driver]
        )
    )

    move_ctrl=Node(
        package="move_ctrl",
        executable="move_node",
    )

    spawn_joint_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=driver,
            on_start=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller',
                         '--set-state', 'active', 'velocity_controller'],
                    output='screen'

                ), ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller',
                         '--set-state', 'active', 'position_controller'],
                    output='screen'
                ), ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                         'joint_state_broadcaster'],
                    output='screen',
                )]
        )
    )
    spawn_robot_state_publisher = RegisterEventHandler(
        OnProcessStart(
            target_action=ign_spawn,
            on_start=[robot_state_publisher]
        )
    )
    spawn_ign = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge,
            on_start=[ign_spawn]
        )
    )
    ld = LaunchDescription()
    ld.add_action(bridge)
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [' -r -v 1 ', world])]),)
    ld.add_action(spawn_robot_state_publisher)
    # ld.add_action(spawn_joint_controller)
    ld.add_action(spawn_ign)

    ld.add_action(spawn_driver)
    ld.add_action(move_ctrl)

    # ld.add_action(ref_detect)
    # ld.add_action(robot_state_publisher)

    return ld
