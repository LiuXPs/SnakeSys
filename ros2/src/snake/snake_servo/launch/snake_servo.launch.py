import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    heart_pkg_name = 'snake_heart'
    frame_pkg_name = 'snake_frame'
    heart_pkg_dir = get_package_share_directory(heart_pkg_name)
    frame_pkg_dir = get_package_share_directory(frame_pkg_name)
    heart_launch_path = os.path.join(heart_pkg_dir, 'launch', 'snake_heart.launch.py')
    frame_launch_path = os.path.join(frame_pkg_dir, 'launch', 'snake_frame.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    heart_rate = LaunchConfiguration('heart_rate')
    servo_idn = LaunchConfiguration('servo_idn')
    link_length = LaunchConfiguration('link_length')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    timeout = LaunchConfiguration('timeout')

    heart_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(heart_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,

            'heart_rate': heart_rate,
            'servo_idn': servo_idn,
            'link_length': link_length,
        }.items(),
    )

    frame_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frame_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    servo_node = Node(
        package='snake_servo',
        executable='snake_servo_ft',
        name='snake_servo_ft',
        namespace='snake',
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,

            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'timeout': timeout,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use simulation time when true, use actual time when false'))

    ld.add_action(DeclareLaunchArgument(
        'heart_rate',
        default_value='30',
        description='control frequency(unit: Hz)'))

    ld.add_action(DeclareLaunchArgument(
        'servo_idn',
        default_value='28',
        description='number of joints'))

    ld.add_action(DeclareLaunchArgument(
        'link_length',
        # default_value='0.0764',  # snake robot I.
        # default_value='0.1060',  # snake robot II.
        default_value='0.1600',  # snake robot III.
        description='link length of snake robot(unit: m)'))

    ld.add_action(DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='serial port of servo'))

    ld.add_action(DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='baud rate of servo'))

    ld.add_action(DeclareLaunchArgument(
        'timeout',
        default_value='1000',
        description='timeout of servo'))

    ld.add_action(heart_node)
    ld.add_action(servo_node)
    ld.add_action(frame_node)

    return ld
