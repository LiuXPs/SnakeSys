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

    heart_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(heart_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    frame_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frame_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    sim_node = Node(
        package='snake_sim',
        executable='snake_sim',
        name='snake_sim',
        namespace='snake',
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use simulation time when true, use actual time when false'))

    ld.add_action(heart_node)
    ld.add_action(sim_node)
    ld.add_action(frame_node)

    return ld
