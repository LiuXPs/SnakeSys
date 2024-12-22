import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gui_pkg_name = 'snake_gui'
    gui_pkg_dir = get_package_share_directory(gui_pkg_name)
    rviz_tf_path = os.path.join(gui_pkg_dir, 'rviz', 'rviz_tf.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='snake',
        respawn="false",
        output="screen",
        arguments=[
            '-d', rviz_tf_path,
        ],
        parameters=[{
        }],
    )

    ld = LaunchDescription()
    ld.add_action(rviz_node)

    return ld
