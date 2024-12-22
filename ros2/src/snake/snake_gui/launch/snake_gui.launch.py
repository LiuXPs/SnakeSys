from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    gui_node = Node(
        package='snake_gui',
        executable='snake_gui',
        name='snake_gui',
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

    ld.add_action(gui_node)

    return ld
