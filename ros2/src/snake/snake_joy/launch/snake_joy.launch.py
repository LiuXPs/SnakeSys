from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace='snake',
        output="both",
        parameters=[{
            'use_sim_time': use_sim_time,

            'device_id': 0,
            'autorepeat_rate': 1.0,
            'deadzone': 0.5,
            'coalesce_interval_ms': 1,
            # 'device_name': "''",
            # 'sticky_buttons': 'false',
        }],
    )
    snake_joy_node = Node(
        package='snake_joy',
        executable='snake_joy',
        name='snake_joy',
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

    ld.add_action(joy_node)
    ld.add_action(snake_joy_node)

    return ld
