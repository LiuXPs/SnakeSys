from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    heart_rate = LaunchConfiguration('heart_rate')
    servo_idn = LaunchConfiguration('servo_idn')
    link_length = LaunchConfiguration('link_length')

    heart_node = Node(
        package='snake_heart',
        executable='snake_heart',
        name='snake_heart',
        namespace='snake',
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,

            'heart_rate': heart_rate,
            'servo_idn': servo_idn,
            'link_length': link_length,
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

    ld.add_action(heart_node)

    return ld
