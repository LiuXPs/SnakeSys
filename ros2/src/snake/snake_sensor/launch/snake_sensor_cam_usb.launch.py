import os
import sys
import argparse
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cam_params = LaunchConfiguration('cam_params')
    use_sim_time = LaunchConfiguration('use_sim_time')

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')

    args, unknown = parser.parse_known_args(sys.argv[4:])
    node_name = args.node_name
    print(cam_params)

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=node_name,
        namespace='snake',
        output='screen',
        parameters=[cam_params],
    )

    usb_cam_show_node = Node(
        package='usb_cam',
        executable='show_image.py',
        name='show_image',
        namespace='snake',
        output='screen',
        # arguments=[image_manip_dir + "/data/mosaic.jpg"])
        # remappings=[('image_in', 'sensor/usb_cam/image_raw')]
    )

    usb_cam_segment_node = Node(
        package='snake_sensor',
        executable='snake_sensor_usb_cam',
        name='snake_sensor_usb_cam',
        namespace='snake',
        output="screen",
        parameters=[{
            'topic_img_stereo': 'sensor/usb_cam/image_raw',
            'topic_img_left': 'sensor/usb_cam/image_left',
            'topic_img_right': 'sensor/usb_cam/image_right',
            'use_sim_time': False,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'cam_params',
        description='usb camera params'))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use simulation time when true, use actual time when false'))

    ld.add_action(usb_cam_node)
    # ld.add_action(usb_cam_show_node)
    ld.add_action(usb_cam_segment_node)

    return ld
