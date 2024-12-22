import os
import sys
import argparse
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sensor_pkg_name = 'snake_sensor'
    sensor_pkg_dir = get_package_share_directory(sensor_pkg_name)
    cam_launch_path = os.path.join(sensor_pkg_dir, 'launch', 'snake_sensor_cam_usb.launch.py')
    cam_params = os.path.join(sensor_pkg_dir, 'config', 'cam_usb_zed2i_params_slam.yaml')

    usb_cam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_launch_path),
        launch_arguments={
            'cam_params': cam_params,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(usb_cam_node)

    return ld
