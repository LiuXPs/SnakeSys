import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    frame_pkg_name = 'snake_frame'
    slam_pkg_name = 'snake_slam'
    frame_pkg_dir = get_package_share_directory(frame_pkg_name)
    slam_pkg_dir = get_package_share_directory(slam_pkg_name)
    frame_launch_path = os.path.join(frame_pkg_dir, 'launch', 'snake_frame.launch.py')
    voc_file_path = os.path.join(slam_pkg_dir, 'vocabulary', 'ORBvoc.txt')
    settings_file_path = os.path.join(slam_pkg_dir, 'config', 'stereo_zed2i.yaml')

    frame_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frame_launch_path),
        launch_arguments={
            'use_sim_time': 'False',
        }.items(),
    )

    slam_node = Node(
        package='snake_slam',
        executable='snake_slam_stereo',
        name='snake_slam_stereo',
        namespace='snake',
        output="screen",
        remappings=[
            ('/camera/left/image_raw', '/zed2i/zed_node/left_raw/image_raw_color'),
            ('/camera/right/image_raw', '/zed2i/zed_node/right_raw/image_raw_color'),
            ('/camera/left/image_raw/compressed', '/zed2i/zed_node/left_raw/image_raw_color/compressed'),
            ('/camera/right/image_raw/compressed', '/zed2i/zed_node/right_raw/image_raw_color/compressed'),
            # ('/camera/left/image_raw', '/zed2i/zed_node/left/image_rect_color'),
            # ('/camera/right/image_raw', '/zed2i/zed_node/right/image_rect_color'),
            # ('/camera/left/image_raw/compressed', '/zed2i/zed_node/left/image_rect_color/compressed'),
            # ('/camera/right/image_raw/compressed', '/zed2i/zed_node/right/image_rect_color/compressed'),
        ],
        parameters=[{
            # Parameters for ORB_SLAM3.
            'frame_id_world': 'snake_world',
            'frame_id_base': 'snake_base',
            'frame_id_camera': 'snake_camera_1',

            'camera_pose': 'orb_slam3/camera_pose',
            'map_points': 'orb_slam3/map_points',

            'topic_imu': '/imu',
            'topic_img_mono': '/camera/mono/image_raw',
            'topic_img_left': '/camera/left/image_raw',
            'topic_img_right': '/camera/right/image_raw',
            'topic_img_rgb': '/camera/rgb/image_raw',
            'topic_img_depth': '/camera/depth_registered/image_raw',

            'orb_slam3_voc_file': voc_file_path,
            'orb_slam3_settings_file': settings_file_path,
            'orb_slam3_enable_pangolin': True,
        }],
    )

    ld = LaunchDescription()

    # ld.add_action(frame_node)
    ld.add_action(slam_node)

    return ld
