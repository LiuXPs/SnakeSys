import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # frame_pkg_name = 'snake_frame'
    # gui_pkg_name = 'snake_gui'
    # joy_pkg_name = 'snake_joy'
    # frame_pkg_dir = get_package_share_directory(frame_pkg_name)
    # gui_pkg_dir = get_package_share_directory(gui_pkg_name)
    # joy_pkg_dir = get_package_share_directory(joy_pkg_name)
    # frame_launch_path = os.path.join(frame_pkg_dir, 'launch', 'snake_frame.launch.py')
    # gui_launch_path = os.path.join(gui_pkg_dir, 'launch', 'snake_gui.launch.py')
    # joy_launch_path = os.path.join(joy_pkg_dir, 'launch', 'snake_joy.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # frame_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(frame_launch_path),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )
    #
    # gui_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gui_launch_path),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )
    #
    # joy_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(joy_launch_path),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )

    cpg_hopf_node = Node(
        package='snake_cpg',
        executable='snake_cpg_hopf',
        name='snake_cpg_hopf',
        namespace='snake',
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,

            'cpg_alpha_y': 0.01,
            'cpg_alpha_p': 0.01,

            'cpg_kg_y': 1.0,
            'cpg_kg_p': 1.0,

            'cpg_lambda_y': 0.2,
            'cpg_lambda_p': 0.2,

            'cpg_sigma_y': 1.0,
            'cpg_sigma_p': 1.0,

            'cpg_uvc_y': 0.0,
            'cpg_uvc_p': 0.0,

            'cpg_rho_y': 0.1,
            'cpg_rho_p': 0.1,

            'cpg_omega_y': 0.1,
            'cpg_omega_p': 0.1,

            'cpg_phi_y': 0.1,
            'cpg_phi_p': 0.1,
            'cpg_phi_yp': 0.5,

            'cpg_topo_mode': 1,
            'cpg_weight_mode': 1,
            'cpg_motion_mode': 1,
            'cpg_couple_mode': 1,
            'cpg_mode': 1,

            'cpg_gauss_mu': 1.0,
            'cpg_gauss_sigma': 5.0,

            'cpg_wave_kn': 2.0,
            'cpg_wave_ay': 1.0,
            'cpg_wave_ap': 0.1,

            'cpg_arc_r': 1.0,

            'cpg_spiral_r': 0.26,
            'cpg_spiral_p': 0.04,

            'cpg_start': False,
            'cpg_change': True,
            'cpg_torque': False,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use simulation time when true, use actual time when false'))

    ld.add_action(cpg_hopf_node)
    # ld.add_action(frame_node)
    # ld.add_action(gui_node)
    # ld.add_action(joy_node)

    return ld
