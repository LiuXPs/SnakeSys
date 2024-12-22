from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ahrs_driver = Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        name="snake_imu",
        namespace='snake',
        output="screen",
        parameters=[{
            'if_debug_': False,
            'serial_port_': '/dev/ttyUSB0',
            'serial_baud_': 921600,
            'imu_topic': 'sensor/imu_n100/imu',
            'imu_frame_id_': 'gyro_link',
            'mag_pose_2d_topic': 'sensor/imu_n100/mag_pose_2d',
            'Magnetic_topic': 'sensor/imu_n100/magnetic',
            'Euler_angles_topic': 'sensor/imu_n100/euler_angles',
            'gps_topic': 'sensor/imu_n100/gps/fix',
            'twist_topic': 'sensor/imu_n100/system_speed',
            'NED_odom_topic': 'sensor/imu_n100/NED_odometry',
            'device_type_': 1,
        }]
    )

    imu_tf = Node(
        package="fdilink_ahrs",
        executable="imu_tf_node",
        name='snake_imu_tf',
        namespace='snake',
        output="screen",
        parameters=[{
            'imu_topic': 'sensor/imu_n100/imu',
            'world_frame_id': '/world',
            'imu_frame_id': '/gyro_link',
            'position_x': 1,
            'position_y': 1,
            'position_z': 1,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(ahrs_driver)
    ld.add_action(imu_tf)

    return ld
