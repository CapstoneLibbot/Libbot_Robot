from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # =========================
    # RPLIDAR A1 launch
    # =========================
    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

    # =========================
    # SLAM Toolbox launch
    # =========================
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    return LaunchDescription([

        # =========================
        # RPLIDAR
        # =========================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'serial_port': '/dev/rplidar',
                'serial_baudrate': '115200',
                'frame_id': 'laser',
            }.items()
        ),

        # =========================
        # STM32
        # =========================
        Node(
            package='libbot_stm32',
            executable='stm32_bridge',
            name='stm32_bridge',
            output='screen',
        ),

        # =========================
        # Static TF
        # =========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', '-0.13',
                '--y', '-0.01',
                '--z', '0.22',
                '--yaw', '1.5708',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ],
            output='screen',
        ),
        # =========================
        # SLAM Toolbox
        # Delay a little so LiDAR + STM32 start first
        # =========================
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch),
                    launch_arguments={
                        'slam_params_file':
                            '/ros2_ws/src/config/mapper_params_online_async.yaml',
                        'use_sim_time': 'false',
                    }.items()
                )
            ]
        ),
    ])