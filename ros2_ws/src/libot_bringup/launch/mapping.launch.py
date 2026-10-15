from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_share = get_package_share_directory('libot_bringup')
    description_share = get_package_share_directory('libot_description')

    # =========================
    # SLAM Toolbox launch
    # =========================
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    description_launch = os.path.join(
        description_share,
        'launch',
        'description.launch.py'
    )

    slam_params_file = os.path.join(
        bringup_share,
        'config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_hardware',
            default_value='true',
            description='Start robot sensors and base; set false when robot_control.launch.py owns hardware.',
        ),

        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/rplidar',
            description='Serial device used by the RPLIDAR'
        ),
        DeclareLaunchArgument(
            'stm32_port',
            default_value='/dev/serial0',
            description='Serial device used by the STM32 bridge'
        ),

        # Publish the fixed URDF chain exactly once:
        # base_footprint -> base_link -> laser.
        GroupAction(condition=IfCondition(LaunchConfiguration('use_hardware')), actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch)),
            Node(
                package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
                output='screen', respawn=True, respawn_delay=3.0,
                parameters=[{
                    'channel_type': 'serial', 'serial_port': LaunchConfiguration('lidar_port'),
                    'serial_baudrate': 115200, 'frame_id': 'laser', 'inverted': False,
                    'angle_compensate': True,
                }],
            ),
            Node(
                package='libbot_stm32', executable='stm32_bridge', name='stm32_bridge',
                output='screen',
                parameters=[{'port': LaunchConfiguration('stm32_port'), 'baudrate': 115200, 'use_sim_time': False}],
            ),
        ]),

        # =========================
        # SLAM Toolbox
        # Delay a little so LiDAR + STM32 start first
        # =========================
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch),
                    launch_arguments={
                        'slam_params_file': slam_params_file,
                        'use_sim_time': 'false',
                    }.items()
                )
            ]
        ),
    ])
