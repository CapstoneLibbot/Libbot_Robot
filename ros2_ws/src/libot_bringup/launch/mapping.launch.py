#!/usr/bin/env python3
"""Launch SLAM mapping for the LiBot robot.

This launch file starts the components required to build a 2D map with a
2D lidar and the SLAM Toolbox:
- robot_state_publisher using the URDF from libot_description
- STM32 bridge for low-level motor control and odometry
- radar/lidar node (rplidar_ros) publishing /scan
- SLAM Toolbox mapping node
- RViz2 display for the in-progress map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('libot_bringup')
    description_pkg = get_package_share_directory('libot_description')

    urdf_path = os.path.join(description_pkg, 'urdf', 'libot.urdf.xacro')
    slam_params = os.path.join(bringup_pkg, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(bringup_pkg, 'config', 'slam.rviz')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial0', # check xem STM32 la cong nao, neu dung STM32F103C8T6 thi mac dinh la /dev/ttyUSB0
        description='STM32 serial port (for motor control and odometry).'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar', # check xem lidar la cong nao, neu dung RPLIDAR thi mac dinh la /dev/ttyUSB0
        description='Lidar serial port. For RPLIDAR this is usually /dev/ttyUSB0.'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2 for map visualization.'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    stm32_bridge = Node(
        package='libot_stm32',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('serial_port'),
            'baudrate': 115200,
        }]
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_ros2',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        use_rviz_arg,
        robot_state_publisher,
        stm32_bridge,
        lidar_node,
        slam_node,
        rviz2_node,
    ])
