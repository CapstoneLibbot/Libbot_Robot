import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the physical robot interfaces required by localization/Nav2."""

    description_launch = os.path.join(
        get_package_share_directory("libot_description"),
        "launch",
        "description.launch.py",
    )

    lidar_port = LaunchConfiguration("lidar_port")
    stm32_port = LaunchConfiguration("stm32_port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_workflow_app = LaunchConfiguration("start_workflow_app")
    robot_app_path = LaunchConfiguration("robot_app_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/rplidar",
            description="Serial device used by the RPLIDAR",
        ),
        DeclareLaunchArgument(
            "stm32_port",
            default_value="/dev/serial0",
            description="Serial device used by the STM32 bridge",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "start_workflow_app",
            default_value="true",
            description="Start the Python workflow TCP server for backend commands",
        ),
        DeclareLaunchArgument(
            "robot_app_path",
            default_value="/ros2_ws/robot_app/main.py",
            description="Absolute path to robot_app/main.py",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
        ),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="libbot_stm32",
            executable="stm32_bridge",
            name="stm32_bridge",
            output="screen",
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                "port": stm32_port,
                "baudrate": 115200,
                "use_sim_time": use_sim_time,
            }],
        ),
        ExecuteProcess(
            cmd=[sys.executable, robot_app_path],
            name="libot_workflow_app",
            output="screen",
            condition=IfCondition(start_workflow_app),
        ),
    ])
