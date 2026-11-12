from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    robot_launch = os.path.join(
        get_package_share_directory("libot_bringup"), "launch", "robot.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument("stm32_port", default_value="/dev/serial0"),
        DeclareLaunchArgument("start_workflow_app", default_value="true"),
        DeclareLaunchArgument("robot_app_path", default_value="/ros2_ws/robot_app/main.py"),
        DeclareLaunchArgument("start_foxglove_bridge", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "lidar_port": LaunchConfiguration("lidar_port"),
                "stm32_port": LaunchConfiguration("stm32_port"),
                "start_workflow_app": LaunchConfiguration("start_workflow_app"),
                "robot_app_path": LaunchConfiguration("robot_app_path"),
                "start_foxglove_bridge": LaunchConfiguration("start_foxglove_bridge"),
                "foxglove_port": LaunchConfiguration("foxglove_port"),
            }.items(),
        ),
    ])
