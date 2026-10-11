from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_share = get_package_share_directory("libot_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

    nav2_launch = os.path.join(
        nav2_share,
        "launch",
        "bringup_launch.py",
    )

    default_params = os.path.join(
        bringup_share,
        "config",
        "nav2.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="/ros2_ws/maps/library_map.yaml",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
        ),

        # Phần này cần include robot/STM32/LiDAR launch.
        # Có thể tách phần phần cứng của mapping.launch.py
        # thành robot.launch.py rồi include tại đây.

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": "false",
                "autostart": "true",
                "slam": "false",
            }.items(),
        ),
    ])