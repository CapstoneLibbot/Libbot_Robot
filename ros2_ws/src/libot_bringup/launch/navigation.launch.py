import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("libot_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

    nav2_launch = os.path.join(
        nav2_share,
        "launch",
        "bringup_launch.py",
    )
    robot_launch = os.path.join(
        bringup_share,
        "launch",
        "robot.launch.py",
    )
    rviz_config = os.path.join(
        nav2_share,
        "rviz",
        "nav2_default_view.rviz",
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
            description="Absolute path to the saved map YAML file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Absolute path to the Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/rplidar",
        ),
        DeclareLaunchArgument(
            "stm32_port",
            default_value="/dev/serial0",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="true",
            description="Start robot description, LiDAR and STM32 bridge",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the standard Nav2 view",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            condition=IfCondition(LaunchConfiguration("use_hardware")),
            launch_arguments={
                "lidar_port": LaunchConfiguration("lidar_port"),
                "stm32_port": LaunchConfiguration("stm32_port"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": "true",
                "slam": "false",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", rviz_config],
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
