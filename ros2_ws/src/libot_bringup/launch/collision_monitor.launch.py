import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the final velocity safety layer independently from Nav2."""

    default_params = os.path.join(
        get_package_share_directory("libot_bringup"),
        "config",
        "collision_monitor.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Collision Monitor parameter file",
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                params_file,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": True,
                "node_names": ["collision_monitor"],
                "bond_timeout": 4.0,
                "attempt_respawn_reconnection": True,
            }],
        ),
    ])
