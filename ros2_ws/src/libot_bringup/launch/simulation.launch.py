import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("libot_description")
    bringup_share = get_package_share_directory("libot_bringup")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    urdf_file = os.path.join(
        description_share, "urdf", "libot.urdf.xacro"
    )
    layout_file = os.path.join(description_share, "config", "library_layout.json")
    with open(layout_file, encoding="utf-8") as stream:
        robot_spawn = json.load(stream)["robotSpawn"]
    world_file = os.path.join(description_share, "worlds", "library.sdf")
    bridge_file = os.path.join(
        bringup_share, "config", "ros_gz_bridge.yaml"
    )

    robot_description = Command(["xacro ", urdf_file])

    gz_launch = PythonLaunchDescriptionSource(
        os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo without its GUI when true",
        ),
        DeclareLaunchArgument("world", default_value=world_file),
        DeclareLaunchArgument("spawn_x", default_value=str(robot_spawn["x"])),
        DeclareLaunchArgument("spawn_y", default_value=str(robot_spawn["y"])),
        DeclareLaunchArgument("spawn_z", default_value=str(robot_spawn["z"])),
        DeclareLaunchArgument("spawn_yaw", default_value=str(robot_spawn["yaw"])),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description,
                    value_type=str,
                ),
                "use_sim_time": True,
            }],
            output="screen",
        ),

        IncludeLaunchDescription(
            gz_launch,
            launch_arguments={
                "gz_args": ["-s -r -v 4 ", LaunchConfiguration("world")],
                "on_exit_shutdown": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("headless")),
        ),
        IncludeLaunchDescription(
            gz_launch,
            launch_arguments={
                "gz_args": ["-r -v 4 ", LaunchConfiguration("world")],
                "on_exit_shutdown": "true",
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("headless")),
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "libot",
                "-topic", "robot_description",
                "-x", LaunchConfiguration("spawn_x"),
                "-y", LaunchConfiguration("spawn_y"),
                "-z", LaunchConfiguration("spawn_z"),
                "-Y", LaunchConfiguration("spawn_yaw"),
            ],
            output="screen",
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{
                "config_file": bridge_file,
            }],
            output="screen",
        ),
    ])
