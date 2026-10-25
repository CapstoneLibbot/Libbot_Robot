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
                "gz_args": ["-s -r -v 4 ", world_file],
                "on_exit_shutdown": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("headless")),
        ),
        IncludeLaunchDescription(
            gz_launch,
            launch_arguments={
                "gz_args": ["-r -v 4 ", world_file],
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
                "-x", str(robot_spawn["x"]),
                "-y", str(robot_spawn["y"]),
                "-z", str(robot_spawn["z"]),
                "-Y", str(robot_spawn["yaw"]),
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
