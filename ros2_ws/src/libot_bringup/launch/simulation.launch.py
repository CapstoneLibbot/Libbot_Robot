import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("libot_description")
    bringup_share = get_package_share_directory("libot_bringup")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    urdf_file = os.path.join(
        description_share, "urdf", "libot.urdf.xacro"
    )
    world_file = os.path.join(
        description_share, "worlds", "library.sdf"
    )
    bridge_file = os.path.join(
        bringup_share, "config", "ros_gz_bridge.yaml"
    )

    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": ["-r -v 4 ", world_file],
                "on_exit_shutdown": "true",
            }.items(),
        ),

        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "libot",
                "-topic", "robot_description",
                "-x", "0",
                "-y", "0",
                "-z", "0.15",
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