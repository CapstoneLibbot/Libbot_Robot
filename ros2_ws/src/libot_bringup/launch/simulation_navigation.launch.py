import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("libot_bringup")
    description_share = get_package_share_directory("libot_description")
    layout_file = os.path.join(description_share, "config", "library_layout.json")
    with open(layout_file, encoding="utf-8") as stream:
        robot_spawn = json.load(stream)["robotSpawn"]
    simulation = os.path.join(bringup_share, "launch", "simulation.launch.py")
    navigation = os.path.join(bringup_share, "launch", "navigation.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="/ros2_ws/maps/library_sim.yaml"),
        DeclareLaunchArgument("backend_url", default_value="http://127.0.0.1:5205"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(simulation)),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="simulation_map_to_odom",
            arguments=[
                "--x", str(robot_spawn["x"]),
                "--y", str(robot_spawn["y"]),
                "--z", "0.0",
                "--yaw", str(robot_spawn["yaw"]), "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        TimerAction(period=4.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={"map": LaunchConfiguration("map"), "use_sim_time": "true", "amcl_tf_broadcast": "false", "use_hardware": "false", "use_rviz": LaunchConfiguration("use_rviz"), "start_workflow_app": "false"}.items(),
        )]),
        ExecuteProcess(
            cmd=["python3", "/ros2_ws/robot_app/main.py"],
            additional_env={"LIBOT_RUNTIME_ID": "sim", "LIBOT_TCP_PORT": "9001", "LIBOT_BACKEND_URL": LaunchConfiguration("backend_url")},
            output="screen",
        ),
    ])
