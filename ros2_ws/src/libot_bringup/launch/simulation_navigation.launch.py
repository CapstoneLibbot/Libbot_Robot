import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("libot_bringup")
    description_share = get_package_share_directory("libot_description")
    layout_file = os.path.join(description_share, "config", "library_layout.json")
    with open(layout_file, encoding="utf-8") as stream:
        robot_spawn = json.load(stream)["robotSpawn"]
    default_world = os.path.join(description_share, "worlds", "library.sdf")
    simulation = os.path.join(bringup_share, "launch", "simulation.launch.py")
    navigation = os.path.join(bringup_share, "launch", "navigation.launch.py")
    collision_monitor = os.path.join(
        bringup_share, "launch", "collision_monitor.launch.py"
    )
    foxglove_bridge = os.path.join(
        bringup_share, "launch", "foxglove_bridge.launch.py"
    )
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="/ros2_ws/maps/library_sim.yaml"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("spawn_x", default_value=str(robot_spawn["x"])),
        DeclareLaunchArgument("spawn_y", default_value=str(robot_spawn["y"])),
        DeclareLaunchArgument("spawn_z", default_value=str(robot_spawn["z"])),
        DeclareLaunchArgument("spawn_yaw", default_value=str(robot_spawn["yaw"])),
        DeclareLaunchArgument("backend_url", default_value="http://127.0.0.1:5205"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("autostart_navigation", default_value="false"),
        DeclareLaunchArgument("start_foxglove_bridge", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "world": LaunchConfiguration("world"),
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(collision_monitor),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(foxglove_bridge),
            launch_arguments={
                "port": LaunchConfiguration("foxglove_port"),
                "use_sim_time": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_foxglove_bridge")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="simulation_map_to_odom",
            arguments=[
                "--x", LaunchConfiguration("spawn_x"),
                "--y", LaunchConfiguration("spawn_y"),
                "--z", "0.0",
                "--yaw", LaunchConfiguration("spawn_yaw"), "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        TimerAction(period=4.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": "true",
                "amcl_tf_broadcast": "false",
                "use_hardware": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "start_workflow_app": "false",
                "autostart_navigation": LaunchConfiguration("autostart_navigation"),
            }.items(),
        )]),
        ExecuteProcess(
            cmd=["python3", "/ros2_ws/robot_app/main.py"],
            additional_env={"LIBOT_RUNTIME_ID": "sim", "LIBOT_TCP_PORT": "9001", "LIBOT_BACKEND_URL": LaunchConfiguration("backend_url")},
            output="screen",
        ),
    ])
