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
        DeclareLaunchArgument("controller_motion_model", default_value="Omni"),
        DeclareLaunchArgument("controller_vy_max", default_value="0.20"),
        DeclareLaunchArgument("controller_vy_std", default_value="0.12"),
        DeclareLaunchArgument("controller_ay_max", default_value="0.8"),
        DeclareLaunchArgument("controller_ay_min", default_value="-0.8"),
        DeclareLaunchArgument("controller_vx_max", default_value="0.20"),
        DeclareLaunchArgument("controller_vx_std", default_value="0.12"),
        DeclareLaunchArgument("controller_ax_max", default_value="0.8"),
        DeclareLaunchArgument("controller_ax_min", default_value="-0.8"),
        DeclareLaunchArgument(
            "amcl_tf_broadcast",
            default_value="true",
            description="Allow AMCL to publish map->odom; disable for Gazebo truth-localization tests",
        ),
        DeclareLaunchArgument("start_foxglove_bridge", default_value="true"),
        DeclareLaunchArgument(
            "show_gazebo_ground_truth",
            default_value="true",
            description="Publish Gazebo model pose for a separate RViz Pose display",
        ),
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
            package="libot_bringup",
            executable="ground_truth_pose.py",
            name="gazebo_ground_truth_pose",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("show_gazebo_ground_truth")),
            output="screen",
        ),
        TimerAction(period=4.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": "true",
                "amcl_tf_broadcast": LaunchConfiguration("amcl_tf_broadcast"),
                "set_initial_pose": "true",
                "initial_pose_x": LaunchConfiguration("spawn_x"),
                "initial_pose_y": LaunchConfiguration("spawn_y"),
                "initial_pose_yaw": LaunchConfiguration("spawn_yaw"),
                "use_hardware": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "start_workflow_app": "false",
                "autostart_navigation": LaunchConfiguration("autostart_navigation"),
                "controller_motion_model": LaunchConfiguration("controller_motion_model"),
                "controller_vy_max": LaunchConfiguration("controller_vy_max"),
                "controller_vy_std": LaunchConfiguration("controller_vy_std"),
                "controller_ay_max": LaunchConfiguration("controller_ay_max"),
                "controller_ay_min": LaunchConfiguration("controller_ay_min"),
                "controller_vx_max": LaunchConfiguration("controller_vx_max"),
                "controller_vx_std": LaunchConfiguration("controller_vx_std"),
                "controller_ax_max": LaunchConfiguration("controller_ax_max"),
                "controller_ax_min": LaunchConfiguration("controller_ax_min"),
            }.items(),
        )]),
        ExecuteProcess(
            cmd=["python3", "/ros2_ws/robot_app/main.py"],
            additional_env={"LIBOT_RUNTIME_ID": "sim", "LIBOT_TCP_PORT": "9001", "LIBOT_BACKEND_URL": LaunchConfiguration("backend_url")},
            output="screen",
        ),
    ])
