import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("libot_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

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
            "amcl_tf_broadcast",
            default_value="true",
            description="Let AMCL publish map to odom (disable when simulation uses ground truth TF)",
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
        DeclareLaunchArgument(
            "start_workflow_app",
            default_value="true",
            description="Start the Python workflow TCP server for backend commands",
        ),
        DeclareLaunchArgument(
            "robot_app_path",
            default_value="/ros2_ws/robot_app/main.py",
            description="Absolute path to robot_app/main.py",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            condition=IfCondition(LaunchConfiguration("use_hardware")),
            launch_arguments={
                "lidar_port": LaunchConfiguration("lidar_port"),
                "stm32_port": LaunchConfiguration("stm32_port"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_workflow_app": LaunchConfiguration("start_workflow_app"),
                "robot_app_path": LaunchConfiguration("robot_app_path"),
            }.items(),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "yaml_filename": LaunchConfiguration("map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "tf_broadcast": ParameterValue(
                        LaunchConfiguration("amcl_tf_broadcast"), value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }],
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
