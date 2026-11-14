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
        DeclareLaunchArgument("zones_enabled", default_value="false"),
        DeclareLaunchArgument("keepout_mask", default_value=""),
        DeclareLaunchArgument("speed_mask", default_value=""),
        DeclareLaunchArgument("speed_filter_multiplier", default_value="0.01"),
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
            description="Let AMCL publish the map to odom transform",
        ),
        DeclareLaunchArgument(
            "set_initial_pose",
            default_value="false",
            description="Initialize AMCL from the initial_pose_* arguments",
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
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
            "autostart_navigation",
            default_value="false",
            description="Automatically activate the navigation lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "controller_motion_model",
            default_value="Omni",
            description="MPPI model; the Gazebo wheel-contact approximation can use DiffDrive",
        ),
        DeclareLaunchArgument("controller_vy_max", default_value="0.12"),
        DeclareLaunchArgument("controller_vy_std", default_value="0.08"),
        DeclareLaunchArgument("controller_ay_max", default_value="1.5"),
        DeclareLaunchArgument("controller_ay_min", default_value="-1.5"),
        DeclareLaunchArgument("controller_vx_max", default_value="0.30"),
        DeclareLaunchArgument("controller_vx_std", default_value="0.20"),
        DeclareLaunchArgument("controller_ax_max", default_value="1.5"),
        DeclareLaunchArgument("controller_ax_min", default_value="-1.5"),
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
                    "set_initial_pose": ParameterValue(
                        LaunchConfiguration("set_initial_pose"), value_type=bool
                    ),
                    "initial_pose.x": ParameterValue(
                        LaunchConfiguration("initial_pose_x"), value_type=float
                    ),
                    "initial_pose.y": ParameterValue(
                        LaunchConfiguration("initial_pose_y"), value_type=float
                    ),
                    "initial_pose.yaw": ParameterValue(
                        LaunchConfiguration("initial_pose_yaw"), value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="keepout_filter_mask_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("zones_enabled")),
            parameters=[LaunchConfiguration("params_file"), {
                "yaml_filename": LaunchConfiguration("keepout_mask"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            remappings=[("map", "/keepout_filter_mask")],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="speed_filter_mask_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("zones_enabled")),
            parameters=[LaunchConfiguration("params_file"), {
                "yaml_filename": LaunchConfiguration("speed_mask"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            remappings=[("map", "/speed_filter_mask")],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="keepout_costmap_filter_info_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("zones_enabled")),
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "type": 0,
                "filter_info_topic": "/keepout_costmap_filter_info",
                "mask_topic": "/keepout_filter_mask",
                "base": 0.0,
                "multiplier": 1.0,
            }],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="speed_costmap_filter_info_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("zones_enabled")),
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "type": 2,
                "filter_info_topic": "/speed_costmap_filter_info",
                "mask_topic": "/speed_filter_mask",
                "base": 0.0,
                "multiplier": ParameterValue(LaunchConfiguration("speed_filter_multiplier"), value_type=float),
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmap_filters",
            output="screen",
            condition=IfCondition(LaunchConfiguration("zones_enabled")),
            parameters=[{
                "autostart": True,
                "node_names": [
                    "keepout_filter_mask_server",
                    "speed_filter_mask_server",
                    "keepout_costmap_filter_info_server",
                    "speed_costmap_filter_info_server",
                ],
            }],
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
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "FollowPath.motion_model": LaunchConfiguration("controller_motion_model"),
                    "FollowPath.vx_max": ParameterValue(
                        LaunchConfiguration("controller_vx_max"), value_type=float
                    ),
                    "FollowPath.vx_std": ParameterValue(
                        LaunchConfiguration("controller_vx_std"), value_type=float
                    ),
                    "FollowPath.ax_max": ParameterValue(
                        LaunchConfiguration("controller_ax_max"), value_type=float
                    ),
                    "FollowPath.ax_min": ParameterValue(
                        LaunchConfiguration("controller_ax_min"), value_type=float
                    ),
                    "FollowPath.vy_max": ParameterValue(
                        LaunchConfiguration("controller_vy_max"), value_type=float
                    ),
                    "FollowPath.vy_std": ParameterValue(
                        LaunchConfiguration("controller_vy_std"), value_type=float
                    ),
                    "FollowPath.ay_max": ParameterValue(
                        LaunchConfiguration("controller_ay_max"), value_type=float
                    ),
                    "FollowPath.ay_min": ParameterValue(
                        LaunchConfiguration("controller_ay_min"), value_type=float
                    ),
                },
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
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "default_nav_to_pose_bt_xml": os.path.join(
                        bringup_share, "behavior_trees", "navigate_to_pose.xml"
                    ),
                    "default_nav_through_poses_bt_xml": os.path.join(
                        bringup_share, "behavior_trees", "navigate_through_poses.xml"
                    ),
                },
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
                # The web runtime keeps this false and activates Nav2 after
                # AMCL is stable. Standalone clients can opt in to autostart.
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart_navigation"), value_type=bool
                ),
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
