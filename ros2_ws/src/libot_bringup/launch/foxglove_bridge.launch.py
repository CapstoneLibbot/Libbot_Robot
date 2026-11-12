import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    upstream_launch = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )
    topic_whitelist = (
        "['^/tf$', '^/tf_static$', '^/robot_description$', '^/map$', '^/scan$', "
        "'^/odom$', '^/plan$', '^/local_plan$', '^/cmd_vel_nav$', '^/cmd_vel$', "
        "'^/cmd_vel_safe$', '^/speed_limit$', '^/collision_monitor_state$', "
        "'^/stm32_bridge/status$', '^/battery_state$', '^/imu/data$', "
        "'^/range/(front|left|right|rear)$', '^/behavior_tree_log$', "
        "'^/(global_costmap|local_costmap)/.*$', '^/foxglove_bridge/sysinfo$']"
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(upstream_launch),
            launch_arguments={
                "address": "0.0.0.0",
                "port": LaunchConfiguration("port"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "topic_whitelist": topic_whitelist,
                # Monitoring only: do not expose remote topic publishing,
                # parameter access, or ROS service calls through Foxglove.
                "capabilities": "[connectionGraph,assets]",
            }.items(),
        ),
    ])
