from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_path = get_package_share_directory(
        'libot_description'
    )

    urdf_file = os.path.join(
        pkg_path,
        'urdf',
        'libot.urdf.xacro'
    )

    robot_description = Command([
        'xacro ',
        urdf_file
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    robot_description,
                    value_type=str,
                )
            }
        ]
    )

    return LaunchDescription([
        robot_state_publisher
    ])
