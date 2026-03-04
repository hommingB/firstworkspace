from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("bno085_publisher")
    config    = os.path.join(pkg_share, "config", "bno085_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config,
                              description="Path to YAML parameter file"),

        Node(
            package="bno085_publisher",
            executable="bno085_node",
            name="bno085_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
            remappings=[],
        ),
    ])
