import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("culvert_pgo")
    pgo_params_file = LaunchConfiguration("pgo_params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pgo_params_file",
            default_value=os.path.join(pkg_share, "config", "culvert_pgo.yaml"),
        ),
        Node(
            package="culvert_pgo",
            executable="culvert_pgo_node",
            name="culvert_pgo_node",
            output="screen",
            parameters=[pgo_params_file],
        ),
    ])
