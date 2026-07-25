import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("culvert_submap_manager")
    submap_params_file = LaunchConfiguration("submap_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "submap_params_file",
                default_value=os.path.join(pkg_share, "config", "submap_manager.yaml"),
                description="Path to submap manager parameters.",
            ),
            Node(
                package="culvert_submap_manager",
                executable="submap_manager_node.py",
                name="submap_manager",
                output="screen",
                parameters=[submap_params_file],
            ),
        ]
    )
