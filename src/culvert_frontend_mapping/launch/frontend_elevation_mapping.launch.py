import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frontend_share = get_package_share_directory("culvert_frontend_mapping")

    fastdem_config = LaunchConfiguration("fastdem_config")
    input_scan = LaunchConfiguration("input_scan")
    traversability_config = LaunchConfiguration("traversability_config")
    global_map_config = LaunchConfiguration("global_map_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "fastdem_config",
            default_value=os.path.join(frontend_share, "config", "fastdem_global_mapping.yaml"),
        ),
        DeclareLaunchArgument(
            "input_scan",
            default_value="/fastlio2/body_cloud",
        ),
        DeclareLaunchArgument(
            "traversability_config",
            default_value=os.path.join(frontend_share, "config", "traversability_to_map.yaml"),
        ),
        DeclareLaunchArgument(
            "global_map_config",
            default_value=os.path.join(frontend_share, "config", "online_global_map_builder.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="fastdem_ros2",
            executable="fastdem_node",
            name="fastdem",
            output="screen",
            parameters=[
                {"config_file": fastdem_config},
                {"input_scan": input_scan},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="culvert_frontend_mapping",
            executable="traversability_to_map.py",
            name="traversability_to_map",
            output="screen",
            parameters=[traversability_config],
        ),
        Node(
            package="culvert_frontend_mapping",
            executable="online_global_map_builder.py",
            name="online_global_map_builder",
            output="screen",
            parameters=[
                global_map_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
