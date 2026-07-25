import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_fastlio_rviz = LaunchConfiguration("launch_fastlio_rviz")
    launch_pgo = LaunchConfiguration("launch_pgo")
    fastdem_config = LaunchConfiguration("fastdem_config")

    livox_share = get_package_share_directory("livox_ros_driver2")
    tf_share = get_package_share_directory("culvert_slam_tf")
    frontend_share = get_package_share_directory("culvert_frontend_mapping")
    submap_share = get_package_share_directory("culvert_submap_manager")
    fastlio_share = get_package_share_directory("fastlio2")
    pgo_share = get_package_share_directory("culvert_pgo")

    fastlio_config = os.path.join(fastlio_share, "config", "lio.yaml")
    fastlio_rviz = os.path.join(fastlio_share, "rviz", "fastlio2.rviz")
    nav2_params = os.path.join(frontend_share, "config", "nav2_mppi_elevation.yaml")
    pgo_params = os.path.join(pgo_share, "config", "culvert_pgo.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_fastlio_rviz", default_value="true"),
            DeclareLaunchArgument("launch_pgo", default_value="true"),
            DeclareLaunchArgument(
                "fastdem_config",
                default_value=os.path.join(frontend_share, "config", "fastdem_local_mapping.yaml"),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(livox_share, "launch_ROS2", "msg_MID360_launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tf_share, "launch", "standard_tf.launch.py")
                ),
                launch_arguments={
                    "map_frame": "map",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "lidar_frame": "lidar",
                    "publish_imu_tf": "false",
                    "publish_map_to_odom_tf": "false",
                }.items(),
            ),
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    {"config_path": fastlio_config},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fastlio2_rviz",
                output="screen",
                arguments=["-d", fastlio_rviz],
                condition=IfCondition(launch_fastlio_rviz),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(frontend_share, "launch", "frontend_elevation_mapping.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "fastdem_config": fastdem_config,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(submap_share, "launch", "submap_manager.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(frontend_share, "launch", "nav2_mppi_navigation.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pgo_share, "launch", "culvert_pgo.launch.py")
                ),
                launch_arguments={
                    "pgo_params_file": pgo_params,
                }.items(),
                condition=IfCondition(launch_pgo),
            ),
        ]
    )
