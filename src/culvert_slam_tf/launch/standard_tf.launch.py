from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    imu_frame = LaunchConfiguration("imu_frame")
    optimized_pose_topic = LaunchConfiguration("optimized_pose_topic")
    publish_imu_tf = LaunchConfiguration("publish_imu_tf")
    publish_map_to_odom_tf = LaunchConfiguration("publish_map_to_odom_tf")

    body_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_lidar_tf",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", base_frame,
            "--child-frame-id", lidar_frame,
        ],
    )

    body_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_imu_tf",
        condition=IfCondition(publish_imu_tf),
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", base_frame,
            "--child-frame-id", imu_frame,
        ],
    )

    map_to_odom = Node(
        package="culvert_slam_tf",
        executable="map_to_odom_tf_node",
        name="map_to_odom_tf",
        output="screen",
        condition=IfCondition(publish_map_to_odom_tf),
        parameters=[
            {
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "optimized_pose_topic": optimized_pose_topic,
                "identity_until_optimized": True,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="lidar"),
        DeclareLaunchArgument("imu_frame", default_value="imu"),
        DeclareLaunchArgument("publish_imu_tf", default_value="false"),
        DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
        DeclareLaunchArgument("optimized_pose_topic", default_value="/optimized_pose"),
        body_to_lidar,
        body_to_imu,
        map_to_odom,
    ])
