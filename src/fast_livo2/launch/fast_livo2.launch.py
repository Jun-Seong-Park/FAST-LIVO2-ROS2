"""
FAST-LIVO2 only launch file.

This launch starts only the FAST-LIVO2 mapping node. Camera, LiDAR, and IMU
publishers must be launched separately. In only Jetson
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    flivo_pkg = get_package_share_directory("fast_livo2")
    rviz_config = os.path.join(flivo_pkg, "rviz_cfg", "fast_livo2.rviz")
    flivo_config = os.path.join(flivo_pkg, "config", "mid360_24cug.yaml")
    fastdds_xml = os.path.join(flivo_pkg, "config", "fastdds_jetson.xml")

    flivo_src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    os.makedirs(os.path.join(flivo_src_dir, "Log", "pcd"), exist_ok=True)

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        choices=["true", "false"],
        description="Launch RViz2 alongside the FAST-LIVO2 node",
    )

    fast_livo2_node = Node(
        package="fast_livo2",
        executable="fastlivo_mapping",
        name="fast_livo2_node",
        output="screen",
        parameters=[flivo_config],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        rviz_arg,
        fast_livo2_node,
        rviz2_node
    ])
