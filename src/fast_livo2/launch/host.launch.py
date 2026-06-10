"""Launch dedicated to the server PC (host).

Receives the topics that the Jetson monitor node re-publishes under /orin22/* and visualizes them in RViz.

Environment variables:
  FASTRTPS_DEFAULT_PROFILES_FILE — fastdds_server.xml (unicast peer config)

Topic remapping:
  The RViz2 node remaps the topics the monitor node re-published under /<robot_name>/* back to their original names.
  Targets: /tf, /tf_static, /cloud_registered, /aft_mapped_to_init, /path, /rgb_img
  If robot_name changes, override it via a launch argument.
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
    rviz_config = os.path.join(flivo_pkg, "rviz_cfg", "fast_livo2_host.rviz")
    fastdds_xml = os.path.join(flivo_pkg, "config", "fastdds_host.xml")

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="orin22",
        description="Jetson robot_name (must match monitor.yaml)",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Whether to launch RViz2",
    )

    robot_name = LaunchConfiguration("robot_name")

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node",
        output="screen",
        arguments=["-d", rviz_config],
        remappings=[
            ("/tf",                ["/", robot_name, "/tf"]),
            ("/tf_static",         ["/", robot_name, "/tf_static"]),
            ("/cloud_registered",  ["/", robot_name, "/cloud_registered"]),
            ("/aft_mapped_to_init",["/", robot_name, "/aft_mapped_to_init"]),
            ("/path",              ["/", robot_name, "/path"]),
            ("/rgb_img/compressed", ["/", robot_name, "/rgb_img/compressed"]),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        robot_name_arg,
        rviz_arg,
        rviz2_node,
    ])
