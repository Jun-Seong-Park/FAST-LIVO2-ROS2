"""Launch for the FAST-LIVO2 monitoring relay node.

The FAST-LIVO2 core is isolated on loopback, and only this monitor node re-publishes
topics onto the external network. It peers directly with the server PC over FastDDS unicast (fastdds_monitor.xml).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    flivo_pkg = get_package_share_directory("fast_livo2")
    monitor_config = os.path.join(flivo_pkg, "config", "monitor.yaml")
    fastdds_xml = os.path.join(flivo_pkg, "config", "fastdds_monitor.xml")

    monitor_node = Node(
        package="fast_livo2",
        executable="monitor",
        name="fast_livo_monitor",
        output="screen",
        parameters=[monitor_config],
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        monitor_node,
    ])
