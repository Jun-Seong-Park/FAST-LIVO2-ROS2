"""FAST-LIVO2 모니터링 릴레이 노드 launch.

FAST-LIVO2 본체는 루프백으로 격리하고, 이 monitor 노드만 외부 네트워크로 토픽을
재발행한다. FastDDS 유니캐스트(fastdds_monitor.xml) 로 서버 PC 와 직접 peer 연결한다.
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
