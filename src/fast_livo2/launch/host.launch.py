"""서버 PC(host) 전용 launch.

Jetson 의 monitor 노드가 /orin22/* 로 재발행한 토픽을 수신해 RViz 로 시각화한다.

환경 변수:
  FASTRTPS_DEFAULT_PROFILES_FILE — fastdds_server.xml (unicast peer 설정)

토픽 remapping:
  monitor 노드가 /<robot_name>/* 로 재발행한 토픽을 RViz2 노드 remapping 으로 원래 이름으로 되돌린다.
  대상: /tf, /tf_static, /cloud_registered, /aft_mapped_to_init, /path, /rgb_img
  robot_name 이 바뀌면 launch argument 로 오버라이드한다.
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
        description="Jetson 의 robot_name (monitor.yaml 과 일치해야 함)",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="RViz2 실행 여부",
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
