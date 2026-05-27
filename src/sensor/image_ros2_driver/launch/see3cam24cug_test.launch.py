import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    fastdds_xml = os.path.join(
        get_package_share_directory("fast_livo2"), "config", "fastdds_jetson.xml")

    camera_node = Node(
        package="image_ros2_driver",
        executable="see3cam24cug_test",
        name="see3cam24cug_test",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        camera_node,
    ])
