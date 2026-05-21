import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fastdds_xml = os.path.join(
        get_package_share_directory("fast_livo2"), "config", "fastdds_jetson.xml")
    appsink_buffers = LaunchConfiguration("appsink_buffers")
    appsink_drop = LaunchConfiguration("appsink_drop")
    diag_sleep_ms = LaunchConfiguration("diag_sleep_ms")

    camera_node = Node(
        package="image_ros2_driver",
        executable="see3cam24cug_trig_hd",
        name="see3cam24cug_trig_hd",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        DeclareLaunchArgument("appsink_buffers", default_value="4"),
        DeclareLaunchArgument("appsink_drop", default_value="true"),
        DeclareLaunchArgument("diag_sleep_ms", default_value="0"),
        SetEnvironmentVariable("SEE3CAM_APPSINK_BUFFERS", appsink_buffers),
        SetEnvironmentVariable("SEE3CAM_APPSINK_DROP", appsink_drop),
        SetEnvironmentVariable("SEE3CAM_DIAG_SLEEP_MS", diag_sleep_ms),
        camera_node,
    ])
