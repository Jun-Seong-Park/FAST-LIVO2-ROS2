import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # PC (host) side: subscribe to the Jetson's compressed image stream and report
    # receive rate / bandwidth. Uses the PC FastDDS profile (self=100.88.16.124,
    # peer=Jetson 100.88.16.123). Requires Tailscale DOWN on both ends.
    config_dir   = os.path.join(get_package_share_directory("econ_ros2_driver"), "config")
    fastdds_path = os.path.join(config_dir, "fastdds_unicast_pc.xml")

    monitor = Node(
        package="econ_ros2_driver",
        executable="image_monitor",
        name="image_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[{"topic": "/camera/image/compressed"}],
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_path),
        monitor,
    ])
