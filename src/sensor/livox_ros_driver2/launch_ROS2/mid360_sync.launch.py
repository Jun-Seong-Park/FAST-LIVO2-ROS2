import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("livox_ros_driver2")
    flivo_pkg  = get_package_share_directory("fast_livo2")
    default_params = os.path.join(pkg_share, "config", "mid360_sync.yaml")
    default_lidar_cfg = os.path.join(pkg_share, "config", "MID360_config.json")
    fastdds_xml = os.path.join(flivo_pkg, "config", "fastdds_jetson.xml")

    params_file = LaunchConfiguration("params_file")
    lidar_cfg = LaunchConfiguration("lidar_cfg")

    lidar_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[params_file, {"user_config_path": lidar_cfg}],
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("lidar_cfg", default_value=default_lidar_cfg),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        lidar_node,
    ])
