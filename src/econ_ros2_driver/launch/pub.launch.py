import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    econ_driver_path = get_package_share_directory("econ_ros2_driver")
    config_dir   = os.path.join(econ_driver_path, "config")
    config_path  = os.path.join(config_dir, "config.yaml")

    with open(config_path) as f:
        params = yaml.safe_load(f)["econ_ros2_driver"]["ros__parameters"]

    localhost_only = params["localhost_only"]
    jetson_xml = params["fastdds_jetson_xml"]
    loopback_xml = params["fastdds_loopback_xml"]

    fastdds_xml    = loopback_xml if localhost_only else jetson_xml

    fastdds_path  = os.path.join(config_dir, fastdds_xml)
    localhost_env = "1" if localhost_only else "0"

    camera_node = Node(
        package="econ_ros2_driver",
        executable="econ_ros2_driver",
        name="econ_ros2_driver",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", localhost_env),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_path),
        camera_node,
    ])
