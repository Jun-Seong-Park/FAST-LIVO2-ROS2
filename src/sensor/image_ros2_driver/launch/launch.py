import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fastdds_xml = os.path.join(
        get_package_share_directory("fast_livo2"), "config", "fastdds_jetson.xml")

    driver_share = get_package_share_directory("image_ros2_driver")
    config_path  = os.path.join(driver_share, "config", "config.yaml")

    # resolution profile selector (default sd). Overrides resolution in config.yaml.
    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="sd",
        choices=["sd", "hd", "fhd", "wuxga"],
        description="Camera resolution profile (sd | hd | fhd | wuxga).",
    )
    resolution = LaunchConfiguration("resolution")

    # publish format selector (default raw). true → MJPG CompressedImage, false → raw Image.
    compressed_arg = DeclareLaunchArgument(
        "compressed",
        default_value="false",
        choices=["true", "false"],
        description="Publish camera MJPG as CompressedImage instead of raw bgr8 Image.",
    )
    compressed = ParameterValue(LaunchConfiguration("compressed"), value_type=bool)

    camera_node = Node(
        package="image_ros2_driver",
        executable="see3cam24cug_trig",
        name="see3cam24cug_trig",
        output="screen",
        emulate_tty=True,
        parameters=[config_path, {"resolution": resolution, "compressed": compressed}],
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        resolution_arg,
        compressed_arg,
        camera_node,
    ])

# ros2 launch image_ros2_driver launch.py resolution:=sd                  (sd | hd | fhd | wuxga)
# ros2 launch image_ros2_driver launch.py resolution:=fhd compressed:=true  (MJPG CompressedImage)
