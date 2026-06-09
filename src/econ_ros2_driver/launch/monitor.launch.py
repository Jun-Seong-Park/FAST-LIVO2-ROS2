import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_monitor_node(encoding):
    # econ_monitor = src/image_viewer.cpp. encoding 에 따라 raw(Image) 또는
    # compressed(CompressedImage) 를 best_effort 로 구독해 화면(cv::imshow)에 띄우고
    # 수신 fps/Mbps 를 로깅한다. topic 기본값은 노드가 encoding 에 맞춰 고른다.
    return Node(
        package="econ_ros2_driver",
        executable="econ_monitor",
        name="econ_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[{"encoding": encoding}],
    )


def launch_setup(context, *args, **kwargs):
    # client 인자를 실제 문자열로 푼다: local = Jetson 내부 loopback / server = 외부 PC.
    client = LaunchConfiguration("client").perform(context)

    valid_clients = ("local", "server")
    if client not in valid_clients:
        raise RuntimeError(f"client must be one of {valid_clients}, got '{client}'")

    is_local = client == "local"

    # encoding 인자를 푼다: raw = sensor_msgs/Image / compressed = CompressedImage.
    encoding = LaunchConfiguration("encoding").perform(context)

    valid_encodings = ("raw", "compressed")
    if encoding not in valid_encodings:
        raise RuntimeError(f"encoding must be one of {valid_encodings}, got '{encoding}'")

    # FastDDS xml 이름은 모두 config.yaml 에 있으므로 거기서 읽는다.
    econ_driver_path = get_package_share_directory("econ_ros2_driver")
    config_dir  = os.path.join(econ_driver_path, "config")
    config_path = os.path.join(config_dir, "config.yaml")

    with open(config_path) as f:
        params = yaml.safe_load(f)["econ_ros2_driver"]["ros__parameters"]

    loopback_xml = params["fastdds_loopback_xml"]
    pc_xml       = params["fastdds_pc_xml"]

    fastdds_xml  = loopback_xml if is_local else pc_xml
    fastdds_path = os.path.join(config_dir, fastdds_xml)

    localhost_env = "1" if is_local else "0"

    set_localhost = SetEnvironmentVariable("ROS_LOCALHOST_ONLY", localhost_env)
    set_fastdds   = SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_path)

    return [set_localhost, set_fastdds, make_monitor_node(encoding)]


def generate_launch_description():
    client_arg = DeclareLaunchArgument(
        "client",
        default_value="local",
        description="local = Jetson 내부 loopback / server = 외부 PC 수신",
    )

    encoding_arg = DeclareLaunchArgument(
        "encoding",
        default_value="raw",
        description="raw = sensor_msgs/Image(bgr8) / compressed = CompressedImage(JPEG)",
    )

    return LaunchDescription([
        client_arg,
        encoding_arg,
        OpaqueFunction(function=launch_setup),
    ])
