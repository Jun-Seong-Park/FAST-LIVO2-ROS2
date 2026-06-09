import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    home     = os.path.expanduser("~")
    save_dir = os.path.join(
        home, "FAST-LIVO2-ROS2", "src", "sensor", "econ_ros2_driver", "Log")

    localhost_only = SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1")

    image_saver = Node(
        package="econ_ros2_driver",
        executable="econ_image_saver",
        name="image_saver",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "topic":      "/camera/image",
            "save_dir":   save_dir,
            "period_sec": 1,
        }],
    )

    return LaunchDescription([localhost_only, image_saver])
