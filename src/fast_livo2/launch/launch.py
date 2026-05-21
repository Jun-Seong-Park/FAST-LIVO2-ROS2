"""
FAST-LIVO2 integrated launch file.

Includes the sensor bringup launch from ros2_dep_ws, which publishes image/lidar topics.
Topic integrity is the sensor stack's responsibility, not this launch's.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # 1. Package directories
    flivo_pkg = get_package_share_directory('fast_livo2')
    sensor_pkg = get_package_share_directory('sensor')

    # 2. Load file paths
    flivo_config = os.path.join(flivo_pkg, 'config', 'sensor_profiles', 'mid360_24cug', 'fast_livo2.yaml')
    sensor_launch_path = os.path.join(sensor_pkg, 'launch', 'launch.py')

    # 3. Include and Nodes
    sensor_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(sensor_launch_path))

    fast_livo2_node = Node(
        package='fast_livo2',
        executable='fastlivo_mapping2',
        name='fast_livo2_node',
        output='screen',
        parameters=[flivo_config],
    )

    return LaunchDescription([
        sensor_include,
        fast_livo2_node,
    ])
