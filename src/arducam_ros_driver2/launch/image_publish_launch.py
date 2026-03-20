import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('image')
    arducam_yaml = os.path.join(pkg_dir, 'config', 'arducam.yaml')
    cam_yaml = os.path.join(pkg_dir, 'config', 'cam.yaml')

    return LaunchDescription([
        Node(
            package='image',
            executable='image_publish',
            name='image_publish_node',
            output='screen',
            parameters=[arducam_yaml, cam_yaml]
        )
    ])
