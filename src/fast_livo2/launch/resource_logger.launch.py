import os

from ament_index_python.packages import get_package_share_directory
from launch                       import LaunchDescription
from launch_ros.actions           import Node


def generate_launch_description():
    pkg_share   = get_package_share_directory('fast_livo2')
    config_path = os.path.join(pkg_share, 'config', 'resource_logger.yaml')

    logger_node = Node(
        package    = 'fast_livo2',
        executable = 'resource_logger',
        name       = 'resource_logger',
        parameters = [config_path],
        output     = 'screen',
    )

    return LaunchDescription([logger_node])
