import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # Declare pkg name [image, lidar, flivo]
    image_pkg_name = 'image'
    lidar_pkg_name = 'livox_ros_driver2'
    flivo_pkg_name = 'fast_livo'

    # Declare executable name
    image_executable_name = 'image_publish'
    lidar_executable_name = 'livox_ros_driver2_node_best'
    flivo_executable_name = 'fastlivo_mapping2'

    # Declare Node name (must match YAML ros__parameters namespace)
    image_node_name = 'image_publish_node'        # matches image_arducam_b0539.yaml
    lidar_node_name = 'livox_lidar_publisher'      # matches lidar_livox_mid360.yaml
    flivo_node_name = 'fast_livo2_node'            # matches /**:ros__parameters

    # ROS2 pkg path
    image_pkg_dir = get_package_share_directory(image_pkg_name)
    lidar_pkg_dir = get_package_share_directory(lidar_pkg_name)
    flivo_pkg_dir = get_package_share_directory(flivo_pkg_name)

    # Define Config folder path
    flivo_config_path = os.path.join(flivo_pkg_dir, "config")

    # LiDAR user_config_path (MID360_config.json)
    lidar_user_config_path = os.path.join(
        get_package_share_directory(lidar_pkg_name), "config", "MID360_config.json")

    # Define Config file path
    image_config_params_path = os.path.join(flivo_config_path, "image_arducam_b0539.yaml")
    flivo_config_params_path = os.path.join(flivo_config_path, "fast_livo2_B0539_MID360.yaml")
    rviz2_config_params_path = os.path.join(flivo_pkg_dir, "rviz_cfg", "fast_livo2.rviz")

    return LaunchDescription([

        Node(
            package=image_pkg_name,
            executable=image_executable_name,
            name=image_node_name,
            output='screen',
            parameters=[
                image_config_params_path
                ]
        ),

        Node(
            package=lidar_pkg_name,
            executable=lidar_executable_name,
            name=lidar_node_name,
            output='screen',
            parameters=[
                {"xfer_format": 1},
                {"multi_topic": 0},
                {"data_src": 0},
                {"publish_freq": 10.0},
                {"output_data_type": 0},
                {"frame_id": "livox_frame"},
                {"user_config_path": lidar_user_config_path},
                {"cmdline_input_bd_code": "livox0000000001"},
            ]
        ),

        Node(
            package=flivo_pkg_name,
            executable=flivo_executable_name,
            name=flivo_node_name,
            parameters=[
                flivo_config_params_path,
                ],
            # prefix=["gdb -ex run --args"],
            output="screen"
        ),

    ])
