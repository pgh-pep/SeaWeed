#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    calibration_file = os.path.join(
        get_package_share_directory('velodyne_pointcloud'),
        'params',
        'VeloView-VLP-32C.yaml'
    )

    velodyne_driver = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver",
        output="screen",
        parameters=[
            {"model": "32C"},
            {"device_ip": "192.168.1.201"},
            {"port": 2368},
            {"frame_id": "velodyne"}
        ]
    )

    velodyne_pointcloud = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_pointcloud",
        output="screen",
        parameters=[
            {"model": "32C"},
            {"calibration": calibration_file},
        ],
    )

    return LaunchDescription([velodyne_driver, velodyne_pointcloud])
