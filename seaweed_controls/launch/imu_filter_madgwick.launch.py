import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_directory = get_package_share_directory("seaweed_controls")

    imu_topic_arg = DeclareLaunchArgument("imu_topic", default_value="imu")
    filtered_imu_topic_arg = DeclareLaunchArgument("filtered_imu_topic", default_value="imu/data")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")

    imu_topic = LaunchConfiguration("imu_topic")
    filtered_imu_topic = LaunchConfiguration("filtered_imu_topic")
    launch_rviz = LaunchConfiguration("rviz")

    rviz_config_file = os.path.join(control_directory, "rviz", "imu_viz.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[
            {
                "use_mag": False,
                "publish_tf": True,
                "world_frame": "enu",
            }
        ],
        remappings=[
            ("imu/data_raw", imu_topic),
            ("imu/data", filtered_imu_topic),
        ],
    )

    return LaunchDescription(
        [
            imu_topic_arg,
            filtered_imu_topic_arg,
            rviz_arg,
            imu_filter_node,
            rviz,
        ]
    )
