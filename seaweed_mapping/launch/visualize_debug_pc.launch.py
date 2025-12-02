import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mapping_directory = get_package_share_directory("seaweed_mapping")

    rviz_config_file = os.path.join(mapping_directory, "rviz", "vizualize_debug_pc.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": False}],
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    test_node = Node(
        package="seaweed_mapping",
        executable="test_node",
        name="test_node",
        output="screen",
        parameters=[{}],
    )

    return [
        test_node,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
