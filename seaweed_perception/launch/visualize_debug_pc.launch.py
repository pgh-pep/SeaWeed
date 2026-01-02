import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    perception_directory = get_package_share_directory("seaweed_perception")

    rviz_config_file = os.path.join(perception_directory, "rviz", "vizualize_debug_pc.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": False}],
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    euclidian_clustering_node = Node(
        package="seaweed_perception",
        executable="euclidian_clustering_node",
        name="euclidian_clustering_node",
        output="screen",
        # parameters=[{}],
    )

    cluster_cache = Node(
        package="seaweed_perception",
        executable="cluster_cache_node",
        name="cluster_cache_node",
        output="screen",
    )

    return [
        euclidian_clustering_node,
        cluster_cache,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
