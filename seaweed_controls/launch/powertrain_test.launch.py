import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_config = os.path.join(
        get_package_share_directory("seaweed_controls"), "config", "bridge.yaml"
    )

    domain_bridge = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="domain_bridge",
        output="screen",
        arguments=[bridge_config],
    )

    return LaunchDescription(
        [
            domain_bridge,
        ]
    )
