import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("seaweed_controls"), "config")

    # Bridges joy in from the base station and odometry out to it
    domain_bridge = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="domain_bridge",
        output="screen",
        arguments=[os.path.join(config_dir, "bridge.yaml")],
    )

    # joy -> cmd_vel
    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[os.path.join(config_dir, "teleop_twist_joy.yaml")],
    )

    # cmd_vel -> thruster commands
    diff_thrust = Node(
        package="seaweed_controls",
        executable="diff_thrust_node.py",
        name="diff_thrust_node",
        output="screen",
        parameters=[os.path.join(config_dir, "diff_thrust.yaml")],
    )

    return LaunchDescription(
        [
            domain_bridge,
            teleop_twist_joy,
            diff_thrust,
        ]
    )
