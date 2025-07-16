from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{"deadzone": 0.01}],
    )

    diff_thrust_controller = Node(
        package="seaweed_sim",
        executable="diff_thrust_controller.py",
        name="diff_thrust_controller",
        output="screen",
    )

    return LaunchDescription(
        [
            joy_node,
            diff_thrust_controller,
        ]
    )
