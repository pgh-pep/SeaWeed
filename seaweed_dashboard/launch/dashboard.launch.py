from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                # TODO: Set to the actual joystick device id
                "device_id": 0,
            }
        ],
    )

    dashboard_node = Node(
        package="seaweed_dashboard",
        executable="dashboard.py",
        name="dashboard_node",
        output="screen",
    )

    return LaunchDescription(
        [
            joy_node,
            dashboard_node,
        ]
    )
