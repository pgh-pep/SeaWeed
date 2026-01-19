from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pd_motion_planner = Node(
        package="seaweed_navigation",
        executable="pd_motion_planner.py",
        name="pd_motion_planner",
        output="screen",
    )

    return LaunchDescription(
        [
            pd_motion_planner,
        ]
    )
