from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: Debug map arg
    test_map_generator_node = Node(
        package="seaweed_navigation",
        executable="test_map_generator.py",
        name="test_map_generator",
        output="screen",
    )

    navigation_manager = Node(
        package="seaweed_navigation",
        executable="navigation_manager.py",
        name="navigation_manager",
        output="screen",
    )

    pd_motion_planner = Node(
        package="seaweed_navigation",
        executable="pd_motion_planner.py",
        name="pd_motion_planner",
        output="screen",
    )

    return LaunchDescription(
        [
            navigation_manager,
            test_map_generator_node,
            pd_motion_planner,
            # rviz
        ]
    )
