from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    nodes = []

    if context.perform_substitution(LaunchConfiguration("debug_map")) == "true":
        nodes.append(
            Node(
                package="seaweed_navigation",
                executable="test_map_generator.py",
                name="test_map_generator",
                output="screen",
            )
        )

    return nodes

def generate_launch_description():
    debug_map_arg = DeclareLaunchArgument(name="debug_map", default_value="false", choices=["true", "false"])

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
            pd_motion_planner,
            debug_map_arg,
            OpaqueFunction(function=launch_setup)
        ]
    )
