import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    sim_pkg_dir = get_package_share_directory('seaweed_sim')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_dir, 'launch', 'full_sim.launch.py')
        )
    )

    return [sim_launch]

def generate_launch_description():
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

    robot_model_arg = DeclareLaunchArgument(
        name="model", default_value="diff_thrust", choices=["x_drive", "diff_thrust"]
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="sydney_regatta",
        choices=[
            "sydney_regatta",
            "sydney_regatta_empty",
            "nbpark",
            "follow_path",
        ],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
    )

    use_gui_arg = DeclareLaunchArgument(name="use_gui", default_value="true")  # unused
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"])

    return LaunchDescription(
        [
            robot_model_arg,
            use_sim_time_arg,
            use_gui_arg,
            world_arg,
            rviz_arg,
            joy_node,
            diff_thrust_controller,
            navigation_manager,
            test_map_generator_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
