import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    description_directory = get_package_share_directory("seaweed_description")

    model = context.perform_substitution(LaunchConfiguration("model"))
    match model:
        case "x_drive":
            model_path = os.path.join(description_directory, "urdf", "x_drive_wamv", "wamv_target.urdf")
        case "diff_thrust":
            model_path = os.path.join(description_directory, "urdf", "diff_thrust_wamv", "wamv_target.urdf")
        case _:
            model_path = os.path.join(description_directory, "urdf", "diff_thrust_wamv", "wamv_target.urdf")

    robot_description = xacro.process_file(model_path).toxml()

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": robot_description,
                "frame_prefix": "wamv/",
            }
        ],
    )

    nodes = [
        robot_state_publisher_node,
    ]

    rviz_config_arg = context.perform_substitution(LaunchConfiguration("rviz_config"))
    match rviz_config_arg:
        case "visualize_model":
            rviz_config_file = os.path.join(description_directory, "rviz", "display_wamv.rviz")
            nodes.append(joint_state_publisher_gui_node)
        case "debug":
            rviz_config_file = os.path.join(description_directory, "rviz", "debug_dashboard.rviz")
            nodes.append(joint_state_publisher_node)
        case _:
            rviz_config_file = os.path.join(description_directory, "rviz", "display_wamv.rviz")
            nodes.append(joint_state_publisher_gui_node)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    nodes.append(rviz)

    return nodes


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        name="model",
        default_value="diff_thrust",
        choices=["x_drive", "diff_thrust"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value="visualize_model",
        choices=["visualize_model", "debug"],
    )

    return LaunchDescription(
        [
            robot_model_arg,
            use_sim_time_arg,
            rviz_config_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
