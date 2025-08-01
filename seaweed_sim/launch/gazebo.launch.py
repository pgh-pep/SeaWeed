import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    description_directory = get_package_share_directory("seaweed_description")
    sim_directory = get_package_share_directory("seaweed_sim")
    vrx_gz_directory = get_package_share_directory("vrx_gz")

    local_world_path = os.path.join(sim_directory, "worlds")
    vrx_worlds_path = os.path.join(vrx_gz_directory, "worlds")

    resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{local_world_path}:{resource_path}:{vrx_worlds_path}"

    model = context.perform_substitution(LaunchConfiguration("model"))

    match model:
        case "x_drive":
            model_path = os.path.join(description_directory, "urdf", "x_drive_wamv", "wamv_target.urdf")
        case "diff_thrust":
            model_path = os.path.join(description_directory, "urdf", "diff_thrust_wamv", "wamv_target.urdf")
        case _:
            model_path = os.path.join(description_directory, "urdf", "diff_thrust_wamv", "wamv_target.urdf")

    world = context.perform_substitution(LaunchConfiguration("world"))

    vrx_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vrx_gz_directory, "/launch/competition.launch.py"]),
        launch_arguments={
            "world": world,
            "urdf": model_path,
            "extra_gz_args": "-v 0",  # verbose levels from 0 to 4
            "spawn_pose": "-532.0,162.0,0.0,0.0,0.0,1.0",  # Original spawn point for sydney_regatta
        }.items(),
    )

    rviz_config_file = os.path.join(sim_directory, "rviz", "gazebo.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return [
        vrx_sim_launch,
        rviz,
    ]


def generate_launch_description():
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

    use_gui_arg = DeclareLaunchArgument(name="use_gui", default_value="true")

    return LaunchDescription(
        [
            robot_model_arg,
            use_sim_time_arg,
            use_gui_arg,
            world_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
