import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    description_directory = get_package_share_directory("seaweed_description")
    sim_directory = get_package_share_directory("seaweed_sim")
    vrx_gz_directory = get_package_share_directory("vrx_gz")
    localization_directory = get_package_share_directory("seaweed_localization")

    # GAZEBO SIMULATION
    local_world_directory = os.path.join(sim_directory, "worlds")
    vrx_worlds_directory = os.path.join(vrx_gz_directory, "worlds")

    resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{local_world_directory}:{resource_path}:{vrx_worlds_directory}"

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
            # "spawn_pose": "-532.0,162.0,0.0,0.0,0.0,1.0",  # Original spawn point for sydney_regatta
            "spawn_pose": "-520.0,180.0,0.0,0.0,0.0,1.57",  # Original spawn point for sydney_regatta
        }.items(),
    )

    # LOCALIZATION
    sim_localization_params = os.path.join(localization_directory, "config", "sim_localization_params.yaml")

    world = "sydney_regatta"
    # manually determined init point; turn into a ros param to pick between more worlds as needed
    locations = {"sydney_regatta": (-33.7226, 150.6741)}
    lat, lon = locations[world]

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[
            ("gps/fix", "/wamv/sensors/gps/gps/fix"),
        ],
        parameters=[
            sim_localization_params,
            # {"datum": [lat, lon, 0.0]},  # comment out to set origin at first gps reading
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[sim_localization_params],
    )

    # Assume no sensor drift
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # VISUALIZATION/TESTING
    rviz_config_file = os.path.join(sim_directory, "rviz", "gazebo_full.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": True}],
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return [
        vrx_sim_launch,
        ekf_node,
        navsat_transform_node,
        static_transform_publisher_node,
        rviz,
    ]


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
            navigation_manager,
            test_map_generator_node,
            robot_model_arg,
            use_sim_time_arg,
            use_gui_arg,
            world_arg,
            rviz_arg,
            joy_node,
            diff_thrust_controller,
            OpaqueFunction(function=launch_setup),
        ]
    )
