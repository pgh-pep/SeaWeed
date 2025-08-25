import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    localization_directory = get_package_share_directory("seaweed_localization")
    sim_localization_params = os.path.join(localization_directory, "config", "sim_localization_params.yaml")

    # manually determined init point; turn into a ros param to pick between more worlds as needed
    world = "sydney_regatta"

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

    launch_rviz = LaunchConfiguration("rviz")
    # rviz_config_file = os.path.join(localization_directory, "rviz", "map_frame_viz.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": True}],
        name="rviz2",
        output="screen",
        # arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    return [
        ekf_node,
        navsat_transform_node,
        static_transform_publisher_node,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
            OpaqueFunction(function=launch_setup),
        ]
    )
