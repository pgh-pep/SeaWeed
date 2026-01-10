from launch import LaunchDescription
from launch_ros.actions import Node


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

    pd_motion_planner = Node(
        package="seaweed_navigation",
        executable="pd_motion_planner.py",
        name="pd_motion_planner",
        output="screen",
    )

    # nav_directory = get_package_share_directory("seaweed_navigation")
    # rviz_config_file = os.path.join(nav_directory, "rviz", "map_dash.rviz")

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config_file],
    # )

    return LaunchDescription(
        [
            navigation_manager,
            test_map_generator_node,
            pd_motion_planner,
            # rviz
        ]
    )
