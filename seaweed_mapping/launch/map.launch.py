import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    params = os.path.join(get_package_share_directory('seaweed_mapping'), 'config', 'costmap.yaml')
    return LaunchDescription([
        Node(
            package='seaweed_mapping',
            executable='point_cluster',
            name='point_cluster_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                "pc_sub_topic": "/wamv/sensors/lidars/lidar_wamv_sensor/points",
                "pc_pub_topic": "/cluster/pointcloud",
                "pose_pub_topic": "/cluster/poses",
                "target_frame": "odom",
                "min_z": 0.75,
                "max_radius": 20.0,
                "cluster_tolerance": 0.3,
                "min_cluster_size": 5,
                "robot_radius": 1.0,
                "max_cluster_size": 10000,
            }]
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',    
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['costmap'],
            }],
        )
    ])