from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('seaweed_localization')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/wamv/sensors/lidars/lidar_wamv_sensor/points')
    imu_topic = LaunchConfiguration('imu_topic', default='/wamv/sensors/imu/imu/data')

    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )

    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'config', 'dlio_sim_config.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'config', 'dlio_sim_params.yaml'])

    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    # DLIO Mapping Node
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )

    return LaunchDescription([
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        dlio_odom_node,
        dlio_map_node,
    ])
