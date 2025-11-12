from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    return LaunchDescription([
        print("testing"),
        # launcehs zed wrapper using zed21 yaml stuff
        Node(
            package='zed_wrapper',
            executable='zed_wrapper_node',
            name='zed2i_camera',
            parameters='zed2i_parameters.yaml',
            output='screen'
        ),
        
        #launches RGB publisher
        Node(
            package='seaweed_perception',
            executable='RGB_publisher_node.py',
            name='zed2i_RGB_publisher',
            output='screen'
        ),

        # launches depth publisher
        Node(
            package='seaweed_perception',
            executable='depth_publisher_node.py',
            name='zed2i_depth_publisher',
            output='screen'
        )

    ])