from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="seaweed_sim",
            executable="diff_thrust_controller.py",  # coincide con tu launch actual
            name="diff_thrust_controller",
            output="screen",
            parameters=[{
                "max_linear_vel": 2.0,
                "max_angular_vel": 1.0,
                "joy_max_lin_speed": 2.0,
                "joy_max_ang_speed": 1.0,
            }],
        ),
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            name="teleop_twist_keyboard",
            output="screen",  # publica en /cmd_vel
        ),
    ])

