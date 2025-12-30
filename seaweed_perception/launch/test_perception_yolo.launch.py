import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from dotenv import load_dotenv, find_dotenv

load_dotenv(find_dotenv())

ROSBAG_DIR = os.getenv("ROSBAG_DIR")
if not ROSBAG_DIR:
    raise EnvironmentError("ROSBAG_DIR environment variable not set. Please set it in the .env file.")

rosbag_file = os.path.join(ROSBAG_DIR, "basic_sim_readings_0.db3")


def generate_launch_description():
    return LaunchDescription(
        [
            # launch rosbag via the ros2 CLI (more portable than launching package executable)
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    rosbag_file,
                ],
                output="screen",
                shell=False,
            ),
            # launch perception node
            Node(
                package="seaweed_perception",
                executable="yolo_node.py",
                name="yolo_node",
                output="screen",
            ),
        ]
    )
