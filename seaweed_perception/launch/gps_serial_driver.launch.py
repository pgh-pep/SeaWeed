#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    nmea_navsat = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        name="gps_serial_driver",
        output="screen",
        parameters=[  # Change these
            {"port": "/dev/ttyUSB0"},
            {"baud": 38400},
        ],
        remappings=[
            ("/fix", "/gps/fix"),
            ("/vel", "/gps/vel"),
            ("/time_reference", "/gps/time_reference"),
        ],
        # arguments=["--ros-args", "--log-level", "debug"],
    )

    rtcm_serial = Node(
        package="seaweed_perception",
        executable="rtcm_serial.py",
        name="rtcm_serial_driver",
        output="screen",
        parameters=[  # change these
            {"port": "/dev/ttyTHS1"},
            {"baud": 38400},
        ],
        remappings=[
            ("/rtcm/corrections", "/gps/rtcm/corrections"),
        ],
        # arguments=["--ros-args", "--log-level", "debug"],
    )

    return LaunchDescription([nmea_navsat, rtcm_serial])
