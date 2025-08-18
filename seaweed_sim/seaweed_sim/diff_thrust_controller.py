#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from typing import Dict
from seaweed_sim.joy_enum import JoyEnum


class DiffThrustController(Node):
    def __init__(self):
        super().__init__("diff_thrust_controller")

        ns = "/wamv/thrusters"

        self.front_left_pub = self.create_publisher(Float64, f"{ns}/front_left/thrust", 10)
        self.front_right_pub = self.create_publisher(Float64, f"{ns}/front_right/thrust", 10)
        self.back_left_pub = self.create_publisher(Float64, f"{ns}/back_left/thrust", 10)
        self.back_right_pub = self.create_publisher(Float64, f"{ns}/back_right/thrust", 10)

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 1)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 1)

        self.max_thrust = 1000.0
        self.left_multiplier = 0
        self.right_multiplier = 0

        self.enabled = False

        timer_period = 1 / 10  # 10 hz
        self.timer = self.create_timer(
            timer_period,
            self.update_thrusters,
        )

        self.get_logger().info("Diff thrust node started")

    def joy_callback(self, msg: Joy) -> None:
        if msg.axes[JoyEnum.RIGHT_TRIGGER] == 1:
            self.enabled = False
        else:
            self.enabled = True

        if self.enabled:
            forward = msg.axes[JoyEnum.LEFT_STICK_Y]
            turn = msg.axes[JoyEnum.LEFT_STICK_X]

            self.left_multiplier = max(-1.0, min(1.0, forward - turn))
            self.right_multiplier = max(-1.0, min(1.0, forward + turn))
        else:
            self.left_multiplier = 0
            self.right_multiplier = 0

    def cmd_vel_callback(self, msg: Twist) -> None:
        # Normalize inputs to [-1, 1]
        max_linear_vel = 2.0
        max_angular_vel = 1.0
        forward = msg.linear.x / max_linear_vel
        turn = msg.angular.z / max_angular_vel

        self.left_multiplier = max(-1.0, min(1.0, forward - turn))
        self.right_multiplier = max(-1.0, min(1.0, forward + turn))

        # Min threshold
        if abs(forward) > 0.001 or abs(turn) > 0.001:
            self.enabled = True
        else:
            self.enabled = False
            self.left_multiplier = 0
            self.right_multiplier = 0

    def update_thrusters(self) -> None:
        # Normalize multipliers
        max_cmd = max(abs(self.left_multiplier), abs(self.right_multiplier))

        if max_cmd > 1.0:
            left_thrust = (self.left_multiplier / max_cmd) * self.max_thrust
            right_thrust = (self.right_multiplier / max_cmd) * self.max_thrust
        else:
            left_thrust = self.left_multiplier * self.max_thrust
            right_thrust = self.right_multiplier * self.max_thrust

        thrusts = {
            "front_left": left_thrust,
            "front_right": right_thrust,
            "back_left": left_thrust,
            "back_right": right_thrust,
        }

        self.pub_thrusts(thrusts)

    def pub_thrusts(self, thrusts: Dict[str, float]) -> None:
        # self.get_logger().info(f"front_left: {thrusts['front_left']}")
        # self.get_logger().info(f"front_right: {thrusts['front_right']}")
        # self.get_logger().info(f"back_left: {thrusts['back_left']}")
        # self.get_logger().info(f"back_right: {thrusts['back_right']}")

        front_left_msg = Float64()
        front_left_msg.data = thrusts["front_left"]
        self.front_left_pub.publish(front_left_msg)

        front_right_msg = Float64()
        front_right_msg.data = thrusts["front_right"]
        self.front_right_pub.publish(front_right_msg)

        back_left_msg = Float64()
        back_left_msg.data = thrusts["back_left"]
        self.back_left_pub.publish(back_left_msg)

        back_right_msg = Float64()
        back_right_msg.data = thrusts["back_right"]
        self.back_right_pub.publish(back_right_msg)


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    try:
        diff_thrust_controller = DiffThrustController()
        rclpy.spin(diff_thrust_controller)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
