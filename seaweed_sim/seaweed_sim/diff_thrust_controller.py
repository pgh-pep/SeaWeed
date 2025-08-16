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

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.max_thrust = 1000.0
        self.left_multiplier  = 0.0
        self.right_multiplier = 0.0
        self.enabled = False

        self.max_linear_vel  = float(self.declare_parameter("max_linear_vel", 2.0).value)   # m/s
        self.max_angular_vel = float(self.declare_parameter("max_angular_vel", 1.0).value)  # rad/s
        self.joy_max_lin_speed = float(self.declare_parameter("joy_max_lin_speed", 2.0).value)
        self.joy_max_ang_speed = float(self.declare_parameter("joy_max_ang_speed", 1.0).value)



        self.enabled = False

        timer_period = 1 / 10  # 10 hz
        self.timer = self.create_timer(
            timer_period,
            self.update_thrusters,
        )

        self.get_logger().info("Diff thrust node started")

    def joy_callback(self, msg: Joy) -> None:

        enabled = (msg.axes[JoyEnum.RIGHT_TRIGGER] != 1)

        tw = Twist()
        if enabled:
            forward = msg.axes[JoyEnum.LEFT_STICK_Y] if len(msg.axes) > JoyEnum.LEFT_STICK_Y else 0.0
            turn    = msg.axes[JoyEnum.LEFT_STICK_X] if len(msg.axes) > JoyEnum.LEFT_STICK_X else 0.0
            tw.linear.x  = float(forward) * self.joy_max_lin_speed
            tw.angular.z = float(turn)    * self.joy_max_ang_speed
        else:
            tw.linear.x  = 0.0
            tw.angular.z = 0.0
        self.cmd_vel_pub.publish(tw)



    def cmd_vel_callback(self, msg: Twist) -> None:
        forward = msg.linear.x  / self.max_linear_vel  if self.max_linear_vel  != 0 else 0.0
        turn = msg.angular.z / self.max_angular_vel if self.max_angular_vel != 0 else 0.0

        forward = max(-1.0, min(1.0, forward))
        turn    = max(-1.0, min(1.0, turn))

        if abs(forward) <= 0.001 and abs(turn) <= 0.001:
            self.enabled = False
            self.left_multiplier  = 0.0
            self.right_multiplier = 0.0
            return

        self.enabled = True
        left  = forward - turn
        right = forward + turn

        self.left_multiplier  = max(-1.0, min(1.0, left))
        self.right_multiplier = max(-1.0, min(1.0, right))

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
        node = DiffThrustController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
