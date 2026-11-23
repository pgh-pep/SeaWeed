#!/usr/bin/env python3

from typing import Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Float64

THRUSTERS = ("front_left", "front_right", "back_left", "back_right")


class DiffThrustNode(Node):
    """Open loop differential drive: cmd_vel -> four thruster commands."""

    def __init__(self):
        super().__init__("diff_thrust_node")

        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "cmd_vel").get_parameter_value().string_value
        self.thrust_pubs = {}
        for name in THRUSTERS:
            topic = (
                self.declare_parameter(f"{name}_thrust_topic", f"{name}_thrust").get_parameter_value().string_value
            )
            self.thrust_pubs[name] = self.create_publisher(Float64, topic, 10)

        # Read on every update so they can be tuned live with `ros2 param set`.
        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("angular_gain", 0.5)
        self.declare_parameter("thrust_limit", 1.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)

        publish_rate = self.declare_parameter("publish_rate", 20.0).get_parameter_value().double_value

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_cmd_time: Time | None = None

        self.cmd_vel_sub = self.create_subscription(TwistStamped, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.update_thrusters)

        self.get_logger().info("Diff thrust node started")

    def cmd_vel_callback(self, msg: TwistStamped) -> None:
        self.linear_x = msg.twist.linear.x
        self.angular_z = msg.twist.angular.z
        self.last_cmd_time = self.get_clock().now()

    def cmd_is_fresh(self) -> bool:
        if self.last_cmd_time is None:
            return False
        timeout = self.get_float_param("cmd_vel_timeout")
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age <= timeout

    def get_float_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def mix(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        linear_gain = self.get_float_param("linear_gain")
        angular_gain = self.get_float_param("angular_gain")
        limit = abs(self.get_float_param("thrust_limit"))

        left = linear_gain * linear_x - angular_gain * angular_z
        right = linear_gain * linear_x + angular_gain * angular_z

        # Scale both sides together so turning is preserved at the limit
        peak = max(abs(left), abs(right))
        if peak > limit:
            scale = limit / peak if peak > 0.0 else 0.0
            left *= scale
            right *= scale

        # Hard clamp
        left = max(-limit, min(limit, left))
        right = max(-limit, min(limit, right))
        return left, right

    def update_thrusters(self) -> None:
        if self.cmd_is_fresh():
            left, right = self.mix(self.linear_x, self.angular_z)
        else:
            left, right = 0.0, 0.0

        self.publish_thrusts(left, right)

    def publish_thrusts(self, left: float, right: float) -> None:
        thrusts = {
            "front_left": left,
            "front_right": right,
            "back_left": left,
            "back_right": right,
        }
        for name, value in thrusts.items():
            self.thrust_pubs[name].publish(Float64(data=float(value)))

    def destroy_node(self):
        # Stop the thrusters on the way out (needs a live context)
        if self.context.ok():
            self.publish_thrusts(0.0, 0.0)
        super().destroy_node()


def main(args=None):  # type: ignore
    # Handle SIGINT here instead of in rclpy, so the context is still valid
    # when destroy_node publishes the final zero thrust.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    diff_thrust_node = None
    try:
        diff_thrust_node = DiffThrustNode()
        rclpy.spin(diff_thrust_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if diff_thrust_node is not None:
            diff_thrust_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()