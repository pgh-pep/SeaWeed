#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from seaweed_interfaces.msg import Command                         # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Command, 'topic', 10)  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.front_left = 0.0
        self.front_right = 0.0
        self.back_left = 0.0
        self.back_right = 0.0

    def timer_callback(self):
        msg = Command()                                                # CHANGE
        msg.front_left = self.front_left
        msg.front_right = self.front_right
        msg.back_left = self.back_left
        msg.back_right = self.back_right                                          # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f, %f, %f, %f"' % (msg.front_left, msg.front_right,  msg.back_left, msg.back_right))       # CHANGE
   


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()