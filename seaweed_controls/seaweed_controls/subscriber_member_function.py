#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
from seaweed_interfaces.msg import Command                     # CHANGE
import struct


class MinimalSubscriber(Node):

    def __init__(self):
        self.front_left = 0.0
        self.front_right = 0.0
        self.back_left = 0.0
        self.back_right = 0.0

        super().__init__('minimal_subscriber')

        self.bus = can.Bus(
            interface='socketcan',
            channel='can0',
            bitrate=500000
        )

        self.subscription = self.create_subscription(
            Command,                                               # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

        self.timer = self.create_timer(0.01, self.timer_callback)



    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%f, %f, %f, %f"' % (msg.front_left, msg.front_right, msg.back_left,  msg.back_right)) 
        self.front_left = msg.front_left
        self.front_right = msg.front_right
        self.back_left = msg.back_left
        self.back_right = msg.back_right

    def timer_callback(self):
        #self.get_logger().info('Sent "%f, %f, %f, %f"' % (self.front_left, self.front_right, self.back_left, self.back_right)) 
        thrusts = [self.front_left, self.front_right, self.back_left, self.back_right]

        for idx, t in enumerate(thrusts):
            data = struct.pack('<f', t)
            id = 0b11100011111
            id = id + idx << 5

            msg = can.Message(
                arbitration_id=id,
                data=data,
                is_extended_id=False
            )

            try:
                self.bus.send(msg)
                self.get_logger().info("Sent thrust: {t:.2f}")
            except can.CanError as e:
                self.get_logger().error(f'CAN send failed: {e}')



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()