#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import ByteMultiArray

class RTCMSerialNode(Node):
    def __init__(self):
        super().__init__('rtcm_serial')
        self.port = self.declare_parameter('port', '/dev/ttyTHS1').get_parameter_value().string_value # change serial port
        self.baud = self.declare_parameter('baud', 38400).get_parameter_value().integer_value
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f'Opened serial port {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None
        self.subscription = self.create_subscription(
            ByteMultiArray,
            '/rtcm/corrections', # change this topic
            self.corrections_callback,
            10
        )

    def corrections_callback(self, msg):  # type: ignore
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(msg.data)
                self.get_logger().info(f'Sent {len(msg.data)} bytes to serial device')
            except Exception as e:
                self.get_logger().error(f'Error writing to serial: {e}')
        else:
            self.get_logger().error('Serial port not open')


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    node = RTCMSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.ser:
        node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()