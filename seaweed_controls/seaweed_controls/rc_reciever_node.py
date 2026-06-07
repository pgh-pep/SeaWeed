#!/usr/bin/env python3

import time
from typing import Dict
import rclpy
from rclpy.node import Node
import serial
from enum import Enum
from std_msgs.msg import Bool
from seaweed_interfaces.msg import RCcontroller


BAUD_RATE = 115200


class Channels(Enum):
    Joy_R_LR = 0
    Joy_R_UD = 1
    Joy_L_UD = 2
    Joy_L_LR = 3
    SW_B = 4
    Vr_B = 5
    SW_A = 6
    Vr_A = 7
    CH9 = 8
    CH10 = 9
    CH11 = 10
    CH12 = 11
    CH13 = 12
    CH14 = 13
    CH15 = 14
    CH16 = 15
    FAIL_SAFE = 16
    FRAME_LOST = 17


class RCRecieverNode(Node):
    def __init__(self):
        super().__init__("rc_reciever_node")

        self.serial_port = "/dev/ttyUSB0"  # make ros param
        self.serial_ESP = serial.Serial(self.serial_port, BAUD_RATE, timeout=0.1)
        time.sleep(2.5)

        rc_controller_topic = "/rc/channels"
        self.rc_msg_pub = self.create_publisher(RCcontroller, rc_controller_topic, 10)

        rc_fail_safe_topic = "/rc/fail_safe"
        self.rc_fail_safe_pub = self.create_publisher(Bool, rc_fail_safe_topic, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(f"Serial ESP on {self.serial_port} @ {BAUD_RATE}")

    def parse_packet(self, packet: str) -> Dict[Channels, float] | None:
        # expected packet: "CH1, CH2 ... CH16, failsafe, frame_lost"
        try:
            parsed_pkt = packet.strip().split(",")
            if len(parsed_pkt) != 18:
                self.get_logger().warn(f"SBUS packet has incorrect length: {len(parsed_pkt)}")
                return None

            temp_channels: Dict[Channels, float] = dict()
            for i, channel in enumerate(Channels):
                temp_channels[channel] = float(parsed_pkt[i].strip())

            return temp_channels

        except (ValueError, IndexError) as e:
            print(f"SBUS parse error: {e}")
            return None

    def close_connection(self):
        if self.serial_ESP:
            self.serial_ESP.close()
            print("ending SBUS comms")

    def destroy_node(self):
        self.close_connection()
        super().destroy_node()

    def pub_rc_msg(self, channels: Dict[Channels, float]):
        msg = RCcontroller()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joy_r_lr = channels[Channels.Joy_R_LR]
        msg.joy_r_ud = channels[Channels.Joy_R_UD]
        msg.joy_l_ud = channels[Channels.Joy_L_UD]
        msg.joy_l_lr = channels[Channels.Joy_L_LR]
        msg.sw_b = channels[Channels.SW_B]
        msg.vr_b = channels[Channels.Vr_B]
        msg.sw_a = channels[Channels.SW_A]
        msg.vr_a = channels[Channels.Vr_A]

        self.rc_msg_pub.publish(msg)

    def pub_fail_safe_msg(self, channels: Dict[Channels, float]):
        msg = Bool()
        msg.data = bool(int(channels[Channels.FAIL_SAFE]))

        self.rc_fail_safe_pub.publish(msg)

    def timer_callback(self):
        raw_packet = self.serial_ESP.readline()
        if not raw_packet:
            return

        packet = raw_packet.decode("utf-8", errors="ignore").strip()
        channels = self.parse_packet(packet)
        if channels is None:
            return

        self.pub_rc_msg(channels)
        self.pub_fail_safe_msg(channels)


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    rc_reciever_node = None
    try:
        rc_reciever_node = RCRecieverNode()
        rclpy.spin(rc_reciever_node)
    finally:
        if rc_reciever_node is not None:
            rc_reciever_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
