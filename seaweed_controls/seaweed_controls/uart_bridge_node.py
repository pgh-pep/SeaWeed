#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import try_shutdown
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int8MultiArray, UInt8
from uart_bridge import UARTBridge
from uart_bridge_state import UARTBridgeState

# IMU (SCH16T) in 20-bit mode
GYRO_LSB_PER_DPS: float = 1600.0
ACCEL_LSB_PER_MS2: float = 3200.0
GYRO_TO_RADS: float = math.pi / 180.0 / GYRO_LSB_PER_DPS


class UARTBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("uart_bridge_node")

        # ros params
        port: str = self.declare_parameter("port", "/dev/ttyACM1").value
        timeout: float = self.declare_parameter("timeout", 0.5).value
        tx_period: float = self.declare_parameter("cmd_tx_period", 0.02).value  # 50 Hz
        self.imu_frame: str = self.declare_parameter("imu_frame", "imu_frame").get_parameter_value().string_value
        debug: bool = self.declare_parameter("debug", True).value
        debug_period: float = self.declare_parameter("debug_period", 0.2).value

        # pubs
        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.volt_pub = self.create_publisher(Float32, "voltage", 10)
        self.curr_pub = self.create_publisher(Float32, "current", 10)

        self.state: UARTBridgeState = UARTBridgeState()
        self.bridge: UARTBridge = UARTBridge(
            port,
            self.state,
            on_rx=self.on_rx,
            timeout=timeout,
            tx_period=tx_period,
            debug=debug,
            debug_period=debug_period,
            log=self.get_logger().info,
        )

        # subs
        self.create_subscription(UInt8, "state_cmd", self.state_cb, 10)
        self.create_subscription(Int8MultiArray, "thrusters", self.thruster_cmd_cb, 10)
        self.create_subscription(UInt8, "launch_action", self.launch_action_cb, 10)
        self.create_subscription(UInt8, "launcher_angle", self.launcher_angle_cb, 10)

        # uart heartbeat
        self.create_timer(1.0, self.check_connected)

        # init comms
        self.bridge.start()

    # ROS -> UART
    def state_cb(self, msg: UInt8) -> None:
        with self.state.lock:
            self.state.state = msg.data
            self.state.state_time = time.time()

    def thruster_cmd_cb(self, msg: Int8MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn("thrusters needs 4 values [fl, fr, bl, br]")
            return
        with self.state.lock:
            self.state.fl, self.state.fr, self.state.bl, self.state.br = msg.data
            self.state.thruster_time = time.time()

    def launch_action_cb(self, msg: UInt8) -> None:
        with self.state.lock:
            self.state.launch_action = msg.data
            self.state.launch_time = time.time()

    def launcher_angle_cb(self, msg: UInt8) -> None:
        with self.state.lock:
            self.state.launcher_angle = msg.data
            self.state.launch_time = time.time()

    # UART -> ROS, called from serial thread
    def on_rx(self, d: dict[str, int]) -> None:
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame
        imu.orientation_covariance[0] = -1.0  # no estimate
        imu.angular_velocity.x = d["gyro_x"] * GYRO_TO_RADS
        imu.angular_velocity.y = d["gyro_y"] * GYRO_TO_RADS
        imu.angular_velocity.z = d["gyro_z"] * GYRO_TO_RADS
        imu.linear_acceleration.x = d["accel_x"] / ACCEL_LSB_PER_MS2
        imu.linear_acceleration.y = d["accel_y"] / ACCEL_LSB_PER_MS2
        imu.linear_acceleration.z = d["accel_z"] / ACCEL_LSB_PER_MS2
        self.imu_pub.publish(imu)
        self.volt_pub.publish(Float32(data=float(d["voltage"])))
        self.curr_pub.publish(Float32(data=float(d["current"])))

    def check_connected(self) -> None:
        with self.state.lock:
            connected: bool = self.state.connected
        if not connected:
            self.get_logger().warn("serial not connected")

    def destroy_node(self) -> None:
        self.bridge.stop()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    uart_bridge_node = UARTBridgeNode()
    try:
        rclpy.spin(uart_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        uart_bridge_node.destroy_node()
        try_shutdown()


if __name__ == "__main__":
    main()
