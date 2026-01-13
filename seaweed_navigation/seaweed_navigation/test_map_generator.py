#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import StaticTransformBroadcaster
from enum import Enum
from geometry_msgs.msg import PoseStamped


class Direction(Enum):
    HORIZONTAL = 0
    VERTICAL = 1


class TestMapGenerator(Node):
    def __init__(self):
        super().__init__("test_map")

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.map = OccupancyGrid()

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", map_qos)
        self.goalPose = PoseStamped()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # map params
        self.map_width = 700
        self.map_height = 700
        self.map_resolution = 0.05
        self.map_origin = [-20.0, -20.0, 0.0]  # keep z = 0.0

        self.init_map()

        self.goalPose: PoseStamped = PoseStamped()
        self.goalPose.pose.position.x = 0.0
        self.goalPose.pose.position.y = 10.0
        self.goalPose.header.frame_id = self.map.header.frame_id

        # reference: Unknown = -1, Obstacles = 100, 0 < Free < 99
        self.grid = np.full((self.map_height, self.map_width), 0, dtype=np.int8)

        # ADD TEST OBSTACLES HERE
        self.generate_rect(100, 100, 40, 40)
        self.generate_rect(300, 300, 75, 30)

        # self.publish_test_robot_tf()
        self.generate_map_timer = self.create_timer(0.1, self.timer_callback)

    def init_map(self):
        self.map.header.frame_id = "map"
        self.map.info.resolution = self.map_resolution
        self.map.info.width = self.map_width
        self.map.info.height = self.map_height
        self.map.info.origin.position.x = self.map_origin[0]
        self.map.info.origin.position.y = self.map_origin[1]
        self.map.info.origin.position.z = self.map_origin[2]
        self.map.info.origin.orientation.w = 1.0

    def generate_circle(self, x: int, y: int, r: int):
        pass

    def generate_line(self, x: int, y: int, dist: int, direction: Direction):
        if direction == Direction.HORIZONTAL:
            for i in range(dist):
                if 0 <= y < self.map_height and 0 <= x + i < self.map_width:
                    self.grid[y, x + i] = 100
        elif direction == Direction.VERTICAL:
            for i in range(dist):
                if 0 <= y + i < self.map_height and 0 <= x < self.map_width:
                    self.grid[y + i, x] = 100

    def generate_rect(self, x: int, y: int, l: int, w: int):
        # l -> horizontal, w -> vertical
        self.generate_line(x, y, l, Direction.HORIZONTAL)
        self.generate_line(x, y + w, l, Direction.HORIZONTAL)
        self.generate_line(x, y, w, Direction.VERTICAL)
        self.generate_line(x + l, y, w, Direction.VERTICAL)

    def publish_test_robot_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "wamv/base_link"

        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.data = self.grid.flatten().astype(np.int8).tolist()
        self.map_pub.publish(self.map)
        self.goal_pub.publish(self.goalPose)


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    try:
        test_map = TestMapGenerator()
        rclpy.spin(test_map)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
