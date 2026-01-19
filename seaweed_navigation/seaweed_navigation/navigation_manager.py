#!/usr/bin/env python3

import math
from typing import Optional
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration

# from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Pose, Point
from a_star import AStarPlanner
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy


class NavigationManager(Node):
    def __init__(self):
        super().__init__("navigation_manager")

        # self.action_server = ActionServer(
        #     self,
        #     NavigateToPose,
        #     'navigate_to_pose',
        #     execute_callback=self.execute_callback,
        #     cancel_callback=self.cancel_callback,
        #     goal_callback=self.goal_callback
        # )

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, "/path", 10)

        self.map = None
        goal_tol = 0.1  # m
        self.planner = AStarPlanner(map=self.map, goal_tolerance=goal_tol)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # DEBUG: Visualize visited nodes
        self.debug_map = OccupancyGrid()
        self.debug_map_pub = self.create_publisher(OccupancyGrid, "/debug/map", 10)

    def map_callback(self, map_msg: OccupancyGrid):
        self.map = map_msg
        self.planner.map = map_msg

        self.debug_map.header.frame_id = map_msg.header.frame_id
        self.debug_map.info = map_msg.info

    def goal_callback(self, goal_pose_stamped: PoseStamped):
        if self.map is None:
            self.get_logger().error("No map received")
            return

        base_link_frame = "wamv/base_link"
        robot_pose = self.lookup_pose(target_frame=self.map.header.frame_id, source_frame=base_link_frame)

        if robot_pose is None:
            self.get_logger().error("Can't get robot pose in map frame")
            return

        robot_position = Point()
        robot_position.x = robot_pose.position.x
        robot_position.y = robot_pose.position.y

        goal_point = Point()
        goal_point.x = goal_pose_stamped.pose.position.x
        goal_point.y = goal_pose_stamped.pose.position.y

        dx = goal_point.x - robot_position.x
        dy = goal_point.y - robot_position.y
        distance = math.sqrt(dx * dx + dy * dy)
        # stop replanning if robot is within tolerance of goal
        if distance >= 6:
            path = self.planner.plan(robot_position, goal_point)
            self.get_logger().info("Replanning.. distance =" + str(distance))
            if path and path.poses:
                # self.get_logger().info("Path found")
                self.path_pub.publish(path)
            else:
                self.get_logger().warn("No path found")

    def lookup_pose(self, target_frame: str, source_frame: str) -> Optional[Pose]:
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=1))

            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation

            return pose

        except Exception as ex:
            self.get_logger().error(f"Cannot transform from {source_frame} to {target_frame}: {ex}")
            return None


def main(args=None):  # type: ignore
    rclpy.init(args=args)
    try:
        navigation_manager = NavigationManager()
        rclpy.spin(navigation_manager)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
