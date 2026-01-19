#!/usr/bin/env python3

import math
from typing import List
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration

# from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from pid import PID
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf_transformations import euler_from_quaternion
# from rclpy.qos import QoSProfile, DurabilityPolicy


class PDMotionPlanner(Node):
    def __init__(self):
        super().__init__("pd_motion_planner")

        self.declare_parameter("heading_kp", 0.75)
        self.declare_parameter("heading_kd", 0.2)

        self.declare_parameter("cross_track_kp", 1.2)
        self.declare_parameter("cross_track_kd", 0.4)

        self.declare_parameter("ks", 1)  # along-track gain
        self.declare_parameter("kn", 0.1)  # cross-track gain
        self.declare_parameter("ktheta", 0.01)  # heading gain

        self.declare_parameter("lookahead_dist", 1)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.declare_parameter("open_loop_enabled", True)
        self.declare_parameter("closed_loop_enabled", True)

        self.heading_kp: float = self.get_parameter("heading_kp").value  # type: ignore
        self.heading_kd: float = self.get_parameter("heading_kd").value  # type: ignore

        self.cross_track_kp: float = self.get_parameter("cross_track_kp").value  # type: ignore
        self.cross_track_kd: float = self.get_parameter("cross_track_kd").value  # type: ignore

        self.ks: float = self.get_parameter("ks").value  # type: ignore
        self.kn: float = self.get_parameter("kn").value  # type: ignore
        self.ktheta: float = self.get_parameter("ktheta").value  # type: ignore

        self.lookahead_dist: float = self.get_parameter("lookahead_dist").value  # type: ignore
        self.max_linear_velocity: float = self.get_parameter("max_linear_velocity").value  # type: ignore
        self.max_angular_velocity: float = self.get_parameter("max_angular_velocity").value  # type: ignore

        self.open_loop_enabled: bool = self.get_parameter("open_loop_enabled").value  # type: ignore
        self.closed_loop_enabled: bool = self.get_parameter("closed_loop_enabled").value  # type: ignore

        self.along_track_pid = PID()
        self.cross_track_pid = PID(self.cross_track_kp, 0.0, self.cross_track_kd)
        self.heading_pid = PID(self.heading_kp, 0.0, self.heading_kd)

        self.path_sub = self.create_subscription(Path, "/path", self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.control_loop = self.create_timer(0.1, self.update_controls)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path: Path = Path()
        self.last_cycle_time = self.get_clock().now()
        self.goal_reached = False

        # modify to recieve path
        # self.action_server = ActionServer(
        #     self,
        #     NavigateToPose,
        #     'navigate_to_pose',
        #     execute_callback=self.execute_callback,
        #     cancel_callback=self.cancel_callback,
        #     goal_callback=self.goal_callback
        # )

        # DEBUG
        self.debug_distance_from_goal = self.create_publisher(Float64, "/debug/distance_from_goal", 10)
        self.debug_heading_error = self.create_publisher(Float64, "/debug/heading_error", 10)
        self.debug_cross_track_error = self.create_publisher(Float64, "/debug/cross_track_error", 10)
        self.debug_along_track_error = self.create_publisher(Float64, "/debug/along_track_error", 10)
        self.debug_goal_heading = self.create_publisher(Float64, "/debug/goal_heading", 10)
        self.debug_robot_heading = self.create_publisher(Float64, "/debug/robot_heading", 10)

    def path_callback(self, msg: Path):
        self.last_cycle_time = self.get_clock().now()
        self.path = msg
        self.heading_pid.reset(reset_integral=True, reset_derivative=True)
        self.cross_track_pid.reset(reset_integral=True, reset_derivative=True)
        self.transform_plan()

    def update_controls(self):
        if not self.path.poses:
            self.publish_cmd_vel((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            return

        dt = (self.get_clock().now() - self.last_cycle_time).nanoseconds * 1e-9

        try:
            robot_pose = self.get_robot_pose()
            next_path_pose, second_next_path_pose = self.get_target_poses(robot_pose.pose, self.lookahead_dist)
        except ValueError as ex:
            self.get_logger().warn(f"Error performing robot/path transforms: {ex}")
            self.publish_cmd_vel((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self.last_cycle_time = self.get_clock().now()
            return

        next_path_pose, second_next_path_pose = self.get_target_poses(robot_pose.pose, self.lookahead_dist)

        along_track_error = self.get_along_track_error(robot_pose.pose, next_path_pose)
        cross_track_error = self.get_cross_track_error(robot_pose.pose, next_path_pose, second_next_path_pose)
        heading_error = self.get_heading_error(robot_pose.pose, next_path_pose, second_next_path_pose)
        test_goal_heading: float = -math.pi

        # open-loop
        desired_curvature = 0.0
        desired_velocity = 0.0
        if self.open_loop_enabled:
            desired_velocity = self.get_desired_velocity(robot_pose.pose, next_path_pose, second_next_path_pose)
            desired_curvature = self.get_desired_curvature(next_path_pose, second_next_path_pose, desired_velocity)

        # feedback
        velocity_correction = 0.0
        curvature_correction = 0.0

        if self.closed_loop_enabled:
            velocity_correction = self.ks * along_track_error
            # curvature_correction = self.kn * cross_track_error + self.ktheta * heading_error
            curvature_correction = self.heading_pid.update(heading_error, dt) + self.cross_track_pid.update(
                cross_track_error, dt
            )

        output_linear_velocity = desired_velocity + velocity_correction
        output_curvature = desired_curvature + curvature_correction
        # output_angular_velocity = output_linear_velocity * output_curvature
        output_angular_velocity = self.heading_pid.update(heading_error, dt)

        final_velocity = max(-self.max_linear_velocity, min(self.max_linear_velocity, output_linear_velocity))
        # angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, output_angular_velocity))
        angular_velocity = output_angular_velocity

        # DEBUG
        self.debug_distance_from_goal.publish(
            Float64(data=self.get_pose_error(robot_pose.pose, list(self.path.poses)[0].pose))
        )
        self.debug_heading_error.publish(Float64(data=heading_error))
        self.debug_cross_track_error.publish(Float64(data=cross_track_error))
        self.debug_goal_heading.publish(Float64(data=test_goal_heading))
        self.debug_robot_heading.publish(Float64(data=self.get_robot_heading(robot_pose.pose)))

        if self.check_reached_goal(robot_pose.pose, 1) and self.goal_reached is False:
            self.get_logger().info("Goal reached!")
            self.goal_reached = True
            self.publish_cmd_vel((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        elif self.goal_reached is True:
            self.publish_cmd_vel((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        else:
            self.publish_cmd_vel((final_velocity, 0.0, 0.0), (0.0, 0.0, angular_velocity))

        # Go to goal if outside replan radius
        if self.goal_reached and not self.check_reached_goal(robot_pose.pose, 5):
            self.goal_reached = False
        # self.publish_cmd_vel((0.0, 0.0, 0.0), (0.0, 0.0, angular_velocity))

        self.last_cycle_time = self.get_clock().now()

    def open_loop_controller(self):
        pass

    def get_desired_velocity(self, robot_pose: Pose, next_pose: Pose, second_next_pose: Pose | None) -> float:
        if not self.check_reached_goal(robot_pose, 7):
            return self.max_linear_velocity * 0.2
        elif second_next_pose is None or self.check_reached_goal(robot_pose, 3):
            # slow down if on last pose (not good but works for now)
            return self.max_linear_velocity * 0.2

        return self.max_linear_velocity

    def get_desired_curvature(self, next_pose: Pose, second_next_pose: Pose | None, velocity: float) -> float:
        if second_next_pose is None:
            return 0.0
        else:
            dtheta = self.get_robot_heading(second_next_pose) - self.get_robot_heading(next_pose)
            ds = math.hypot(
                second_next_pose.position.x - next_pose.position.x, second_next_pose.position.y - next_pose.position.y
            )
            return dtheta / ds

    def get_along_track_error(self, robot_pose: Pose, goal_pose: Pose) -> float:
        target_dx = goal_pose.position.x - robot_pose.position.x
        target_dy = goal_pose.position.y - robot_pose.position.y

        robot_heading = self.get_robot_heading(robot_pose)
        robot_heading_x = math.cos(robot_heading)
        robot_heading_y = math.sin(robot_heading)

        # Pos = robot behind goal; neg = robot ahead of goal
        # neg should not occur as per how we select the next local goal
        return target_dx * robot_heading_x + target_dy * robot_heading_y

    def get_cross_track_error(
        self, robot_pose: Pose, next_path_pose: Pose, second_next_path_pose: Pose | None
    ) -> float:
        # on last pose in path
        if second_next_path_pose is None:
            robot_dx = robot_pose.position.x - next_path_pose.position.x
            robot_dy = robot_pose.position.y - next_path_pose.position.y
            return math.sqrt(robot_dx**2 + robot_dy**2)

        robot_dx = robot_pose.position.x - next_path_pose.position.x
        robot_dy = robot_pose.position.y - next_path_pose.position.y

        path_dx = second_next_path_pose.position.x - next_path_pose.position.x
        path_dy = second_next_path_pose.position.y - next_path_pose.position.y
        pose_vector_mag = math.sqrt(path_dx**2 + path_dy**2)

        # Pos -> left of path; Neg -> right of path
        return (path_dx * robot_dy - robot_dx * path_dy) / pose_vector_mag

    def get_heading_error(self, robot_pose: Pose, next_path_pose: Pose, second_next_path_pose: Pose | None) -> float:
        # on last pose in path
        if second_next_path_pose is None:
            dx = next_path_pose.position.x - robot_pose.position.x
            dy = next_path_pose.position.y - robot_pose.position.y

            # if within threshold, ignore heading error
            distance_to_target = math.sqrt(dx**2 + dy**2)
            if distance_to_target < 0.1:  # within 10cm (arb chosen for now)
                return 0.0

        else:
            # Not on last pose
            dx = second_next_path_pose.position.x - next_path_pose.position.x
            dy = second_next_path_pose.position.y - next_path_pose.position.y

        desired_heading = math.atan2(dy, dx)
        robot_heading = self.get_robot_heading(robot_pose)

        # normalize heading error to [-pi, pi]
        heading_error = desired_heading - robot_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        return heading_error

    def get_robot_pose(self, target_frame: str = "map", source_frame: str = "wamv/base_link") -> PoseStamped:
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=1))

            pose = PoseStamped()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation

            pose.header.frame_id = target_frame

            return pose

        except Exception as ex:
            self.get_logger().error(f"Cannot transform from {source_frame} to {target_frame}: {ex}")
            return PoseStamped()

    def get_robot_heading(self, robot_pose: Pose) -> float:
        q = [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]

        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def transform_plan(self, target_frame: str = "map") -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, self.path.header.frame_id, Time())
        except Exception as ex:
            self.get_logger().error(
                f"Couldn't transform plan from frame {self.path.header.frame_id} to {target_frame}: {ex}"
            )
            return False

        poses: List[PoseStamped] = self.path.poses
        for poseStamped in poses:
            poseStamped.pose = do_transform_pose(poseStamped.pose, tf)
            poseStamped.header.frame_id = target_frame

        self.path.header.frame_id = target_frame
        return True

    def get_target_poses(self, robot_pose: Pose, lookahead_dist: float) -> tuple[Pose, Pose | None]:
        remaining_poses: List[PoseStamped] = list(self.path.poses)
        if not remaining_poses:
            raise ValueError("Empty Path (No poses)")

        for i, pose in enumerate(self.path.poses):
            dx = pose.pose.position.x - robot_pose.position.x
            dy = pose.pose.position.y - robot_pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance >= lookahead_dist and self.get_along_track_error(robot_pose, pose.pose) >= 0:
                self.path.poses = remaining_poses[i:]
                second_next_pose = self.path.poses[1].pose if len(self.path.poses) >= 2 else None
                return pose.pose, second_next_pose

        # else, did not find any valid poses
        self.get_logger().warn("No valid target pose found, returning pose of last waypoint")
        return remaining_poses[-1].pose, None

    def get_pose_error(self, pose1: Pose, pose2: Pose) -> float:
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx * dx + dy * dy)

    def publish_cmd_vel(self, linear: tuple[float, float, float], angular: tuple[float, float, float]):
        cmd_vel = TwistStamped()
        cmd_vel.header.frame_id = "wamv/base_link"
        cmd_vel.header.stamp = self.get_clock().now().to_msg()

        cmd_vel.twist.linear.x = linear[0]
        cmd_vel.twist.linear.y = linear[1]
        cmd_vel.twist.linear.z = linear[2]

        cmd_vel.twist.angular.x = angular[0]
        cmd_vel.twist.angular.y = angular[1]
        cmd_vel.twist.angular.z = angular[2]

        self.cmd_vel_pub.publish(cmd_vel)

    def check_reached_goal(self, robot_pose: Pose, goal_threshold: float) -> bool:
        pose_list = list(self.path.poses)
        last_pose = pose_list[-1]
        dx = last_pose.pose.position.x - robot_pose.position.x
        dy = last_pose.pose.position.y - robot_pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance <= goal_threshold


def main(args=None):  # type: ignore
    print("hi")
    rclpy.init(args=args)
    try:
        pd_motion_planner = PDMotionPlanner()
        rclpy.spin(pd_motion_planner)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
