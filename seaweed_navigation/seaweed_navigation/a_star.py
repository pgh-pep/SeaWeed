from base_grid_planner import WeightedNode, BasePlanner

import heapq
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import rclpy
from typing import List, Optional, Tuple, Set

import math


class AStarPlanner(BasePlanner):
    def __init__(self, *args, **kwargs):  # type: ignore
        super().__init__(*args, **kwargs)

    def heuristic(self, point1: Point, point2: Point) -> float:
        return math.hypot(point1.x - point2.x, point1.y - point2.y)

    def a_star(self, start: Point, goal: Point) -> Optional[WeightedNode]:
        # directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        pending_nodes: List[Tuple[float, WeightedNode]] = []
        visited_nodes: Set[Tuple[float, float]] = set()

        start_point: Point = self.world_to_grid(start)
        goal_point: Point = self.world_to_grid(goal)

        start_node = WeightedNode(point=start_point, cost=0)
        heur = self.heuristic(start_point, goal_point)

        heapq.heappush(pending_nodes, (start_node.cost + heur, start_node))

        while pending_nodes and rclpy.ok():
            _, current_node = heapq.heappop(pending_nodes)

            if self.is_goal_reached(current_node.point, goal_point):
                return current_node

            current_point = (current_node.point.x, current_node.point.y)
            if current_point in visited_nodes:
                continue

            visited_nodes.add(current_point)

            for dx, dy in directions:
                new_x = current_node.point.x + dx
                new_y = current_node.point.y + dy
                new_point = Point(x=new_x, y=new_y)
                new_point_tuple = (new_x, new_y)

                if new_point_tuple in visited_nodes:
                    continue

                if not self.is_in_bounds(new_point):
                    continue

                if not self.is_traversable(new_point):
                    continue

                # NOTE: Not currently using each grid pts value in calc move cost
                # Will need to change once implementing obstacle inflation regions
                if abs(dx) + abs(dy) == 1:  # Ortho move
                    move_cost = 1.0
                else:  # Diagonal move
                    move_cost = math.sqrt(2)

                new_node_cost = current_node.cost + move_cost

                new_node = WeightedNode(point=new_point, parent=current_node, cost=new_node_cost)
                heur = self.heuristic(new_point, goal_point)

                heapq.heappush(pending_nodes, (new_node.cost + heur, new_node))

        return None

    def reconstruct_path(self, node: WeightedNode) -> Path:
        path = Path()
        path.header.frame_id = self.map.header.frame_id

        poses: List[PoseStamped] = []

        while node and node.parent and rclpy.ok():
            current_pose: PoseStamped = PoseStamped()
            current_pose.pose.position.x = self.grid_to_world(node.point).x
            current_pose.pose.position.y = self.grid_to_world(node.point).y
            current_pose.header.frame_id = self.map.header.frame_id

            poses.append(current_pose)
            node = node.parent

        poses.reverse()
        path.poses = poses

        return path

    def plan(self, start: Point, goal: Point) -> Optional[Path]:
        # start and goal inputs in world/map frame
        self.grid_start = self.world_to_grid(start)
        self.grid_goal = self.world_to_grid(goal)

        if not self.is_in_bounds(self.grid_start):
            self.logger.warn(f"Start pos {start} is out of bounds")
            return None

        if not self.is_in_bounds(self.grid_goal):
            self.logger.warn(f"Goal pos {goal} is out of bounds")
            return None

        if not self.is_traversable(self.grid_start):
            self.logger.warn(f"Start pos {start} is occupied")
            return None

        if not self.is_traversable(self.grid_goal):
            self.logger.warn(f"Goal pos {goal} is occupied")
            return None

        goal_node = self.a_star(start, goal)

        if not goal_node:
            self.logger.warn("No path found")
            return None

        path: Path = self.reconstruct_path(goal_node)
        self.logger.info(f"Path found w/ {len(path.poses)} poses")
        return path
