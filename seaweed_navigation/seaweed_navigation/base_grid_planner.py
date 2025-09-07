from abc import ABC, abstractmethod

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.logging import get_logger

import math
from typing import Optional

# Contains Utils for creating grid-based path planners

class WeightedNode:
    def __init__(self, point: Point, parent: Optional["WeightedNode"] = None, cost: float = 0):
        self.cost = cost
        self.point = point
        self.parent = parent

    def __lt__(self, other: "WeightedNode") -> bool:
        return self.cost < other.cost

    def __eq__(self, other: "WeightedNode"):
        return self.point.x == other.point.x and self.point.y == other.point.y

    def __hash__(self):
        return hash((self.point.x, self.point.y))

    def __add__(self, other: "WeightedNode"):
        return WeightedNode(point=Point(x=self.point.x + other.point.x, y=self.point.y + other.point.y), cost=0)


class BasePlanner(ABC):
    def __init__(
        self,
        map: OccupancyGrid,
        # start: Point,
        # goal: Point,
        # obstacle_tol: float,
        goal_tolerance: float,
    ):
        self.map = map # OccupancyGrid()
        self.debug_map: OccupancyGrid = OccupancyGrid()
        # self.world_start = start
        # self.world_goal = goal
        # self.grid_start = self.world_to_grid(start)
        # self.grid_goal = self.world_to_grid(goal)
        # self.obstacle_tol = obstacle_tol
        self.goal_tolerance = goal_tolerance
        self.logger = get_logger("base_path_planner")

    @abstractmethod
    def plan(self) -> Optional[Path]:
        pass

    def world_to_grid(self, world_point: Point) -> Point:
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        grid_x = (world_point.x - origin.x) // resolution
        grid_y = (world_point.y - origin.y) // resolution
        return Point(x=grid_x, y=grid_y)

    def grid_to_world(self, grid_point: Point) -> Point:
        origin = self.map.info.origin.position
        resolution = self.map.info.resolution
        world_x = (grid_point.x * resolution) + origin.x
        world_y = (grid_point.y * resolution) + origin.y
        return Point(x=world_x, y=world_y)

    def is_goal_reached(self, current_point: Point, goal_point: Point) -> bool:
        goal_tolerance_grid_frame = self.goal_tolerance / self.map.info.resolution
        return math.hypot(current_point.x - goal_point.x, current_point.y - goal_point.y) < goal_tolerance_grid_frame

    def is_in_bounds(self, grid_point: Point) -> bool:
        return 0 <= grid_point.x < self.map.info.width and 0 <= grid_point.y < self.map.info.height

    def is_traversable(self, grid_point: Point) -> bool:
        grid_val = self.get_grid_val(grid_point)
        return 0 <= grid_val < 99

    def get_grid_val(self, grid_point: Point) -> int:
        # alr validate this earlier in a* so can remove perchance
        if not self.is_in_bounds(grid_point):
            return -1
        return self.map.data[self.get_grid_index(grid_point)]

    def get_grid_index(self, grid_point: Point) -> int:
        return int(grid_point.x + grid_point.y * self.map.info.width)
