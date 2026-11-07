from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from seaweed_dashboard.state import DashboardState
import time

class RosNode(Node):
    
    def __init__(self, state:DashboardState):
        super().__init__("dashboard_node")   
        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.state = state

    def odom_callback(self, msg: Odometry) -> None:
        linear_speed_x = msg.twist.twist.linear.x
        linear_speed_y = msg.twist.twist.linear.y
        linear_speed = math.hypot(linear_speed_x, linear_speed_y)

        #acquire-release mechanism
        with self.state.lock:
            self.state.speed = linear_speed
            self.state.speed_time = time.monotonic()
            
        self.get_logger().info(f"Linear speed is {linear_speed:.2f}")

