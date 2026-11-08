from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState
import math
from seaweed_dashboard.state import DashboardState
import time

class RosNode(Node):
    
    def __init__(self, state:DashboardState):
        super().__init__("dashboard_node")   
        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.left_thurster = self.create_subscription(Float64, "/wamv/thrusters/front_left/thrust", self.left_thruster_callback, 10)
        self.right_thruster = self.create_subscription(Float64, "/wamv/thrusters/front_right/thrust", self.right_thruster_callback, 10)
        self.battery = self.create_subscription(BatteryState, "/battery", self.battery_callback, 10)
        self.state = state

    def odom_callback(self, msg: Odometry) -> None:
        linear_speed_x = msg.twist.twist.linear.x
        linear_speed_y = msg.twist.twist.linear.y
        linear_speed = math.hypot(linear_speed_x, linear_speed_y)

        quaternion_x = msg.pose.pose.orientation.x
        quaternion_y = msg.pose.pose.orientation.y
        quaternion_z = msg.pose.pose.orientation.z
        quaternion_w = msg.pose.pose.orientation.w
        yaw_radians = math.atan2(2*(quaternion_w*quaternion_z + quaternion_x*quaternion_y), 1 - 2*(quaternion_y**2 + quaternion_z**2))   
        yaw_degrees = math.degrees(yaw_radians)
        compass = (90 - round(yaw_degrees)) % 360

        #acquire-release mechanism
        with self.state.lock:
            self.state.speed = linear_speed
            self.state.speed_time = time.monotonic()
            self.state.heading = compass
            self.state.heading_time = time.monotonic()
            

    def left_thruster_callback(self, msg: Float64):
        left_thrust = msg.data/10
        
        with self.state.lock:
            self.state.left_thrust = left_thrust
            self.state.left_thrust_time = time.monotonic()

    def right_thruster_callback(self, msg: Float64):
        right_thrust = msg.data/10
        
        with self.state.lock:
            self.state.right_thrust = right_thrust
            self.state.right_thrust_time = time.monotonic()

    def battery_callback(self, msg: BatteryState):
        if math.isnan(msg.percentage): 
            battery = None 
        else: 
            battery = msg.percentage*100

        with self.state.lock:
            self.state.battery = battery
            self.state.battery_time = time.monotonic()