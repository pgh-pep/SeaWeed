from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import Log
import math
from seaweed_dashboard.state import DashboardState
import time
from tf2_ros import Buffer, TransformListener, TransformException # pyright: ignore[reportAttributeAccessIssue]
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy

class RosNode(Node):
    
    def __init__(self, state:DashboardState):
        super().__init__("dashboard_node")  

        self.state = state

        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.left_thurster = self.create_subscription(Float64, "/wamv/thrusters/front_left/thrust", self.left_thruster_callback, 10)
        self.right_thruster = self.create_subscription(Float64, "/wamv/thrusters/front_right/thrust", self.right_thruster_callback, 10)
        self.battery = self.create_subscription(BatteryState, "/battery", self.battery_callback, 10)
        
        self.tf_buffer = Buffer()
        self.transform = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tf_callback)
        self.last_tf_stamp = None

        log_qos = QoSProfile(depth = 100, durability = DurabilityPolicy.TRANSIENT_LOCAL)
        self.log_sub = self.create_subscription(Log, "/rosout", self.log_callback, log_qos)

       
    def odom_callback(self, msg: Odometry) -> None:
        linear_speed_x = msg.twist.twist.linear.x
        linear_speed_y = msg.twist.twist.linear.y
        linear_speed = math.hypot(linear_speed_x, linear_speed_y)

        #acquire-release mechanism
        with self.state.lock:
            self.state.speed = linear_speed
            self.state.speed_time = time.monotonic()
            

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

    def tf_callback(self):
        try:
            tf = self.tf_buffer.lookup_transform("map" ,"wamv/base_link", Time())
        except TransformException:
            return

        quaternion_x = tf.transform.rotation.x
        quaternion_y = tf.transform.rotation.y
        quaternion_z = tf.transform.rotation.z
        quaternion_w = tf.transform.rotation.w
        yaw_radians = math.atan2(2*(quaternion_w*quaternion_z + quaternion_x*quaternion_y), 1 - 2*(quaternion_y**2 + quaternion_z**2))   
        yaw_degrees = math.degrees(yaw_radians)
        compass = (90 - round(yaw_degrees)) % 360

        if tf.header.stamp == self.last_tf_stamp:
            return
        else:
            with self.state.lock:
                self.state.x = tf.transform.translation.x
                self.state.y = tf.transform.translation.y
                self.state.pose_time = time.monotonic()
                self.state.heading = compass
                self.state.heading_time = time.monotonic()

        self.last_tf_stamp = tf.header.stamp

    def log_callback(self, msg: Log):
        if msg.level < 30:
            return
        elif msg.name == self.get_name():
            return
        else:
            self.state.log_queue.put(f"[{msg.name}] {msg.msg}")
        
