import threading
import queue

class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.speed: float | None = None
        self.speed_time: float | None = None
        self.left_thrust: float | None = None
        self.left_thrust_time: float | None = None
        self.right_thrust: float | None = None
        self.right_thrust_time: float | None = None
        self.heading: float | None = None
        self.heading_time: float | None = None
        self.battery: float | None = None
        self.battery_time: float | None = None
        self.x: float | None = None
        self.y: float | None = None
        self.pose_time: float | None = None
        self.log_queue: queue.Queue[str] = queue.Queue()