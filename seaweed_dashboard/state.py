import threading

class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.speed = None
        self.speed_time = None
    