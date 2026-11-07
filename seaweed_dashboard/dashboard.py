import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer
from seaweed_dashboard.ui_base_station_dashboard import Ui_BaseStationDashboard
import threading
from seaweed_dashboard.ros_node import RosNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from seaweed_dashboard.state import DashboardState
import time


class Dashboard(QMainWindow):
    def __init__(self, state:DashboardState) -> None:
        super().__init__()
        self.ui = Ui_BaseStationDashboard()
        self.ui.setupUi(self)
        self.ui.soft_estop.clicked.connect(self.on_estop)
        self.timer = QTimer(self)
        self.state = state
        self.timer.timeout.connect(self.refresh)
        self.timer.setInterval(100)
        self.timer.start()

    def on_estop(self):
        print("E-stop pressed")

    def refresh(self):
        #acquire-release mechanism with ros
        with self.state.lock:
            speed = self.state.speed
            speed_time = self.state.speed_time 
        
        if speed is None or speed_time is None:
            self.ui.speed.display("----")
        elif time.monotonic() - speed_time > 2:         
            self.ui.speed.display("----")
        else:
            self.ui.speed.display(f"{speed:.2f}")


def main():
    state = DashboardState()

    rclpy.init()
    with SingleThreadedExecutor() as executor:
        #Thread starts with Ros Node
        ros_node = RosNode(state)
        executor.add_node(ros_node)
        threader = threading.Thread(target=executor.spin,daemon=True)
        threader.start()

        #Thread enters Qt Dashboard 
        app = QApplication(sys.argv)
        window = Dashboard(state)
        window.show()
        exit_num = app.exec()

    threader.join()
    ros_node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(exit_num)

if __name__ == "__main__":
    main()
