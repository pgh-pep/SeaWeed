import sys, time
from PySide6.QtWidgets import QApplication, QMainWindow, QLCDNumber, QLabel
from PySide6.QtCore import QTimer
from seaweed_dashboard.ui_base_station_dashboard import Ui_BaseStationDashboard
import threading
from seaweed_dashboard.ros_node import RosNode
import rclpy, rclpy.utilities
from rclpy.executors import SingleThreadedExecutor
from seaweed_dashboard.state import DashboardState
import queue


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
        self.ui.misc_text_log.document().setMaximumBlockCount(500)

    def on_estop(self):
        print("E-stop pressed")

    def refresh(self):
        #acquire-release mechanism with ROS
        with self.state.lock:
            speed = self.state.speed
            speed_time = self.state.speed_time 
            left_thrust = self.state.left_thrust
            left_thrust_time = self.state.left_thrust_time
            right_thrust = self.state.right_thrust
            right_thrust_time = self.state.right_thrust_time
            heading = self.state.heading
            heading_time = self.state.heading_time
            battery = self.state.battery
            battery_time = self.state.battery_time
            x = self.state.x
            y = self.state.y
            pose_time = self.state.pose_time
            
        self.refresh_helper(self.ui.speed, speed, speed_time, 2, 2)
        self.refresh_helper(self.ui.left_thruster, left_thrust, left_thrust_time, 0, 2)
        self.refresh_helper(self.ui.right_thruster, right_thrust, right_thrust_time, 0, 2)
        self.refresh_helper(self.ui.heading, heading, heading_time, 0, 2)
        self.refresh_helper(self.ui.battery, battery, battery_time, 0, 10)

        heartbeat = [n for n in [speed_time, left_thrust_time, right_thrust_time, heading_time, battery_time, pose_time] if n is not None]
        if not heartbeat: 
            self.ui.heartbeat.display("----")
        else:
            self.ui.heartbeat.display(f"{min((time.monotonic() - max(heartbeat)), 999.9):.1f}")

        self.refresh_label_helper(self.ui.x_coordinates, "X: ", x, pose_time)
        self.refresh_label_helper(self.ui.y_coordinates, "Y: ", y, pose_time)

        while True:
            try:
                line = self.state.log_queue.get_nowait()
            except queue.Empty:
                break
            self.ui.misc_text_log.append(line)

    def refresh_helper(self, lcd: QLCDNumber, value: float | None, value_time: float | None, decimals:int, stale_time:int):
        if value is None or value_time is None:
            lcd.display("----")
        elif time.monotonic() - value_time > stale_time:         
            lcd.display("----")
        else:
            lcd.display(f"{value:.{decimals}f}") 

    def refresh_label_helper(self, label: QLabel, prefix: str, value: float | None, value_time: float | None):
        if value is None or value_time is None or (time.monotonic() - value_time) > 2:
            label.setText(f"{prefix}----")
        else:
            label.setText(f"{prefix}{value:.1f}")


def main():
    state = DashboardState()

    rclpy.init()
    with SingleThreadedExecutor() as executor:
        #Thread starts with ROS Node
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
    rclpy.utilities.try_shutdown()
    sys.exit(exit_num)

if __name__ == "__main__":
    main()
