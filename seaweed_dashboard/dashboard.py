import sys, time
from PySide6.QtWidgets import QApplication, QMainWindow, QLCDNumber, QLabel, QGraphicsScene, QMessageBox
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
from sensor_msgs.msg import Image
from seaweed_dashboard.ui_base_station_dashboard import Ui_BaseStationDashboard
import threading
from seaweed_dashboard.ros_node import RosNode
import rclpy, rclpy.utilities
from rclpy.executors import SingleThreadedExecutor
from seaweed_dashboard.state import DashboardState
from seaweed_dashboard.terminal import BashTerminal
from seaweed_dashboard.map_view import BoatMap
import queue

GOAL_ACCEPT_TIMEOUT_S = 10.0


class Dashboard(QMainWindow):
    def __init__(self, state:DashboardState, ros_node:RosNode) -> None:
        super().__init__()
        self.ui = Ui_BaseStationDashboard()
        self.ui.setupUi(self)
        self.ui.soft_estop.clicked.connect(self.on_estop)
        self.timer = QTimer(self)
        self.state = state
        self.ros_node = ros_node
        self.home: tuple[float, float] | None = None
        # x, y, planner's goal count when sent, time sent
        self.pending_goal: tuple[float, float, int, float] | None = None
        self.timer.timeout.connect(self.refresh)
        self.timer.setInterval(100)
        self.timer.start()
        self.ui.misc_text_log.document().setMaximumBlockCount(500)
        self.camera_scene = QGraphicsScene(self)
        self.ui.live_camera_feed.setScene(self.camera_scene)
        self.camera_item = self.camera_scene.addPixmap(QPixmap())
        self.last_camera_time = None
        self.map = BoatMap(self.ui.boat_map)
        self.ui.left.setStretchFactor(self.ui.boat_map, 1)
        self.ui.left.setStretchFactor(self.ui.live_camera_feed, 1)
        self.ui.confirm_waypoint.clicked.connect(self.confirm_waypoint)
        self.ui.confirm_origin.clicked.connect(self.confirm_origin)
        self.ui.return_home.clicked.connect(self.return_home)
        self.ui.reset.clicked.connect(self.reset)
        self.terminal = BashTerminal(self.ui.terminal_console)
        self.log_action("Dashboard started")

    def on_estop(self):
        self.log_action("Soft E-Stop pressed")

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
            camera_msg = self.state.camera_msg
            camera_time = self.state.camera_time
            nav_goals_started = self.state.nav_goals_started

        self.refresh_helper(self.ui.speed, speed, speed_time, 2, 2)
        self.refresh_helper(self.ui.left_thruster, left_thrust, left_thrust_time, 0, 2)
        self.refresh_helper(self.ui.right_thruster, right_thrust, right_thrust_time, 0, 2)
        self.refresh_helper(self.ui.heading, heading, heading_time, 0, 2)
        self.refresh_helper(self.ui.battery, battery, battery_time, 0, 10)

        heartbeat = [n for n in [speed_time, left_thrust_time, right_thrust_time, heading_time, battery_time, pose_time, camera_time] if n is not None]
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

        self.camera_update(camera_msg, camera_time)
        self.map.update_boat(x, y, heading, pose_time)
        self.check_pending_goal(nav_goals_started)

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

    def camera_update(self, msg: Image | None, msg_time: float | None):
        if msg is None or msg_time is None:
            return
        
        if time.monotonic() - msg_time > 2:
            self.camera_item.setPixmap(QPixmap())
            return

        if msg_time == self.last_camera_time:
            return
        self.last_camera_time = msg_time

        if msg.encoding == "bgr8":
            the_format = QImage.Format.Format_BGR888
        elif msg.encoding == "rgb8":
            the_format = QImage.Format.Format_RGB888
        else:
            return

        image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, the_format).copy()
        self.camera_item.setPixmap(QPixmap.fromImage(image))
        self.ui.live_camera_feed.fitInView(self.camera_item, Qt.AspectRatioMode.IgnoreAspectRatio)

    def log_action(self, text: str):
        stamp = time.strftime("%H:%M:%S")
        self.ui.misc_text_log.append(f"[{stamp}] {text}")

    def confirm_waypoint(self):
        self.send_waypoint("Send waypoint", self.ui.set_waypoint_x.value(), self.ui.set_waypoint_y.value())

    def confirm_origin(self):
        x = self.ui.set_origin_x.value()
        y = self.ui.set_origin_y.value()
        self.home = (x, y)
        self.map.set_home(x, y)
        self.log_action(f"Origin set to X: {x:.2f}, Y: {y:.2f}")

    def return_home(self):
        if self.home is None:
            self.log_action("Return Home ignored: set an origin first")
            return
        self.send_waypoint("Return home", self.home[0], self.home[1])

    def reset(self):
        self.ui.set_waypoint_x.setValue(0)
        self.ui.set_waypoint_y.setValue(0)
        self.ui.set_origin_x.setValue(0)
        self.ui.set_origin_y.setValue(0)
        self.map.clear_trail()
        self.log_action("Coordinate fields and map trail cleared")

    def send_waypoint(self, title: str, x: float, y: float):
        answer = QMessageBox.question(
            self,
            title,
            f"Send the boat to X: {x:.2f}, Y: {y:.2f}?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        with self.state.lock:
            nav_busy = self.state.nav_busy
            goals_started = self.state.nav_goals_started

        if self.pending_goal is not None:
            self.log_action("Waypoint not sent: still waiting for the planner to answer the last one")
            return
        if nav_busy:
            self.log_action("Waypoint not sent: the boat is still on its current goal")
            return

        listeners = self.ros_node.send_goal(x, y)
        if listeners == 0:
            self.log_action(f"Waypoint X: {x:.2f}, Y: {y:.2f} published, but nothing is listening on /goal_pose")
            return

        self.pending_goal = (x, y, goals_started, time.monotonic())
        self.log_action(f"Waypoint X: {x:.2f}, Y: {y:.2f} requested")

    def check_pending_goal(self, goals_started: int):
        if self.pending_goal is None:
            return

        x, y, goals_when_sent, sent_time = self.pending_goal
        if goals_started > goals_when_sent:
            self.map.set_waypoint(x, y)
            self.log_action(f"Waypoint X: {x:.2f}, Y: {y:.2f} accepted by the planner")
            self.pending_goal = None
        elif time.monotonic() - sent_time > GOAL_ACCEPT_TIMEOUT_S:
            self.log_action(f"Waypoint X: {x:.2f}, Y: {y:.2f} NOT accepted - the planner never started it")
            self.pending_goal = None



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
        window = Dashboard(state, ros_node)
        window.show()
        exit_num = app.exec()

    threader.join()
    ros_node.destroy_node()
    rclpy.utilities.try_shutdown()
    sys.exit(exit_num)

if __name__ == "__main__":
    main()
