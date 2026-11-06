import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from seaweed_dashboard.ui_base_station_dashboard import Ui_BaseStationDashboard

class Dashboard(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_BaseStationDashboard()
        self.ui.setupUi(self)
        self.ui.soft_estop.clicked.connect(self.on_estop)

    def on_estop(self):
        print("E-stop pressed")

app = QApplication(sys.argv)
window = Dashboard()
window.show()
sys.exit(app.exec())

