# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'base_station_dashboard.ui'
##
## Created by: Qt User Interface Compiler version 6.12.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QCheckBox, QComboBox,
    QDoubleSpinBox, QFrame, QGraphicsView, QGridLayout,
    QHBoxLayout, QLCDNumber, QLabel, QMainWindow,
    QPlainTextEdit, QPushButton, QSizePolicy, QTextBrowser,
    QVBoxLayout, QWidget)

class Ui_BaseStationDashboard(object):
    def setupUi(self, BaseStationDashboard):
        if not BaseStationDashboard.objectName():
            BaseStationDashboard.setObjectName(u"BaseStationDashboard")
        BaseStationDashboard.resize(1445, 756)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(BaseStationDashboard.sizePolicy().hasHeightForWidth())
        BaseStationDashboard.setSizePolicy(sizePolicy)
        BaseStationDashboard.setLayoutDirection(Qt.LayoutDirection.LeftToRight)
        BaseStationDashboard.setStyleSheet(u"/* ============================================================================\n"
"   DASHBOARD - MINIMAL STYLING\n"
"   Keep all elements visible, just style colors and fonts\n"
"   ============================================================================ */\n"
"\n"
"QMainWindow {\n"
"    background-color: #003594;\n"
"}\n"
"\n"
"QWidget {\n"
"    background-color: #003594;\n"
"    color: #FFFFFF;\n"
"    font-size: 13pt;\n"
"    font-family: \"Segoe UI\";\n"
"}\n"
"\n"
"QFrame {\n"
"    background-color: #003594;\n"
"    border: 3px solid #FFB81C;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: #FFFFFF;\n"
"    background-color: transparent;\n"
"    font-size: 13pt;\n"
"    font-weight: bold;\n"
"	border: none;\n"
"}\n"
"\n"
"/* Buttons */\n"
"QPushButton {\n"
"    background-color: #003594;\n"
"    color: #FFFFFF;\n"
"    border: 3px solid #FFB81C;\n"
"    padding: 8px 12px;\n"
"    font-size: 12pt;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: #001F50;\n"
"    border: 3"
                        "px solid #FFFFFF;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #FFB81C;\n"
"    color: #003594;\n"
"}\n"
"\n"
"/* E-Stop Button */\n"
"#soft_estop {\n"
"    background-color: #E74C3C;\n"
"    color: #FFFFFF;\n"
"    border: 3px solid #FFFFFF;\n"
"}\n"
"\n"
"#soft_estop:hover {\n"
"    background-color: #C0392B;\n"
"}\n"
"\n"
"#soft_estop:pressed {\n"
"    background-color: #FFFFFF;\n"
"    color: #E74C3C;\n"
"}\n"
"\n"
"/* Spinbox */\n"
"QSpinBox, QDoubleSpinBox {\n"
"    color: #FFFFFF;\n"
"    border: 2px solid #003594;\n"
"    padding: 6px;\n"
"    font-size: 12pt;\n"
"}\n"
"\n"
"QSpinBox:focus, QDoubleSpinBox:focus {\n"
"    border: 3px solid #FFB81C;\n"
"}\n"
"\n"
"/* Combo Box - MINIMAL */\n"
"QComboBox {\n"
"    color: #FFFFFF;\n"
"    border: 2px solid #003594;\n"
"    padding: 6px;\n"
"    font-size: 12pt;\n"
"}\n"
"\n"
"QComboBox:focus {\n"
"    border: 3px solid #FFB81C;\n"
"}\n"
"\n"
"QComboBox QAbstractItemView {\n"
"    color: #FFFFFF;\n"
"    selection-background-color: #FFB81C"
                        ";\n"
"    selection-color: #003594;\n"
"    font-size: 12pt;\n"
"}\n"
"\n"
"/* Line Edit */\n"
"QLineEdit {\n"
"    color: #FFFFFF;\n"
"    border: 2px solid #003594;\n"
"    padding: 6px;\n"
"    font-size: 12pt;\n"
"}\n"
"\n"
"QLineEdit:focus {\n"
"    border: 3px solid #FFB81C;\n"
"}\n"
"\n"
"/* Text Browser - Black background */\n"
"QTextBrowser {\n"
"    background-color: #000000;\n"
"    color: #FFFFFF;\n"
"    border: 3px solid #FFB81C;\n"
"    font-size: 11pt;\n"
"}\n"
"\n"
"/* Text Edit */\n"
"QTextEdit {\n"
"    background-color: #000000;\n"
"    color: #FFFFFF;\n"
"    border: 3px solid #FFB81C;\n"
"    font-size: 11pt;\n"
"}\n"
"\n"
"/* Plain Text Edit */\n"
"QPlainTextEdit {\n"
"    background-color: #000000;\n"
"    color: #FFFFFF;\n"
"    border: 3px solid #FFB81C;\n"
"    font-size: 11pt;\n"
"}\n"
"\n"
"/* Graphics View - Black for video */\n"
"QGraphicsView {\n"
"    background-color: #000000;\n"
"    border: 3px solid #FFB81C;\n"
"}\n"
"\n"
"/* Checkboxes */\n"
"QCheckBox {\n"
"    color: #FF"
                        "FFFF;\n"
"    font-size: 12pt;\n"
"}\n"
"\n"
"QCheckBox::indicator {\n"
"    border: 2px solid #FFB81C;\n"
"    background: #003594;\n"
"}\n"
"\n"
"QCheckBox::indicator:checked {\n"
"    background-color: #FFB81C;\n"
"}\n"
"\n"
"/* Scroll Bars */\n"
"QScrollBar:vertical {\n"
"    background-color: #003594;\n"
"    width: 12px;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical {\n"
"    background-color: #FFB81C;\n"
"    border-radius: 6px;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical:hover {\n"
"    background-color: #FFFFFF;\n"
"}\n"
"\n"
"QScrollBar:horizontal {\n"
"    background-color: #003594;\n"
"    height: 12px;\n"
"}\n"
"\n"
"QScrollBar::handle:horizontal {\n"
"    background-color: #FFB81C;\n"
"    border-radius: 6px;\n"
"}\n"
"\n"
"QScrollBar::handle:horizontal:hover {\n"
"    background-color: #FFFFFF;\n"
"}\n"
"\n"
"/* Tabs */\n"
"QTabWidget::pane {\n"
"    border: 2px solid #FFB81C;\n"
"}\n"
"\n"
"QTabBar::tab {\n"
"    background: #1A4D7F;\n"
"    color: #FFFFFF;\n"
"    padding: 6px 12px;\n"
"    border"
                        ": 1px solid #FFB81C;\n"
"    font-size: 12pt;\n"
"}\n"
"\n"
"QTabBar::tab:selected {\n"
"    background: #FFB81C;\n"
"    color: #003594;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"/* Group Box */\n"
"QGroupBox {\n"
"    border: 2px solid #FFB81C;\n"
"    color: #FFB81C;\n"
"    font-size: 12pt;\n"
"    font-weight: bold;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    subcontrol-position: top center;\n"
"    padding: 0 5px;\n"
"    color: #FFB81C;\n"
"}\n"
"\n"
"QPushButton:checked { background-color: #FFB81C; color: #003594; }")
        self.central_widget = QWidget(BaseStationDashboard)
        self.central_widget.setObjectName(u"central_widget")
        sizePolicy.setHeightForWidth(self.central_widget.sizePolicy().hasHeightForWidth())
        self.central_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout = QHBoxLayout(self.central_widget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.central_frame = QFrame(self.central_widget)
        self.central_frame.setObjectName(u"central_frame")
        self.central_frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.central_frame.setFrameShadow(QFrame.Shadow.Raised)
        self.horizontalLayout_2 = QHBoxLayout(self.central_frame)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.left = QVBoxLayout()
        self.left.setObjectName(u"left")
        self.measurements_layout = QGridLayout()
        self.measurements_layout.setObjectName(u"measurements_layout")
        self.heading_layout = QVBoxLayout()
        self.heading_layout.setObjectName(u"heading_layout")
        self.heading_label = QLabel(self.central_frame)
        self.heading_label.setObjectName(u"heading_label")

        self.heading_layout.addWidget(self.heading_label)

        self.heading = QLCDNumber(self.central_frame)
        self.heading.setObjectName(u"heading")
        self.heading.setMinimumSize(QSize(0, 60))
        self.heading.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)

        self.heading_layout.addWidget(self.heading)


        self.measurements_layout.addLayout(self.heading_layout, 0, 1, 1, 1)

        self.left_thruster_layout = QVBoxLayout()
        self.left_thruster_layout.setObjectName(u"left_thruster_layout")
        self.left_thruster_label = QLabel(self.central_frame)
        self.left_thruster_label.setObjectName(u"left_thruster_label")

        self.left_thruster_layout.addWidget(self.left_thruster_label)

        self.left_thruster = QLCDNumber(self.central_frame)
        self.left_thruster.setObjectName(u"left_thruster")
        self.left_thruster.setMinimumSize(QSize(0, 60))
        self.left_thruster.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)

        self.left_thruster_layout.addWidget(self.left_thruster)


        self.measurements_layout.addLayout(self.left_thruster_layout, 1, 0, 1, 1)

        self.battery_percentage_layout = QVBoxLayout()
        self.battery_percentage_layout.setObjectName(u"battery_percentage_layout")
        self.battery_percentage_label = QLabel(self.central_frame)
        self.battery_percentage_label.setObjectName(u"battery_percentage_label")

        self.battery_percentage_layout.addWidget(self.battery_percentage_label)

        self.battery = QLCDNumber(self.central_frame)
        self.battery.setObjectName(u"battery")
        self.battery.setMinimumSize(QSize(0, 60))
        self.battery.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)

        self.battery_percentage_layout.addWidget(self.battery)


        self.measurements_layout.addLayout(self.battery_percentage_layout, 0, 2, 1, 1)

        self.right_thruster_layout = QVBoxLayout()
        self.right_thruster_layout.setObjectName(u"right_thruster_layout")
        self.right_thruster_label = QLabel(self.central_frame)
        self.right_thruster_label.setObjectName(u"right_thruster_label")

        self.right_thruster_layout.addWidget(self.right_thruster_label)

        self.right_thruster = QLCDNumber(self.central_frame)
        self.right_thruster.setObjectName(u"right_thruster")
        self.right_thruster.setMinimumSize(QSize(0, 60))
        self.right_thruster.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)

        self.right_thruster_layout.addWidget(self.right_thruster)


        self.measurements_layout.addLayout(self.right_thruster_layout, 1, 1, 1, 1)

        self.heartbeat_status_layout = QVBoxLayout()
        self.heartbeat_status_layout.setObjectName(u"heartbeat_status_layout")
        self.heartbeat_label = QLabel(self.central_frame)
        self.heartbeat_label.setObjectName(u"heartbeat_label")

        self.heartbeat_status_layout.addWidget(self.heartbeat_label)

        self.heartbeat = QLCDNumber(self.central_frame)
        self.heartbeat.setObjectName(u"heartbeat")
        self.heartbeat.setMinimumSize(QSize(0, 60))
        self.heartbeat.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)

        self.heartbeat_status_layout.addWidget(self.heartbeat)


        self.measurements_layout.addLayout(self.heartbeat_status_layout, 1, 2, 1, 1)

        self.speed_layout = QVBoxLayout()
        self.speed_layout.setObjectName(u"speed_layout")
        self.speed_label = QLabel(self.central_frame)
        self.speed_label.setObjectName(u"speed_label")
        self.speed_label.setFrameShape(QFrame.Shape.NoFrame)
        self.speed_label.setFrameShadow(QFrame.Shadow.Plain)

        self.speed_layout.addWidget(self.speed_label)

        self.speed = QLCDNumber(self.central_frame)
        self.speed.setObjectName(u"speed")
        self.speed.setMinimumSize(QSize(0, 60))
        font = QFont()
        font.setFamilies([u"Segoe UI"])
        font.setPointSize(13)
        font.setBold(False)
        self.speed.setFont(font)
        self.speed.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)

        self.speed_layout.addWidget(self.speed)


        self.measurements_layout.addLayout(self.speed_layout, 0, 0, 1, 1)


        self.left.addLayout(self.measurements_layout)

        self.boat_coordinates_layout = QHBoxLayout()
        self.boat_coordinates_layout.setObjectName(u"boat_coordinates_layout")
        self.boat_coordinates_label = QLabel(self.central_frame)
        self.boat_coordinates_label.setObjectName(u"boat_coordinates_label")
        self.boat_coordinates_label.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.boat_coordinates_layout.addWidget(self.boat_coordinates_label)

        self.x_coordinates = QLabel(self.central_frame)
        self.x_coordinates.setObjectName(u"x_coordinates")
        self.x_coordinates.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.boat_coordinates_layout.addWidget(self.x_coordinates)

        self.y_coordinates = QLabel(self.central_frame)
        self.y_coordinates.setObjectName(u"y_coordinates")

        self.boat_coordinates_layout.addWidget(self.y_coordinates)

        self.boat_coordinates_layout.setStretch(0, 2)
        self.boat_coordinates_layout.setStretch(1, 1)
        self.boat_coordinates_layout.setStretch(2, 1)

        self.left.addLayout(self.boat_coordinates_layout)

        self.boat_map = QGraphicsView(self.central_frame)
        self.boat_map.setObjectName(u"boat_map")

        self.left.addWidget(self.boat_map)

        self.live_camera_feed = QGraphicsView(self.central_frame)
        self.live_camera_feed.setObjectName(u"live_camera_feed")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.live_camera_feed.sizePolicy().hasHeightForWidth())
        self.live_camera_feed.setSizePolicy(sizePolicy1)

        self.left.addWidget(self.live_camera_feed)


        self.horizontalLayout_2.addLayout(self.left)

        self.right = QVBoxLayout()
        self.right.setObjectName(u"right")
        self.top_right = QGridLayout()
        self.top_right.setObjectName(u"top_right")
        self.manual_override = QPushButton(self.central_frame)
        self.manual_override.setObjectName(u"manual_override")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.manual_override.sizePolicy().hasHeightForWidth())
        self.manual_override.setSizePolicy(sizePolicy2)
        self.manual_override.setMinimumSize(QSize(40, 60))
        self.manual_override.setStyleSheet(u"")
        self.manual_override.setCheckable(True)

        self.top_right.addWidget(self.manual_override, 0, 1, 1, 1)

        self.set_test_layout_label = QVBoxLayout()
        self.set_test_layout_label.setObjectName(u"set_test_layout_label")
        self.set_test_event_label = QLabel(self.central_frame)
        self.set_test_event_label.setObjectName(u"set_test_event_label")

        self.set_test_layout_label.addWidget(self.set_test_event_label)

        self.test_event = QComboBox(self.central_frame)
        self.test_event.addItem("")
        self.test_event.addItem("")
        self.test_event.addItem("")
        self.test_event.addItem("")
        self.test_event.addItem("")
        self.test_event.setObjectName(u"test_event")
        sizePolicy.setHeightForWidth(self.test_event.sizePolicy().hasHeightForWidth())
        self.test_event.setSizePolicy(sizePolicy)
        self.test_event.setEditable(False)
        self.test_event.setMaxCount(10)
        self.test_event.setInsertPolicy(QComboBox.InsertPolicy.InsertAtTop)
        self.test_event.setMinimumContentsLength(5)
        self.test_event.setModelColumn(0)

        self.set_test_layout_label.addWidget(self.test_event)


        self.top_right.addLayout(self.set_test_layout_label, 0, 0, 1, 1)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.set_origin_label = QLabel(self.central_frame)
        self.set_origin_label.setObjectName(u"set_origin_label")

        self.verticalLayout_4.addWidget(self.set_origin_label)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.set_origin_x = QDoubleSpinBox(self.central_frame)
        self.set_origin_x.setObjectName(u"set_origin_x")
        self.set_origin_x.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.PlusMinus)
        self.set_origin_x.setMinimum(-10000.000000000000000)
        self.set_origin_x.setMaximum(10000.000000000000000)

        self.horizontalLayout_6.addWidget(self.set_origin_x)

        self.set_origin_y = QDoubleSpinBox(self.central_frame)
        self.set_origin_y.setObjectName(u"set_origin_y")
        self.set_origin_y.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.PlusMinus)
        self.set_origin_y.setMinimum(-10000.000000000000000)
        self.set_origin_y.setMaximum(10000.000000000000000)

        self.horizontalLayout_6.addWidget(self.set_origin_y)


        self.verticalLayout_4.addLayout(self.horizontalLayout_6)


        self.top_right.addLayout(self.verticalLayout_4, 1, 1, 1, 1)

        self.set_waypoint = QVBoxLayout()
        self.set_waypoint.setObjectName(u"set_waypoint")
        self.set_waypoint_label = QLabel(self.central_frame)
        self.set_waypoint_label.setObjectName(u"set_waypoint_label")

        self.set_waypoint.addWidget(self.set_waypoint_label)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.set_waypoint_x = QDoubleSpinBox(self.central_frame)
        self.set_waypoint_x.setObjectName(u"set_waypoint_x")
        self.set_waypoint_x.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.PlusMinus)
        self.set_waypoint_x.setMinimum(-10000.000000000000000)
        self.set_waypoint_x.setMaximum(10000.000000000000000)

        self.horizontalLayout_4.addWidget(self.set_waypoint_x)

        self.set_waypoint_y = QDoubleSpinBox(self.central_frame)
        self.set_waypoint_y.setObjectName(u"set_waypoint_y")
        self.set_waypoint_y.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.PlusMinus)
        self.set_waypoint_y.setMinimum(-10000.000000000000000)
        self.set_waypoint_y.setMaximum(10000.000000000000000)

        self.horizontalLayout_4.addWidget(self.set_waypoint_y)


        self.set_waypoint.addLayout(self.horizontalLayout_4)


        self.top_right.addLayout(self.set_waypoint, 1, 0, 1, 1)


        self.right.addLayout(self.top_right)

        self.send_coordinates_reset = QHBoxLayout()
        self.send_coordinates_reset.setObjectName(u"send_coordinates_reset")
        self.confirm_waypoint = QPushButton(self.central_frame)
        self.confirm_waypoint.setObjectName(u"confirm_waypoint")

        self.send_coordinates_reset.addWidget(self.confirm_waypoint)

        self.reset = QPushButton(self.central_frame)
        self.reset.setObjectName(u"reset")

        self.send_coordinates_reset.addWidget(self.reset)

        self.confirm_origin = QPushButton(self.central_frame)
        self.confirm_origin.setObjectName(u"confirm_origin")

        self.send_coordinates_reset.addWidget(self.confirm_origin)


        self.right.addLayout(self.send_coordinates_reset)

        self.middle_right = QHBoxLayout()
        self.middle_right.setObjectName(u"middle_right")
        self.return_home = QPushButton(self.central_frame)
        self.return_home.setObjectName(u"return_home")

        self.middle_right.addWidget(self.return_home)

        self.enable_estop = QCheckBox(self.central_frame)
        self.enable_estop.setObjectName(u"enable_estop")
        self.enable_estop.setLayoutDirection(Qt.LayoutDirection.LeftToRight)

        self.middle_right.addWidget(self.enable_estop)

        self.soft_estop = QPushButton(self.central_frame)
        self.soft_estop.setObjectName(u"soft_estop")
        self.soft_estop.setAutoFillBackground(False)
        self.soft_estop.setStyleSheet(u"")

        self.middle_right.addWidget(self.soft_estop)


        self.right.addLayout(self.middle_right)

        self.terminal_console = QPlainTextEdit(self.central_frame)
        self.terminal_console.setObjectName(u"terminal_console")

        self.right.addWidget(self.terminal_console)

        self.MiscTextLabel = QLabel(self.central_frame)
        self.MiscTextLabel.setObjectName(u"MiscTextLabel")

        self.right.addWidget(self.MiscTextLabel)

        self.misc_text_log = QTextBrowser(self.central_frame)
        self.misc_text_log.setObjectName(u"misc_text_log")

        self.right.addWidget(self.misc_text_log)


        self.horizontalLayout_2.addLayout(self.right)


        self.horizontalLayout.addWidget(self.central_frame)

        BaseStationDashboard.setCentralWidget(self.central_widget)

        self.retranslateUi(BaseStationDashboard)

        QMetaObject.connectSlotsByName(BaseStationDashboard)
    # setupUi

    def retranslateUi(self, BaseStationDashboard):
        BaseStationDashboard.setWindowTitle(QCoreApplication.translate("BaseStationDashboard", u"SeaWeed Base Station", None))
        self.heading_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Heading (\u00b0)", None))
        self.left_thruster_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Left Thruster (%)", None))
        self.battery_percentage_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Battery (%)", None))
        self.right_thruster_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Right Thruster (%)", None))
        self.heartbeat_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Heartbeat Status", None))
        self.speed_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Speed (m/s)", None))
        self.boat_coordinates_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Boat Coordinates", None))
        self.x_coordinates.setText(QCoreApplication.translate("BaseStationDashboard", u"X: ", None))
        self.y_coordinates.setText(QCoreApplication.translate("BaseStationDashboard", u"Y: ", None))
        self.manual_override.setText(QCoreApplication.translate("BaseStationDashboard", u"Automatic/Manual Override", None))
        self.set_test_event_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Set Test Event", None))
        self.test_event.setItemText(0, QCoreApplication.translate("BaseStationDashboard", u"Event 1", None))
        self.test_event.setItemText(1, QCoreApplication.translate("BaseStationDashboard", u"Event 2", None))
        self.test_event.setItemText(2, QCoreApplication.translate("BaseStationDashboard", u"Event 3", None))
        self.test_event.setItemText(3, QCoreApplication.translate("BaseStationDashboard", u"Event 4", None))
        self.test_event.setItemText(4, QCoreApplication.translate("BaseStationDashboard", u"Event 5", None))

        self.test_event.setCurrentText(QCoreApplication.translate("BaseStationDashboard", u"Event 1", None))
        self.set_origin_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Set Origin", None))
        self.set_origin_x.setPrefix(QCoreApplication.translate("BaseStationDashboard", u"X: ", None))
        self.set_origin_y.setPrefix(QCoreApplication.translate("BaseStationDashboard", u"Y: ", None))
        self.set_waypoint_label.setText(QCoreApplication.translate("BaseStationDashboard", u"Set Waypoint/Destination", None))
        self.set_waypoint_x.setPrefix(QCoreApplication.translate("BaseStationDashboard", u"X: ", None))
        self.set_waypoint_y.setPrefix(QCoreApplication.translate("BaseStationDashboard", u"Y: ", None))
        self.confirm_waypoint.setText(QCoreApplication.translate("BaseStationDashboard", u"Confirm Waypoint", None))
        self.reset.setText(QCoreApplication.translate("BaseStationDashboard", u"Reset", None))
        self.confirm_origin.setText(QCoreApplication.translate("BaseStationDashboard", u"Confirm Origin", None))
        self.return_home.setText(QCoreApplication.translate("BaseStationDashboard", u"Return Home", None))
        self.enable_estop.setText(QCoreApplication.translate("BaseStationDashboard", u"Enable E-Stop", None))
        self.soft_estop.setText(QCoreApplication.translate("BaseStationDashboard", u"Soft E-Stop", None))
        self.MiscTextLabel.setText(QCoreApplication.translate("BaseStationDashboard", u"Misc Text Log", None))
    # retranslateUi

