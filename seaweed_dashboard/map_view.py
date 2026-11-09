import math
import time
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QGraphicsEllipseItem, QGraphicsItem, QGraphicsScene, QGraphicsSimpleTextItem, QGraphicsView, QLabel


GRID_SPACING_M = 10
GRID_LINES_EACH_WAY = 50
MIN_VIEW_SPAN_M = 40.0
TRAIL_POINT_SPACING_M = 0.5
TRAIL_MAX_POINTS = 5000
STALE_AFTER_S = 2.0

BOAT_LIVE = QColor("#FFB81C")
BOAT_STALE = QColor("#777777")
TRAIL_COLOR = QColor("#4FC3F7")
WAYPOINT_COLOR = QColor("#FF5252")
HOME_COLOR = QColor("#69F0AE")
GRID_COLOR = QColor("#2A3448")
AXIS_COLOR = QColor("#55637D")


class BoatMap:

    def __init__(self, view: QGraphicsView):
        self.view = view
        self.scene = QGraphicsScene(view)
        self.scene.setSceneRect(-10000, -10000, 20000, 20000)
        view.setScene(self.scene)
        view.scale(1, -1)  # Qt's y points down and ROS's points up
        view.setRenderHint(QPainter.RenderHint.Antialiasing)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.draw_grid()

        self.trail: deque[QPointF] = deque(maxlen=TRAIL_MAX_POINTS)
        trail_pen = QPen(TRAIL_COLOR, 2)
        trail_pen.setCosmetic(True) 
        self.trail_item = self.scene.addPath(QPainterPath(), trail_pen)
        self.trail_item.setZValue(1)

        arrow = QPolygonF([QPointF(14, 0), QPointF(-9, 8), QPointF(-5, 0), QPointF(-9, -8)])
        self.boat = self.scene.addPolygon(arrow, QPen(Qt.GlobalColor.black), QBrush(BOAT_LIVE))
        self.boat.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.boat.setZValue(10)
        self.boat.hide()

        self.waypoint = self.make_marker("WP", WAYPOINT_COLOR, 0)
        self.home = self.make_marker("Home", HOME_COLOR, -16)

        self.last_fit: tuple[float, float, float, float, int, int] | None = None

        scale_label = QLabel(f"Grid: {GRID_SPACING_M} m", view)
        scale_label.setStyleSheet("color: #9FB3D1; background: transparent; font-size: 9pt;")
        scale_label.move(8, 6)
        scale_label.adjustSize()

    def draw_grid(self) -> None:
        grid_pen = QPen(GRID_COLOR)
        grid_pen.setCosmetic(True)
        axis_pen = QPen(AXIS_COLOR)
        axis_pen.setCosmetic(True)
        extent = GRID_LINES_EACH_WAY * GRID_SPACING_M

        for i in range(-GRID_LINES_EACH_WAY, GRID_LINES_EACH_WAY + 1):
            offset = i * GRID_SPACING_M
            pen = axis_pen if i == 0 else grid_pen
            self.scene.addLine(offset, -extent, offset, extent, pen).setZValue(-1)
            self.scene.addLine(-extent, offset, extent, offset, pen).setZValue(-1)

    def make_marker(self, label: str, color: QColor, label_y: int) -> QGraphicsEllipseItem:
        marker = self.scene.addEllipse(-6, -6, 12, 12, QPen(color), QBrush(color))
        marker.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        marker.setZValue(5)
        text = QGraphicsSimpleTextItem(label, marker)
        text.setBrush(QBrush(color))
        text.setPos(9, label_y)
        marker.hide()
        return marker

    def update_boat(self, x: float | None, y: float | None, heading: float | None, pose_time: float | None):
        if x is not None and y is not None and heading is not None and pose_time is not None:
            stale = time.monotonic() - pose_time > STALE_AFTER_S
            self.boat.setPos(x, y)
            self.boat.setRotation(heading - 90)
            self.boat.setBrush(QBrush(BOAT_STALE if stale else BOAT_LIVE))
            self.boat.show()
            if not stale:
                self.add_trail_point(x, y)

        self.fit_view()

    def add_trail_point(self, x: float, y: float):
        
        if self.trail:
            last = self.trail[-1]
            if math.hypot(x - last.x(), y - last.y()) < TRAIL_POINT_SPACING_M:
                return
            
        self.trail.append(QPointF(x, y))
        self.redraw_trail()

    def redraw_trail(self):
        path = QPainterPath()
        path.addPolygon(QPolygonF(list(self.trail)))
        self.trail_item.setPath(path)

    def clear_trail(self) -> None:
        self.trail.clear()
        self.redraw_trail()

    def set_waypoint(self, x: float, y: float):
        self.waypoint.setPos(x, y)
        self.waypoint.show()

    def set_home(self, x: float, y: float):
        self.home.setPos(x, y)
        self.home.show()

    def fit_view(self):
        
        xs: list[float] = []
        ys: list[float] = []
        
        if self.trail:
            bounds = self.trail_item.path().boundingRect()
            xs += [bounds.left(), bounds.right()]
            ys += [bounds.top(), bounds.bottom()]

        for item in (self.boat, self.waypoint, self.home):
            if item.isVisible():
                xs.append(item.x())
                ys.append(item.y())

        if not xs:
            return

        width = max(max(xs) - min(xs), MIN_VIEW_SPAN_M) * 1.2
        height = max(max(ys) - min(ys), MIN_VIEW_SPAN_M) * 1.2
        center_x = (max(xs) + min(xs)) / 2
        center_y = (max(ys) + min(ys)) / 2

        viewport = self.view.viewport()
        fit = (center_x, center_y, width, height, viewport.width(), viewport.height())
        if fit == self.last_fit:
            return
        
        self.last_fit = fit
        rect = QRectF(center_x - width / 2, center_y - height / 2, width, height)
        self.view.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)
