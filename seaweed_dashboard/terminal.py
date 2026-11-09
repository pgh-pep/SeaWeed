import fcntl
import math
import os
import pty
import struct
import subprocess
import termios

import pyte
from PySide6.QtCore import QEvent, QObject, QSocketNotifier, Qt, QTimer
from PySide6.QtGui import QFontMetricsF, QGuiApplication
from PySide6.QtWidgets import QPlainTextEdit

K = Qt.Key
KEYS = {
    K.Key_Return.value: b"\r",
    K.Key_Enter.value: b"\r",
    K.Key_Backspace.value: b"\x7f",
    K.Key_Tab.value: b"\t",
    K.Key_Escape.value: b"\x1b",
    K.Key_Up.value: b"\x1b[A",
    K.Key_Down.value: b"\x1b[B",
    K.Key_Right.value: b"\x1b[C",
    K.Key_Left.value: b"\x1b[D",
    K.Key_Home.value: b"\x1b[H",
    K.Key_End.value: b"\x1b[F",
    K.Key_Delete.value: b"\x1b[3~",
    K.Key_PageUp.value: b"\x1b[5~",
    K.Key_PageDown.value: b"\x1b[6~",
}


class BashTerminal(QObject):

    def __init__(self, view: QPlainTextEdit):
        super().__init__(view)
        self.view = view
        view.setUndoRedoEnabled(False)
        view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        view.setStyleSheet("font-family: monospace; font-size: 10pt;")

        self.rows, self.cols = self.grid_size()
        self.screen = pyte.HistoryScreen(self.cols, self.rows, history=1000, ratio=0.5)
        self.stream = pyte.ByteStream(self.screen)

        self.fd, child_side = pty.openpty()
        self.shell = subprocess.Popen(["setsid", "--ctty", "bash"], stdin=child_side, stdout=child_side, stderr=child_side, env=dict(os.environ, TERM="linux"))
        os.close(child_side)
        self.set_pty_size()

        self.exited = False
        self.dirty = False
        self.notifier = QSocketNotifier(self.fd, QSocketNotifier.Type.Read, self)
        self.notifier.activated.connect(self.on_read)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.render)
        self.timer.start(33)

        view.installEventFilter(self)
        view.viewport().installEventFilter(self)

    def grid_size(self) -> tuple[int, int]:
        self.view.ensurePolished()
        metrics = QFontMetricsF(self.view.font())
        margin = 2 * self.view.document().documentMargin()
        viewport = self.view.viewport()
        rows = int((viewport.height() - margin) // math.ceil(metrics.lineSpacing()))
        cols = int((viewport.width() - margin) // metrics.horizontalAdvance("M"))
        return max(2, rows), max(10, cols)

    def set_pty_size(self):
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, struct.pack("HHHH", self.rows, self.cols, 0, 0))

    def on_read(self) -> None:
        try:
            data = os.read(self.fd, 65536)
        except OSError: 
            data = b""

        if not data:
            self.notifier.setEnabled(False)
            self.shell.wait()
            self.exited = True
            self.view.appendPlainText("[bash exited: restart the dashboard for a new terminal]")
            return

        self.stream.feed(data)
        self.dirty = True

    def render(self):
        if not self.dirty or self.exited:
            return
        self.dirty = False

        screen = self.screen
        history = ["".join(line[x].data for x in range(screen.columns)).rstrip() for line in screen.history.top]
        display = [line.rstrip() for line in screen.display]
        display[screen.cursor.y] = display[screen.cursor.y].ljust(screen.cursor.x)
        self.view.setPlainText("\n".join(history + display))

        cursor = self.view.textCursor()
        block = self.view.document().findBlockByNumber(len(history) + screen.cursor.y)
        cursor.setPosition(block.position() + screen.cursor.x)
        self.view.setTextCursor(cursor)
        self.view.ensureCursorVisible()

    def eventFilter(self, obj: QObject, event: QEvent):
        if self.exited:
            return False

        if event.type() == QEvent.Type.Resize:
            rows, cols = self.grid_size()
            if (rows, cols) != (self.rows, self.cols):
                self.rows, self.cols = rows, cols
                self.screen.resize(rows, cols)
                self.set_pty_size()
                self.dirty = True
            return False

        if event.type() == QEvent.Type.KeyPress:
            key = event.key()
            modifiers = event.modifiers()
            ctrl = bool(modifiers & Qt.KeyboardModifier.ControlModifier)
            shift = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)

            if ctrl and shift and key == K.Key_C.value:
                self.view.copy()
                return True
            if ctrl and shift and key == K.Key_V.value:
                os.write(self.fd, QGuiApplication.clipboard().text().encode())
                return True

            if ctrl and K.Key_A.value <= key <= K.Key_Z.value:
                data = bytes([key - K.Key_A.value + 1])  # Ctrl+C is byte 3, and so on.
            else:
                data = KEYS.get(key) or event.text().encode()

            if data:
                os.write(self.fd, data)
            return True

        return False
