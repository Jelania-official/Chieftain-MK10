from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass, field

import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


TELEMETRY_FIELDS = [
    "time_ms",
    "left_target", "left_actual", "left_pwm", "left_stalled",
    "right_target", "right_actual", "right_pwm", "right_stalled",
    "pitch_target", "pitch_actual", "pitch_servo",
    "yaw_target", "yaw_actual", "yaw_relative", "yaw_voltage",
    "chassis_pitch", "chassis_pitch_rate", "chassis_yaw_rate",
    "stabilization", "imu_healthy", "yaw_sensor_healthy",
    "chassis_ready", "turret_ready", "battery_voltage", "battery_valid",
]


@dataclass
class InputState:
    trigger_l: float = 0.0
    trigger_r: float = 0.0
    joy_lx: float = 0.0
    joy_rx: float = 0.0
    joy_ry: float = 0.0
    a_pressed: bool = False
    stop: bool = False

    keys: set[int] = field(default_factory=set)

    def update_from_keys(self) -> None:
        self.trigger_r = 1.0 if QtCore.Qt.Key_W in self.keys else 0.0
        self.trigger_l = 1.0 if QtCore.Qt.Key_S in self.keys else 0.0
        left = -1.0 if QtCore.Qt.Key_A in self.keys else 0.0
        right = 1.0 if QtCore.Qt.Key_D in self.keys else 0.0
        self.joy_lx = left + right
        self.a_pressed = QtCore.Qt.Key_Space in self.keys

    def zero_motion(self) -> None:
        self.keys.clear()
        self.trigger_l = 0.0
        self.trigger_r = 0.0
        self.joy_lx = 0.0
        self.joy_rx = 0.0
        self.joy_ry = 0.0
        self.a_pressed = False

    def command_line(self) -> str:
        return (
            f"IN,{self.trigger_l:.3f},{self.trigger_r:.3f},{self.joy_lx:.3f},"
            f"{self.joy_rx:.3f},{self.joy_ry:.3f},{1 if self.a_pressed else 0},"
            f"{1 if self.stop else 0}\n"
        )


class MousePad(QtWidgets.QFrame):
    changed = QtCore.Signal(float, float)

    def __init__(self) -> None:
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setMinimumSize(220, 220)
        self.setMouseTracking(True)
        self._active = False
        self._pos = QtCore.QPointF(0.0, 0.0)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self._active = True
            self._update_from_pos(event.position())

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._active:
            self._update_from_pos(event.position())

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self._active = False
            self._pos = QtCore.QPointF(0.0, 0.0)
            self.changed.emit(0.0, 0.0)
            self.update()

    def leaveEvent(self, event: QtCore.QEvent) -> None:
        if self._active:
            self._active = False
            self._pos = QtCore.QPointF(0.0, 0.0)
            self.changed.emit(0.0, 0.0)
            self.update()

    def _update_from_pos(self, pos: QtCore.QPointF) -> None:
        w = max(1.0, float(self.width()))
        h = max(1.0, float(self.height()))
        x = max(-1.0, min(1.0, (pos.x() - w / 2.0) / (w / 2.0)))
        y = max(-1.0, min(1.0, (pos.y() - h / 2.0) / (h / 2.0)))
        self._pos = QtCore.QPointF(x, y)
        self.changed.emit(x, y)
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect().adjusted(10, 10, -10, -10)
        center = rect.center()
        painter.setPen(QtGui.QPen(QtGui.QColor("#777"), 1))
        painter.drawLine(rect.left(), center.y(), rect.right(), center.y())
        painter.drawLine(center.x(), rect.top(), center.x(), rect.bottom())
        dot_x = center.x() + self._pos.x() * rect.width() / 2.0
        dot_y = center.y() + self._pos.y() * rect.height() / 2.0
        painter.setBrush(QtGui.QColor("#2d7ff9"))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(QtCore.QPointF(dot_x, dot_y), 7, 7)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Chieftain MK10 Debug Tool")
        self.resize(1280, 840)
        self.serial_port: serial.Serial | None = None
        self.input_state = InputState()
        self.last_rx_time = 0.0
        self.data: dict[str, deque[float]] = {
            name: deque(maxlen=800) for name in TELEMETRY_FIELDS
        }
        self.data["t"] = deque(maxlen=800)

        self._build_ui()
        self._refresh_ports()

        self.io_timer = QtCore.QTimer(self)
        self.io_timer.timeout.connect(self._poll_serial)
        self.io_timer.start(20)

        self.input_timer = QtCore.QTimer(self)
        self.input_timer.timeout.connect(self._send_input)
        self.input_timer.start(20)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(50)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        root.addLayout(top)
        self.port_combo = QtWidgets.QComboBox()
        refresh_btn = QtWidgets.QPushButton("Refresh")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.status_label = QtWidgets.QLabel("Disconnected")
        top.addWidget(QtWidgets.QLabel("Port"))
        top.addWidget(self.port_combo, 1)
        top.addWidget(refresh_btn)
        top.addWidget(self.connect_btn)
        top.addWidget(self.status_label)
        refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._toggle_connection)

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)

        controls = QtWidgets.QVBoxLayout()
        body.addLayout(controls)
        controls.addWidget(QtWidgets.QLabel("W/S: throttle    A/D: steer    Space: stabilizer toggle"))
        controls.addWidget(QtWidgets.QLabel("Drag inside pad: turret yaw/pitch, release to center"))

        self.mouse_pad = MousePad()
        self.mouse_pad.changed.connect(self._mouse_changed)
        controls.addWidget(self.mouse_pad)

        self.input_label = QtWidgets.QLabel()
        controls.addWidget(self.input_label)

        zero_btn = QtWidgets.QPushButton("Zero Input")
        stop_btn = QtWidgets.QPushButton("Emergency Stop")
        clear_stop_btn = QtWidgets.QPushButton("Clear Stop")
        controls.addWidget(zero_btn)
        controls.addWidget(stop_btn)
        controls.addWidget(clear_stop_btn)
        zero_btn.clicked.connect(self._zero_input)
        stop_btn.clicked.connect(self._emergency_stop)
        clear_stop_btn.clicked.connect(self._clear_stop)

        self.telemetry_label = QtWidgets.QLabel("No telemetry")
        self.telemetry_label.setMinimumWidth(280)
        controls.addWidget(self.telemetry_label)
        controls.addStretch(1)

        plots = QtWidgets.QGridLayout()
        body.addLayout(plots, 1)
        pg.setConfigOptions(antialias=True)
        self.curves = {}
        self._add_plot(plots, 0, 0, "Left track speed", [
            ("left_target", "target", "#f5a623"),
            ("left_actual", "actual", "#2d7ff9"),
        ])
        self._add_plot(plots, 0, 1, "Right track speed", [
            ("right_target", "target", "#f5a623"),
            ("right_actual", "actual", "#2d7ff9"),
        ])
        self._add_plot(plots, 1, 0, "Turret yaw", [
            ("yaw_target", "target", "#f5a623"),
            ("yaw_actual", "actual", "#2d7ff9"),
            ("yaw_relative", "relative", "#42b883"),
        ])
        self._add_plot(plots, 1, 1, "Gun pitch", [
            ("pitch_target", "target", "#f5a623"),
            ("pitch_actual", "actual", "#2d7ff9"),
            ("pitch_servo", "servo", "#42b883"),
        ])
        self._add_plot(plots, 2, 0, "PWM / yaw voltage", [
            ("left_pwm", "left pwm", "#2d7ff9"),
            ("right_pwm", "right pwm", "#f5a623"),
            ("yaw_voltage", "yaw V", "#d0021b"),
        ])
        self._add_plot(plots, 2, 1, "Chassis IMU", [
            ("chassis_pitch", "pitch", "#42b883"),
            ("chassis_pitch_rate", "pitch rate", "#2d7ff9"),
            ("chassis_yaw_rate", "yaw rate", "#f5a623"),
        ])

    def _add_plot(self, layout: QtWidgets.QGridLayout, row: int, col: int, title: str, items: list[tuple[str, str, str]]) -> None:
        plot = pg.PlotWidget(title=title)
        plot.addLegend()
        plot.showGrid(x=True, y=True, alpha=0.25)
        layout.addWidget(plot, row, col)
        for field, label, color in items:
            self.curves[field] = plot.plot([], [], pen=pg.mkPen(color, width=2), name=label)

    def _refresh_ports(self) -> None:
        self.port_combo.clear()
        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{port.device}  {port.description}", port.device)

    def _toggle_connection(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            return

        port = self.port_combo.currentData()
        if not port:
            return
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0)
            self.connect_btn.setText("Disconnect")
            self.status_label.setText(f"Connected: {port}")
        except serial.SerialException as exc:
            QtWidgets.QMessageBox.warning(self, "Connection failed", str(exc))

    def _poll_serial(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            while self.serial_port.in_waiting:
                raw = self.serial_port.readline().decode("ascii", errors="ignore").strip()
                if raw:
                    self._handle_line(raw)
        except serial.SerialException as exc:
            self.status_label.setText(f"Serial error: {exc}")
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.setText("Connect")

    def _handle_line(self, line: str) -> None:
        if not line.startswith("TEL,"):
            return
        parts = line.split(",")[1:]
        if len(parts) != len(TELEMETRY_FIELDS):
            return
        values = {}
        for name, text in zip(TELEMETRY_FIELDS, parts):
            try:
                values[name] = float(text)
            except ValueError:
                return
        t0 = values["time_ms"] * 0.001
        if not self.data["t"]:
            self._first_time = t0
        self.data["t"].append(t0 - getattr(self, "_first_time", t0))
        for name, value in values.items():
            self.data[name].append(value)
        self.last_rx_time = time.monotonic()
        self._update_status_text(values)

    def _update_status_text(self, values: dict[str, float]) -> None:
        self.telemetry_label.setText(
            "Telemetry\n"
            f"chassisReady={int(values['chassis_ready'])}  turretReady={int(values['turret_ready'])}\n"
            f"stab={int(values['stabilization'])}  imu={int(values['imu_healthy'])}  yawSensor={int(values['yaw_sensor_healthy'])}\n"
            f"stall L/R={int(values['left_stalled'])}/{int(values['right_stalled'])}\n"
            f"battery={values['battery_voltage']:.2f}V valid={int(values['battery_valid'])}\n"
            f"last rx={time.monotonic() - self.last_rx_time:.2f}s"
        )

    def _send_input(self) -> None:
        self.input_state.update_from_keys()
        self.input_label.setText(
            f"LT {self.input_state.trigger_l:.1f}  RT {self.input_state.trigger_r:.1f}\n"
            f"LX {self.input_state.joy_lx:.2f}\n"
            f"RX {self.input_state.joy_rx:.2f}  RY {self.input_state.joy_ry:.2f}\n"
            f"A {int(self.input_state.a_pressed)}  STOP {int(self.input_state.stop)}"
        )
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            if self.input_state.stop:
                self.serial_port.write(b"STOP\n")
            else:
                self.serial_port.write(self.input_state.command_line().encode("ascii"))
        except serial.SerialException:
            self.serial_port.close()
            self.serial_port = None
            self.connect_btn.setText("Connect")

    def _update_plots(self) -> None:
        x = list(self.data["t"])
        if not x:
            return
        for field, curve in self.curves.items():
            curve.setData(x, list(self.data[field]))

    def _mouse_changed(self, x: float, y: float) -> None:
        self.input_state.joy_rx = x
        self.input_state.joy_ry = -y

    def _zero_input(self) -> None:
        self.input_state.zero_motion()
        self.mouse_pad._pos = QtCore.QPointF(0.0, 0.0)
        self.mouse_pad.update()

    def _emergency_stop(self) -> None:
        self.input_state.zero_motion()
        self.input_state.stop = True

    def _clear_stop(self) -> None:
        self.input_state.stop = False

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.isAutoRepeat():
            return
        if event.key() == QtCore.Qt.Key_Escape:
            self._emergency_stop()
            return
        if event.key() in {QtCore.Qt.Key_W, QtCore.Qt.Key_A, QtCore.Qt.Key_S, QtCore.Qt.Key_D, QtCore.Qt.Key_Space}:
            self.input_state.keys.add(event.key())

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.isAutoRepeat():
            return
        self.input_state.keys.discard(event.key())

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"STOP\n")
                self.serial_port.close()
            except serial.SerialException:
                pass
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
