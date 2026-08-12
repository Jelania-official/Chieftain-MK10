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
    "left_target", "left_control_actual", "left_display_actual", "left_pwm", "left_stalled",
    "right_target", "right_control_actual", "right_display_actual", "right_pwm", "right_stalled",
    "pitch_target", "pitch_actual", "pitch_servo",
    "yaw_target", "yaw_actual", "yaw_relative", "yaw_voltage", "yaw_stalled",
    "chassis_pitch", "chassis_pitch_rate", "chassis_yaw_rate",
    "stabilization", "imu_healthy", "chassis_imu_healthy", "turret_imu_healthy", "yaw_sensor_healthy",
    "chassis_imu_initialized", "turret_imu_initialized", "yaw_sensor_initialized", "yaw_foc_initialized",
    "chassis_ready", "turret_ready", "battery_voltage", "battery_valid",
]

HANDSHAKE_REQUEST = b"HELLO,3\n"
HANDSHAKE_RESPONSE = "HELLO,ChieftainMK10,3"


class AutoConnectWorker(QtCore.QThread):
    device_found = QtCore.Signal(object, str)
    progress = QtCore.Signal(str)
    failed = QtCore.Signal(str)

    def __init__(self, requested_port: str | None = None, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.requested_port = requested_port

    def run(self) -> None:
        all_ports = list(serial.tools.list_ports.comports())
        if self.requested_port:
            ports = [port for port in all_ports if port.device == self.requested_port]
        else:
            ports = [
                port for port in all_ports
                if "BTHENUM" in (port.hwid or "").upper()
            ]
        if not ports:
            message = "没有找到所选串口" if self.requested_port else "没有发现已配对的蓝牙串口"
            self.failed.emit(message)
            return

        for info in ports:
            if self.isInterruptionRequested():
                return
            self.progress.emit(f"正在识别 {info.device}……")
            candidate: serial.Serial | None = None
            try:
                candidate = serial.Serial(
                    info.device,
                    115200,
                    timeout=0.2,
                    write_timeout=0.5,
                )
                candidate.reset_input_buffer()
                candidate.write(HANDSHAKE_REQUEST)
                deadline = time.monotonic() + 1.5
                while time.monotonic() < deadline:
                    if self.isInterruptionRequested():
                        candidate.close()
                        return
                    line = candidate.readline().decode("ascii", errors="ignore").strip()
                    if line == HANDSHAKE_RESPONSE:
                        candidate.timeout = 0
                        self.device_found.emit(candidate, info.device)
                        return
            except (serial.SerialException, OSError):
                pass

            if candidate is not None and candidate.is_open:
                candidate.close()

        if self.requested_port:
            self.failed.emit(f"{self.requested_port} 没有响应整车固件握手")
        else:
            self.failed.emit("未找到运行整车调试固件的 ESP32；请先在 Windows 中完成蓝牙配对")


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
        # 键盘没有模拟量行程；使用40%摇杆量更接近正常行进转向，
        # 避免A/D每次都等同于打满方向。固件仍保留完整的±1输入范围。
        left = -0.4 if QtCore.Qt.Key_A in self.keys else 0.0
        right = 0.4 if QtCore.Qt.Key_D in self.keys else 0.0
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
        self.setWindowTitle("Chieftain MK10 整车调试工具")
        self.resize(1280, 840)
        self.serial_port: serial.Serial | None = None
        self.auto_connect_worker: AutoConnectWorker | None = None
        self.input_state = InputState()
        self.direct_pwm_active = False
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

        QtCore.QTimer.singleShot(400, self._start_auto_connect)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        root.addLayout(top)
        self.port_combo = QtWidgets.QComboBox()
        refresh_btn = QtWidgets.QPushButton("刷新端口")
        self.auto_connect_btn = QtWidgets.QPushButton("自动连接")
        self.connect_btn = QtWidgets.QPushButton("连接")
        self.status_label = QtWidgets.QLabel("未连接")
        top.addWidget(QtWidgets.QLabel("串口"))
        top.addWidget(self.port_combo, 1)
        top.addWidget(refresh_btn)
        top.addWidget(self.auto_connect_btn)
        top.addWidget(self.connect_btn)
        top.addWidget(self.status_label)
        refresh_btn.clicked.connect(self._refresh_ports)
        self.auto_connect_btn.clicked.connect(self._start_auto_connect)
        self.connect_btn.clicked.connect(self._toggle_connection)

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)

        controls_panel = QtWidgets.QWidget()
        controls = QtWidgets.QVBoxLayout(controls_panel)
        controls_scroll = QtWidgets.QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setWidget(controls_panel)
        controls_scroll.setMinimumWidth(370)
        body.addWidget(controls_scroll)
        controls.addWidget(QtWidgets.QLabel("W/S：前进/倒车    A/D：转向    空格：切换稳定器"))
        controls.addWidget(QtWidgets.QLabel("在控制区拖动：控制炮塔方位/俯仰；松开后自动回中"))

        self.mouse_pad = MousePad()
        self.mouse_pad.changed.connect(self._mouse_changed)
        controls.addWidget(self.mouse_pad)

        self.input_label = QtWidgets.QLabel()
        controls.addWidget(self.input_label)

        zero_btn = QtWidgets.QPushButton("输入归零")
        stop_btn = QtWidgets.QPushButton("紧急停止")
        clear_stop_btn = QtWidgets.QPushButton("解除急停")
        controls.addWidget(zero_btn)
        controls.addWidget(stop_btn)
        controls.addWidget(clear_stop_btn)
        zero_btn.clicked.connect(self._zero_input)
        stop_btn.clicked.connect(self._emergency_stop)
        clear_stop_btn.clicked.connect(self._clear_stop)

        diagnostic_group = QtWidgets.QGroupBox("固定 PWM 标定 / 硬件诊断")
        diagnostic_layout = QtWidgets.QGridLayout(diagnostic_group)
        self.direct_pwm_check = QtWidgets.QCheckBox("确认周围安全，启用固定 PWM 模式")
        self.left_pwm_spin = QtWidgets.QSpinBox()
        self.right_pwm_spin = QtWidgets.QSpinBox()
        for spin in (self.left_pwm_spin, self.right_pwm_spin):
            spin.setRange(-180, 180)
            spin.setSingleStep(1)
            spin.setValue(0)
            spin.setEnabled(False)
        self.direct_pwm_output_btn = QtWidgets.QPushButton("开始输出 PWM")
        self.direct_pwm_output_btn.setEnabled(False)
        diagnostic_layout.addWidget(self.direct_pwm_check, 0, 0, 1, 2)
        diagnostic_layout.addWidget(QtWidgets.QLabel("左履带 PWM"), 1, 0)
        diagnostic_layout.addWidget(self.left_pwm_spin, 1, 1)
        diagnostic_layout.addWidget(QtWidgets.QLabel("右履带 PWM"), 2, 0)
        diagnostic_layout.addWidget(self.right_pwm_spin, 2, 1)
        diagnostic_layout.addWidget(self.direct_pwm_output_btn, 3, 0, 1, 2)
        diagnostic_layout.addWidget(QtWidgets.QLabel("范围 ±180；再次点击停止；失焦、断连或急停立即归零"), 4, 0, 1, 2)
        controls.addWidget(diagnostic_group)
        self.direct_pwm_check.toggled.connect(self._toggle_direct_pwm)
        self.direct_pwm_output_btn.clicked.connect(self._toggle_direct_pwm_output)

        pid_group = QtWidgets.QGroupBox("履带 PID 手动调参（左右共用）")
        pid_layout = QtWidgets.QGridLayout(pid_group)
        self.pid_kp_spin = QtWidgets.QDoubleSpinBox()
        self.pid_ki_spin = QtWidgets.QDoubleSpinBox()
        self.pid_kd_spin = QtWidgets.QDoubleSpinBox()
        pid_specs = (
            (self.pid_kp_spin, 0.0, 50.0, 0.5, 5.0),
            (self.pid_ki_spin, 0.0, 50.0, 0.1, 0.8),
            (self.pid_kd_spin, 0.0, 10.0, 0.01, 0.0),
        )
        for spin, minimum, maximum, step, value in pid_specs:
            spin.setRange(minimum, maximum)
            spin.setDecimals(4)
            spin.setSingleStep(step)
            spin.setValue(value)
        self.pid_apply_btn = QtWidgets.QPushButton("应用到内存")
        self.pid_read_btn = QtWidgets.QPushButton("读取当前参数")
        self.pid_default_btn = QtWidgets.QPushButton("恢复固件默认")
        self.pid_status_label = QtWidgets.QLabel("尚未从 ESP32 读取参数")
        pid_layout.addWidget(QtWidgets.QLabel("KP"), 0, 0)
        pid_layout.addWidget(self.pid_kp_spin, 0, 1)
        pid_layout.addWidget(QtWidgets.QLabel("KI"), 1, 0)
        pid_layout.addWidget(self.pid_ki_spin, 1, 1)
        pid_layout.addWidget(QtWidgets.QLabel("KD"), 2, 0)
        pid_layout.addWidget(self.pid_kd_spin, 2, 1)
        pid_layout.addWidget(self.pid_apply_btn, 3, 0, 1, 2)
        pid_layout.addWidget(self.pid_read_btn, 4, 0)
        pid_layout.addWidget(self.pid_default_btn, 4, 1)
        pid_layout.addWidget(self.pid_status_label, 5, 0, 1, 2)
        pid_layout.addWidget(QtWidgets.QLabel("仅本次运行有效；重启 ESP32 后恢复固件默认值"), 6, 0, 1, 2)
        controls.addWidget(pid_group)
        self.pid_apply_btn.clicked.connect(self._apply_pid_gains)
        self.pid_read_btn.clicked.connect(self._request_pid_gains)
        self.pid_default_btn.clicked.connect(self._restore_pid_defaults)

        self.telemetry_label = QtWidgets.QLabel("尚未收到遥测数据")
        self.telemetry_label.setMinimumWidth(330)
        speed_display_row = QtWidgets.QHBoxLayout()
        self.actual_speed_combo = QtWidgets.QComboBox()
        self.actual_speed_combo.addItem("控制反馈（PI 实际使用）", "control")
        self.actual_speed_combo.addItem("平滑显示", "display")
        speed_display_row.addWidget(QtWidgets.QLabel("履带实际速度显示"))
        speed_display_row.addWidget(self.actual_speed_combo, 1)
        controls.addLayout(speed_display_row)
        controls.addWidget(self.telemetry_label)
        controls.addStretch(1)

        plots = QtWidgets.QGridLayout()
        body.addLayout(plots, 1)
        pg.setConfigOptions(antialias=True)
        self.curves = {}
        self._add_plot(plots, 0, 0, "左履带速度（km/h）", [
            ("left_target", "目标速度", "#f5a623"),
            ("left_actual_selected", "实际速度（可选）", "#2d7ff9"),
        ])
        self._add_plot(plots, 0, 1, "右履带速度（km/h）", [
            ("right_target", "目标速度", "#f5a623"),
            ("right_actual_selected", "实际速度（可选）", "#2d7ff9"),
        ])
        self._add_plot(plots, 1, 0, "炮塔方位角（°）", [
            ("yaw_target", "目标角度", "#f5a623"),
            ("yaw_actual", "实际角度", "#2d7ff9"),
            ("yaw_relative", "车体相对角", "#42b883"),
        ])
        self._add_plot(plots, 1, 1, "炮管俯仰角（°）", [
            ("pitch_target", "目标角度", "#f5a623"),
            ("pitch_actual", "实际角度", "#2d7ff9"),
            ("pitch_servo", "舵机指令", "#42b883"),
        ])
        self._add_plot(plots, 2, 0, "履带 PWM / 炮塔方位电压", [
            ("left_pwm", "左履带 PWM", "#2d7ff9"),
            ("right_pwm", "右履带 PWM", "#f5a623"),
            ("yaw_voltage", "方位电压（V）", "#d0021b"),
        ])
        self._add_plot(plots, 2, 1, "车体 IMU", [
            ("chassis_pitch", "俯仰角", "#42b883"),
            ("chassis_pitch_rate", "俯仰角速度", "#2d7ff9"),
            ("chassis_yaw_rate", "方位角速度", "#f5a623"),
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

    def _start_auto_connect(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            return
        if self.auto_connect_worker and self.auto_connect_worker.isRunning():
            return

        self.auto_connect_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.status_label.setText("正在搜索已配对的 ESP32……")
        self.auto_connect_worker = AutoConnectWorker(parent=self)
        self.auto_connect_worker.device_found.connect(self._auto_connect_succeeded)
        self.auto_connect_worker.progress.connect(self.status_label.setText)
        self.auto_connect_worker.failed.connect(self._auto_connect_failed)
        self.auto_connect_worker.finished.connect(self._auto_connect_finished)
        self.auto_connect_worker.start()

    def _auto_connect_finished(self) -> None:
        self.auto_connect_btn.setEnabled(True)
        self.connect_btn.setEnabled(True)

    def _auto_connect_succeeded(self, port_object: object, port_name: str) -> None:
        if not isinstance(port_object, serial.Serial):
            self._auto_connect_failed("自动连接返回了无效串口")
            return
        if self.serial_port and self.serial_port.is_open:
            port_object.close()
            return
        self._adopt_connection(port_object, port_name)

    def _auto_connect_failed(self, message: str) -> None:
        self.status_label.setText(message)

    def _adopt_connection(self, port: serial.Serial, port_name: str) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = port
        self._set_direct_pwm_active(False)
        self.input_state.zero_motion()
        self.input_state.stop = True
        self.connect_btn.setText("断开连接")
        self.status_label.setText(f"已连接：{port_name}（底盘保持急停）")
        self.pid_status_label.setText("正在读取 ESP32 当前参数……")
        self._request_pid_gains()

    def _disconnect(self) -> None:
        self._emergency_stop()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"STOP\n")
            except serial.SerialException:
                pass
            self.serial_port.close()
        self.serial_port = None
        self.connect_btn.setText("连接")
        self.status_label.setText("未连接")
        self.pid_status_label.setText("未连接；参数未应用")

    def _toggle_connection(self) -> None:
        if self.auto_connect_worker and self.auto_connect_worker.isRunning():
            return
        if self.serial_port and self.serial_port.is_open:
            self._disconnect()
            return

        port = self.port_combo.currentData()
        if not port:
            return
        self.auto_connect_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.status_label.setText(f"正在连接并识别 {port}……")
        self.auto_connect_worker = AutoConnectWorker(requested_port=port, parent=self)
        self.auto_connect_worker.device_found.connect(self._auto_connect_succeeded)
        self.auto_connect_worker.progress.connect(self.status_label.setText)
        self.auto_connect_worker.failed.connect(self._auto_connect_failed)
        self.auto_connect_worker.finished.connect(self._auto_connect_finished)
        self.auto_connect_worker.start()

    def _poll_serial(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            while self.serial_port.in_waiting:
                raw = self.serial_port.readline().decode("ascii", errors="ignore").strip()
                if raw:
                    self._handle_line(raw)
        except serial.SerialException as exc:
            self.status_label.setText(f"串口错误：{exc}")
            self.serial_port.close()
            self.serial_port = None
            self._set_direct_pwm_active(False)
            self.connect_btn.setText("连接")
            self.input_state.zero_motion()
            self.input_state.stop = True

    def _handle_line(self, line: str) -> None:
        if line.startswith("PID,VALUE,"):
            parts = line.split(",")
            if len(parts) != 5:
                return
            try:
                kp, ki, kd = (float(value) for value in parts[2:5])
            except ValueError:
                return
            for spin, value in (
                (self.pid_kp_spin, kp),
                (self.pid_ki_spin, ki),
                (self.pid_kd_spin, kd),
            ):
                blocker = QtCore.QSignalBlocker(spin)
                spin.setValue(value)
                del blocker
            self.pid_status_label.setText(
                f"ESP32 当前参数：KP={kp:.4f}  KI={ki:.4f}  KD={kd:.4f}"
            )
            return
        if line.startswith("PID,ERROR,"):
            reason = line.partition("PID,ERROR,")[2] or "未知错误"
            self.pid_status_label.setText(f"参数被 ESP32 拒绝：{reason}")
            return
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
        actual_mode = self.actual_speed_combo.currentData()
        actual_label = "控制反馈" if actual_mode == "control" else "平滑显示"
        left_actual = values[f"left_{actual_mode}_actual"]
        right_actual = values[f"right_{actual_mode}_actual"]
        self.telemetry_label.setText(
            "遥测状态\n"
            f"底盘就绪={int(values['chassis_ready'])}  炮塔整体就绪={int(values['turret_ready'])}\n"
            f"初始化：底盘IMU={int(values['chassis_imu_initialized'])}  炮塔IMU={int(values['turret_imu_initialized'])}  AS5600={int(values['yaw_sensor_initialized'])}  FOC={int(values['yaw_foc_initialized'])}\n"
            f"运行健康：底盘IMU={int(values['chassis_imu_healthy'])}  炮塔IMU={int(values['turret_imu_healthy'])}  AS5600={int(values['yaw_sensor_healthy'])}  双IMU={int(values['imu_healthy'])}\n"
            f"稳定器={int(values['stabilization'])}  方位堵转={int(values['yaw_stalled'])}\n"
            f"左履带 目标/{actual_label}/PWM={values['left_target']:.2f}/{left_actual:.2f}/{values['left_pwm']:.1f}\n"
            f"右履带 目标/{actual_label}/PWM={values['right_target']:.2f}/{right_actual:.2f}/{values['right_pwm']:.1f}\n"
            f"左右履带堵转={int(values['left_stalled'])}/{int(values['right_stalled'])}\n"
            f"电池={values['battery_voltage']:.2f}V  有效={int(values['battery_valid'])}\n"
            f"最近接收={time.monotonic() - self.last_rx_time:.2f}s"
        )

    def _send_input(self) -> None:
        diagnostic_enabled = self.direct_pwm_check.isChecked()
        if diagnostic_enabled:
            self.input_state.zero_motion()
        else:
            self.input_state.update_from_keys()
        direct_left = self.left_pwm_spin.value() if self.direct_pwm_active else 0
        direct_right = self.right_pwm_spin.value() if self.direct_pwm_active else 0
        self.input_label.setText(
            f"倒车 LT={self.input_state.trigger_l:.1f}  前进 RT={self.input_state.trigger_r:.1f}\n"
            f"转向 LX={self.input_state.joy_lx:.2f}\n"
            f"炮塔 RX={self.input_state.joy_rx:.2f}  RY={self.input_state.joy_ry:.2f}\n"
            f"稳定器 A={int(self.input_state.a_pressed)}  急停={int(self.input_state.stop)}\n"
            f"固定PWM模式={int(diagnostic_enabled)}  输出={direct_left}/{direct_right}"
        )
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            if self.input_state.stop:
                self.serial_port.write(b"STOP\n")
            elif diagnostic_enabled:
                self.serial_port.write(f"PWM,{direct_left},{direct_right}\n".encode("ascii"))
            else:
                self.serial_port.write(self.input_state.command_line().encode("ascii"))
        except serial.SerialException:
            self.serial_port.close()
            self.serial_port = None
            self._set_direct_pwm_active(False)
            self.connect_btn.setText("连接")
            self.input_state.zero_motion()
            self.input_state.stop = True

    def _update_plots(self) -> None:
        x = list(self.data["t"])
        if not x:
            return
        actual_mode = self.actual_speed_combo.currentData()
        for field, curve in self.curves.items():
            source_field = field
            if field == "left_actual_selected":
                source_field = f"left_{actual_mode}_actual"
            elif field == "right_actual_selected":
                source_field = f"right_{actual_mode}_actual"
            curve.setData(x, list(self.data[source_field]))

    def _mouse_changed(self, x: float, y: float) -> None:
        self.input_state.joy_rx = x
        self.input_state.joy_ry = -y

    def _zero_input(self) -> None:
        self.input_state.zero_motion()
        self._set_direct_pwm_active(False, send_zero=True)
        self.left_pwm_spin.setValue(0)
        self.right_pwm_spin.setValue(0)
        self.mouse_pad._pos = QtCore.QPointF(0.0, 0.0)
        self.mouse_pad.update()

    def _emergency_stop(self) -> None:
        self.input_state.zero_motion()
        self._set_direct_pwm_active(False, send_zero=True)
        self.input_state.stop = True

    def _clear_stop(self) -> None:
        self.input_state.zero_motion()
        self._set_direct_pwm_active(False, send_zero=True)
        self.input_state.stop = False
        if self.serial_port and self.serial_port.is_open:
            try:
                # 先用正常的零输入明确解除固件急停；PWM 指令本身没有解锁权限。
                self.serial_port.write(self.input_state.command_line().encode("ascii"))
            except serial.SerialException:
                pass

    def _toggle_direct_pwm(self, enabled: bool) -> None:
        self._set_direct_pwm_active(False, send_zero=True)
        if enabled:
            answer = QtWidgets.QMessageBox.warning(
                self,
                "固定 PWM 诊断",
                "该模式会绕过速度 PI 和堵转保护。\n\n"
                "首次测试应架空履带；落地标定时车辆可能突然移动。\n"
                "请清空周围区域，并确保可以立即按下急停。",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                blocker = QtCore.QSignalBlocker(self.direct_pwm_check)
                self.direct_pwm_check.setChecked(False)
                del blocker
                enabled = False
        self.left_pwm_spin.setEnabled(enabled)
        self.right_pwm_spin.setEnabled(enabled)
        self.direct_pwm_output_btn.setEnabled(enabled)
        self.input_state.zero_motion()
        self._send_direct_pwm_zero()

    def _toggle_direct_pwm_output(self) -> None:
        if self.direct_pwm_active:
            self._set_direct_pwm_active(False, send_zero=True)
            return
        if self.direct_pwm_check.isChecked() and not self.input_state.stop:
            self._set_direct_pwm_active(True)

    def _set_direct_pwm_active(self, active: bool, send_zero: bool = False) -> None:
        self.direct_pwm_active = active
        if hasattr(self, "direct_pwm_output_btn"):
            self.direct_pwm_output_btn.setText("停止输出 PWM" if active else "开始输出 PWM")
            self.direct_pwm_output_btn.setStyleSheet(
                "QPushButton { background-color: #f5a623; font-weight: bold; }" if active else ""
            )
        if not active and send_zero:
            self._send_direct_pwm_zero()

    def _send_direct_pwm_zero(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            self.serial_port.write(b"PWM,0,0\n")
        except serial.SerialException:
            pass

    def _send_pid_command(self, command: str) -> bool:
        if not self.serial_port or not self.serial_port.is_open:
            self.pid_status_label.setText("未连接，无法发送 PID 参数")
            return False
        try:
            self.serial_port.write((command + "\n").encode("ascii"))
            return True
        except serial.SerialException as exc:
            self.pid_status_label.setText(f"PID 参数发送失败：{exc}")
            return False

    def _apply_pid_gains(self) -> None:
        kp = self.pid_kp_spin.value()
        ki = self.pid_ki_spin.value()
        kd = self.pid_kd_spin.value()
        if self._send_pid_command(f"PID,SET,{kp:.4f},{ki:.4f},{kd:.4f}"):
            self.pid_status_label.setText("参数已发送，等待 ESP32 确认……")

    def _request_pid_gains(self) -> None:
        if self._send_pid_command("PID,GET"):
            self.pid_status_label.setText("正在读取 ESP32 当前参数……")

    def _restore_pid_defaults(self) -> None:
        if self._send_pid_command("PID,DEFAULT"):
            self.pid_status_label.setText("正在恢复固件默认参数……")

    def event(self, event: QtCore.QEvent) -> bool:
        if event.type() == QtCore.QEvent.WindowDeactivate and hasattr(self, "input_state"):
            self.input_state.zero_motion()
            self._set_direct_pwm_active(False, send_zero=True)
        return super().event(event)

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
        if self.auto_connect_worker and self.auto_connect_worker.isRunning():
            self.auto_connect_worker.requestInterruption()
            self.auto_connect_worker.wait(6000)
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"STOP\n")
                self.serial_port.close()
            except serial.SerialException:
                pass
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QtGui.QFont("Microsoft YaHei UI", 9))
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
