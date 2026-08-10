from __future__ import annotations

import csv
import math
import statistics
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


BAUD_RATE = 115200
HANDSHAKE_REQUEST = b"HELLO,1\n"
HANDSHAKE_RESPONSE = "HELLO,Chieftain-AS5600-FOC,1"
MAX_POINTS = 2400

TELEMETRY_FIELDS = [
    "time_ms",
    "state",
    "mode",
    "foc_ready",
    "sensor_ok",
    "battery_v",
    "raw_count",
    "raw_deg",
    "position_deg",
    "target_position_deg",
    "target_velocity_dps",
    "actual_velocity_dps",
    "voltage_v",
    "voltage_limit_v",
    "position_error_deg",
    "stall_latched",
    "i2c_errors",
    "foc_hz",
]

STATE_NAMES = {
    0: "仅传感器",
    1: "FOC对齐中",
    2: "已就绪",
    3: "正在输出",
    4: "故障锁存",
}

MODE_NAMES = {
    0: "空闲",
    1: "直接电压",
    2: "速度闭环",
    3: "位置串级闭环",
}


def wrap180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


class AutoConnectWorker(QtCore.QThread):
    device_found = QtCore.Signal(object, str)
    progress = QtCore.Signal(str)
    failed = QtCore.Signal(str)

    def __init__(self, requested_port: str | None = None, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.requested_port = requested_port

    def run(self) -> None:
        ports = list(serial.tools.list_ports.comports())
        if self.requested_port:
            ports = [port for port in ports if port.device == self.requested_port]
        if not ports:
            self.failed.emit("没有找到可用串口")
            return

        for info in ports:
            if self.isInterruptionRequested():
                return
            self.progress.emit(f"正在识别 {info.device}……")
            candidate: serial.Serial | None = None
            try:
                candidate = serial.Serial(
                    info.device,
                    BAUD_RATE,
                    timeout=0.20,
                    write_timeout=0.5,
                )
                # ESP32打开USB串口时通常会复位，等待独立固件完成安全初始化。
                deadline = time.monotonic() + 3.5
                next_hello = 0.0
                while time.monotonic() < deadline:
                    if self.isInterruptionRequested():
                        candidate.close()
                        return
                    now = time.monotonic()
                    if now >= next_hello:
                        candidate.write(HANDSHAKE_REQUEST)
                        next_hello = now + 0.4
                    line = candidate.readline().decode("ascii", errors="ignore").strip()
                    if line == HANDSHAKE_RESPONSE:
                        candidate.timeout = 0
                        candidate.reset_input_buffer()
                        self.device_found.emit(candidate, info.device)
                        return
            except (serial.SerialException, OSError):
                pass

            if candidate is not None and candidate.is_open:
                candidate.close()

        if self.requested_port:
            self.failed.emit(f"{self.requested_port} 没有响应AS5600/FOC调试固件握手")
        else:
            self.failed.emit("未找到运行AS5600/FOC调试固件的ESP32")


class TurretDial(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.actual_deg = 0.0
        self.target_deg = 0.0
        self.front_calibrated = False
        self.setMinimumSize(300, 300)

    def set_angles(self, actual_deg: float, target_deg: float, calibrated: bool) -> None:
        self.actual_deg = wrap180(actual_deg)
        self.target_deg = wrap180(target_deg)
        self.front_calibrated = calibrated
        self.update()

    @staticmethod
    def _point(center: QtCore.QPointF, radius: float, angle_deg: float) -> QtCore.QPointF:
        radians = math.radians(angle_deg - 90.0)
        return QtCore.QPointF(
            center.x() + radius * math.cos(radians),
            center.y() + radius * math.sin(radians),
        )

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        del event
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#111722"))

        side = min(self.width(), self.height()) - 38
        circle = QtCore.QRectF(
            (self.width() - side) / 2,
            (self.height() - side) / 2,
            side,
            side,
        )
        center = circle.center()
        radius = side / 2

        painter.setPen(QtGui.QPen(QtGui.QColor("#526174"), 2))
        painter.setBrush(QtGui.QColor("#182231"))
        painter.drawEllipse(circle)

        # 后甲板区域：外层±15°，内层±10°。
        avoid_pen = QtGui.QPen(QtGui.QColor("#d68a2f"), 12)
        avoid_pen.setCapStyle(QtCore.Qt.FlatCap)
        painter.setPen(avoid_pen)
        painter.drawArc(circle.adjusted(8, 8, -8, -8), int((270 - 15) * 16), int(30 * 16))
        full_pen = QtGui.QPen(QtGui.QColor("#d94b4b"), 12)
        full_pen.setCapStyle(QtCore.Qt.FlatCap)
        painter.setPen(full_pen)
        painter.drawArc(circle.adjusted(23, 23, -23, -23), int((270 - 10) * 16), int(20 * 16))

        painter.setFont(QtGui.QFont("Microsoft YaHei", 9))
        painter.setPen(QtGui.QColor("#dce6f2"))
        for angle, label in ((0, "前"), (90, "+90"), (-90, "-90"), (180, "后")):
            point = self._point(center, radius - 18, angle)
            painter.drawText(QtCore.QRectF(point.x() - 25, point.y() - 10, 50, 20),
                             QtCore.Qt.AlignCenter, label)

        target_end = self._point(center, radius - 38, self.target_deg)
        painter.setPen(QtGui.QPen(QtGui.QColor("#f5a623"), 3, QtCore.Qt.DashLine))
        painter.drawLine(center, target_end)

        actual_end = self._point(center, radius - 30, self.actual_deg)
        painter.setPen(QtGui.QPen(QtGui.QColor("#39c5bb"), 5))
        painter.drawLine(center, actual_end)
        painter.setBrush(QtGui.QColor("#39c5bb"))
        painter.drawEllipse(center, 6, 6)

        painter.setPen(QtGui.QColor("#eef4fb"))
        painter.setFont(QtGui.QFont("Consolas", 11))
        suffix = "已校准正前" if self.front_calibrated else "临时FOC零点"
        painter.drawText(12, self.height() - 14,
                         f"实际 {self.actual_deg:+7.2f}°  目标 {self.target_deg:+7.2f}°  {suffix}")
        painter.end()


@dataclass
class TestSegment:
    mode: str
    target: float
    duration_s: float
    label: str


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Chieftain MK10 AS5600 / SimpleFOC 调试工具")
        self.resize(1480, 920)

        self.serial_port: serial.Serial | None = None
        self.rx_buffer = bytearray()
        self.auto_worker: AutoConnectWorker | None = None
        self.last_rx_host = 0.0
        self.latest: dict[str, float] = {name: 0.0 for name in TELEMETRY_FIELDS}
        self.data: dict[str, deque[float]] = {
            name: deque(maxlen=MAX_POINTS) for name in TELEMETRY_FIELDS
        }
        self.data["t"] = deque(maxlen=MAX_POINTS)
        self.session_samples: list[dict[str, float]] = []
        self.first_device_time: float | None = None

        self.active_mode: str | None = None
        self.active_target = 0.0
        self.front_offset_deg: float | None = None

        self.test_segments: list[TestSegment] = []
        self.test_index = -1
        self.test_segment_started = 0.0
        self.test_windows: list[tuple[TestSegment, float, float]] = []
        self.test_sample_start = 0

        self._build_ui()
        self._refresh_ports()

        self.io_timer = QtCore.QTimer(self)
        self.io_timer.timeout.connect(self._poll_serial)
        self.io_timer.start(20)

        self.command_timer = QtCore.QTimer(self)
        self.command_timer.timeout.connect(self._command_tick)
        self.command_timer.start(20)

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
        refresh_btn = QtWidgets.QPushButton("刷新串口")
        self.auto_btn = QtWidgets.QPushButton("自动连接")
        self.connect_btn = QtWidgets.QPushButton("连接")
        self.connection_label = QtWidgets.QLabel("未连接")
        top.addWidget(QtWidgets.QLabel("串口（USB/蓝牙）"))
        top.addWidget(self.port_combo, 1)
        top.addWidget(refresh_btn)
        top.addWidget(self.auto_btn)
        top.addWidget(self.connect_btn)
        top.addWidget(self.connection_label)
        refresh_btn.clicked.connect(self._refresh_ports)
        self.auto_btn.clicked.connect(self._start_auto_connect)
        self.connect_btn.clicked.connect(self._toggle_connection)

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)

        controls_widget = QtWidgets.QWidget()
        controls = QtWidgets.QVBoxLayout(controls_widget)
        controls_scroll = QtWidgets.QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setWidget(controls_widget)
        controls_scroll.setMinimumWidth(430)
        controls_scroll.setMaximumWidth(510)
        body.addWidget(controls_scroll)

        safety = QtWidgets.QGroupBox("状态与安全")
        safety_grid = QtWidgets.QGridLayout(safety)
        self.state_label = QtWidgets.QLabel("状态：--")
        self.mode_label = QtWidgets.QLabel("模式：--")
        self.power_label = QtWidgets.QLabel("动力电源：--")
        self.sensor_label = QtWidgets.QLabel("AS5600：--")
        self.loop_label = QtWidgets.QLabel("FOC频率：--")
        self.error_label = QtWidgets.QLabel("I²C错误：--")
        safety_grid.addWidget(self.state_label, 0, 0)
        safety_grid.addWidget(self.mode_label, 0, 1)
        safety_grid.addWidget(self.power_label, 1, 0)
        safety_grid.addWidget(self.sensor_label, 1, 1)
        safety_grid.addWidget(self.loop_label, 2, 0)
        safety_grid.addWidget(self.error_label, 2, 1)

        self.align_btn = QtWidgets.QPushButton("确认安全并执行FOC对齐")
        self.zero_btn = QtWidgets.QPushButton("当前位置设为控制零点")
        self.clear_fault_btn = QtWidgets.QPushButton("清除故障锁存")
        self.stop_btn = QtWidgets.QPushButton("紧急停止（Esc）")
        self.stop_btn.setStyleSheet("background:#a82d2d;color:white;font-weight:bold;padding:8px")
        safety_grid.addWidget(self.align_btn, 3, 0, 1, 2)
        safety_grid.addWidget(self.zero_btn, 4, 0)
        safety_grid.addWidget(self.clear_fault_btn, 4, 1)
        safety_grid.addWidget(self.stop_btn, 5, 0, 1, 2)
        self.align_btn.clicked.connect(self._align_foc)
        self.zero_btn.clicked.connect(lambda: self._send("ZERO"))
        self.clear_fault_btn.clicked.connect(lambda: self._send("CLEAR"))
        self.stop_btn.clicked.connect(self._emergency_stop)
        controls.addWidget(safety)

        limit_group = QtWidgets.QGroupBox("输出限制")
        limit_layout = QtWidgets.QHBoxLayout(limit_group)
        self.voltage_limit_spin = QtWidgets.QDoubleSpinBox()
        self.voltage_limit_spin.setRange(0.5, 6.0)
        self.voltage_limit_spin.setSingleStep(0.25)
        self.voltage_limit_spin.setValue(1.0)
        apply_limit_btn = QtWidgets.QPushButton("应用电压上限")
        limit_layout.addWidget(QtWidgets.QLabel("q轴电压上限/V"))
        limit_layout.addWidget(self.voltage_limit_spin)
        limit_layout.addWidget(apply_limit_btn)
        apply_limit_btn.clicked.connect(
            lambda: self._send(f"LIMIT,{self.voltage_limit_spin.value():.3f}"))
        controls.addWidget(limit_group)

        direct_group = QtWidgets.QGroupBox("直接q轴电压点动（最长3秒）")
        direct_layout = QtWidgets.QGridLayout(direct_group)
        self.direct_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.direct_voltage_spin.setRange(0.0, 6.0)
        self.direct_voltage_spin.setDecimals(2)
        self.direct_voltage_spin.setSingleStep(0.1)
        self.direct_voltage_spin.setValue(0.4)
        self.direct_negative_btn = QtWidgets.QPushButton("按住负向")
        self.direct_positive_btn = QtWidgets.QPushButton("按住正向")
        direct_layout.addWidget(QtWidgets.QLabel("电压幅值/V"), 0, 0)
        direct_layout.addWidget(self.direct_voltage_spin, 0, 1)
        direct_layout.addWidget(self.direct_negative_btn, 1, 0)
        direct_layout.addWidget(self.direct_positive_btn, 1, 1)
        self.direct_negative_btn.pressed.connect(lambda: self._start_direct(-1.0))
        self.direct_positive_btn.pressed.connect(lambda: self._start_direct(1.0))
        self.direct_negative_btn.released.connect(self._stop_output)
        self.direct_positive_btn.released.connect(self._stop_output)
        controls.addWidget(direct_group)

        closed_loop = QtWidgets.QGroupBox("速度/位置闭环")
        closed_grid = QtWidgets.QGridLayout(closed_loop)
        self.velocity_spin = QtWidgets.QDoubleSpinBox()
        self.velocity_spin.setRange(-25.0, 25.0)
        self.velocity_spin.setSuffix(" °/s")
        self.velocity_spin.setValue(2.0)
        velocity_start_btn = QtWidgets.QPushButton("开始速度控制")
        self.position_spin = QtWidgets.QDoubleSpinBox()
        self.position_spin.setRange(-90.0, 90.0)
        self.position_spin.setSuffix(" °")
        self.position_spin.setValue(5.0)
        position_start_btn = QtWidgets.QPushButton("开始位置控制")
        stop_output_btn = QtWidgets.QPushButton("停止输出")
        closed_grid.addWidget(QtWidgets.QLabel("目标速度"), 0, 0)
        closed_grid.addWidget(self.velocity_spin, 0, 1)
        closed_grid.addWidget(velocity_start_btn, 0, 2)
        closed_grid.addWidget(QtWidgets.QLabel("目标位置"), 1, 0)
        closed_grid.addWidget(self.position_spin, 1, 1)
        closed_grid.addWidget(position_start_btn, 1, 2)
        closed_grid.addWidget(stop_output_btn, 2, 0, 1, 3)
        velocity_start_btn.clicked.connect(self._start_velocity)
        position_start_btn.clicked.connect(self._start_position)
        stop_output_btn.clicked.connect(self._stop_output)
        controls.addWidget(closed_loop)

        pid_group = QtWidgets.QGroupBox("串级PID（RAM，重启恢复默认）")
        pid_grid = QtWidgets.QGridLayout(pid_group)
        self.pid_spins: dict[str, QtWidgets.QDoubleSpinBox] = {}
        specs = [
            ("outer_kp", "外环 KP", 0.0, 20.0, 0.4, 0.1),
            ("outer_kd", "外环 KD", 0.0, 5.0, 0.0, 0.02),
            ("inner_kp", "内环 KP", 0.0, 5.0, 0.01, 0.005),
            ("inner_ki", "内环 KI", 0.0, 5.0, 0.0, 0.005),
            ("inner_kd", "内环 KD", 0.0, 1.0, 0.0, 0.001),
        ]
        for row, (key, label, minimum, maximum, value, step) in enumerate(specs):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(minimum, maximum)
            spin.setDecimals(5)
            spin.setSingleStep(step)
            spin.setValue(value)
            self.pid_spins[key] = spin
            pid_grid.addWidget(QtWidgets.QLabel(label), row, 0)
            pid_grid.addWidget(spin, row, 1)
        pid_apply = QtWidgets.QPushButton("应用参数")
        pid_read = QtWidgets.QPushButton("读取参数")
        pid_default = QtWidgets.QPushButton("恢复固件默认")
        pid_grid.addWidget(pid_apply, 5, 0, 1, 2)
        pid_grid.addWidget(pid_read, 6, 0)
        pid_grid.addWidget(pid_default, 6, 1)
        pid_apply.clicked.connect(self._apply_pid)
        pid_read.clicked.connect(lambda: self._send("PID,GET"))
        pid_default.clicked.connect(lambda: self._send("PID,DEFAULT"))
        controls.addWidget(pid_group)

        test_group = QtWidgets.QGroupBox("自动性能测试")
        test_layout = QtWidgets.QVBoxLayout(test_group)
        step_btn = QtWidgets.QPushButton("运行 ±15°位置阶跃")
        sweep_btn = QtWidgets.QPushButton("运行 ±5/10/15/22.5°/s扫速")
        cancel_test_btn = QtWidgets.QPushButton("取消测试")
        self.test_result = QtWidgets.QPlainTextEdit()
        self.test_result.setReadOnly(True)
        self.test_result.setMaximumHeight(145)
        test_layout.addWidget(step_btn)
        test_layout.addWidget(sweep_btn)
        test_layout.addWidget(cancel_test_btn)
        test_layout.addWidget(self.test_result)
        step_btn.clicked.connect(self._start_step_test)
        sweep_btn.clicked.connect(self._start_sweep_test)
        cancel_test_btn.clicked.connect(self._cancel_test)
        controls.addWidget(test_group)

        calibration = QtWidgets.QGroupBox("正前方与后甲板校准")
        calibration_grid = QtWidgets.QGridLayout(calibration)
        self.sensor_sign_combo = QtWidgets.QComboBox()
        self.sensor_sign_combo.addItem("+1", 1.0)
        self.sensor_sign_combo.addItem("-1", -1.0)
        capture_front_btn = QtWidgets.QPushButton("将当前原始角设为正前")
        copy_config_btn = QtWidgets.QPushButton("复制RobotConfig参数")
        self.front_label = QtWidgets.QLabel("正前偏移：尚未捕获")
        calibration_grid.addWidget(QtWidgets.QLabel("相对角符号"), 0, 0)
        calibration_grid.addWidget(self.sensor_sign_combo, 0, 1)
        calibration_grid.addWidget(capture_front_btn, 1, 0, 1, 2)
        calibration_grid.addWidget(copy_config_btn, 2, 0, 1, 2)
        calibration_grid.addWidget(self.front_label, 3, 0, 1, 2)
        capture_front_btn.clicked.connect(self._capture_front)
        copy_config_btn.clicked.connect(self._copy_config)
        self.sensor_sign_combo.currentIndexChanged.connect(self._refresh_dial)
        controls.addWidget(calibration)

        data_group = QtWidgets.QGroupBox("数据")
        data_layout = QtWidgets.QHBoxLayout(data_group)
        export_btn = QtWidgets.QPushButton("导出CSV")
        clear_data_btn = QtWidgets.QPushButton("清空曲线")
        data_layout.addWidget(export_btn)
        data_layout.addWidget(clear_data_btn)
        export_btn.clicked.connect(self._export_csv)
        clear_data_btn.clicked.connect(self._clear_data)
        controls.addWidget(data_group)

        self.event_log = QtWidgets.QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumHeight(130)
        controls.addWidget(self.event_log)
        controls.addStretch(1)

        right = QtWidgets.QVBoxLayout()
        body.addLayout(right, 1)
        self.dial = TurretDial()
        right.addWidget(self.dial, 2)

        plots = pg.GraphicsLayoutWidget()
        plots.setBackground("#111722")
        right.addWidget(plots, 3)

        self.angle_plot = plots.addPlot(row=0, col=0, title="位置角 / 误差（°）")
        self.angle_plot.showGrid(x=True, y=True, alpha=0.25)
        self.angle_plot.addLegend()
        self.angle_target_curve = self.angle_plot.plot(pen=pg.mkPen("#f5a623", width=2), name="目标")
        self.angle_actual_curve = self.angle_plot.plot(pen=pg.mkPen("#39c5bb", width=2), name="实际")
        self.angle_error_curve = self.angle_plot.plot(pen=pg.mkPen("#d26bd4", width=1), name="误差")

        self.speed_plot = plots.addPlot(row=1, col=0, title="角速度（°/s）")
        self.speed_plot.showGrid(x=True, y=True, alpha=0.25)
        self.speed_plot.addLegend()
        self.speed_target_curve = self.speed_plot.plot(pen=pg.mkPen("#f5a623", width=2), name="目标")
        self.speed_actual_curve = self.speed_plot.plot(pen=pg.mkPen("#4b8df8", width=2), name="实际")

        self.voltage_plot = plots.addPlot(row=2, col=0, title="q轴输出 / 动力电压（V）")
        self.voltage_plot.showGrid(x=True, y=True, alpha=0.25)
        self.voltage_plot.addLegend()
        self.voltage_curve = self.voltage_plot.plot(pen=pg.mkPen("#e94f4f", width=2), name="q轴电压")
        self.battery_curve = self.voltage_plot.plot(pen=pg.mkPen("#76d275", width=1), name="动力电压")

    def _log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.event_log.appendPlainText(f"[{stamp}] {text}")

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        self.port_combo.clear()
        for info in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{info.device}  {info.description}", info.device)
        if current:
            index = self.port_combo.findData(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def _start_auto_connect(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            return
        if self.auto_worker and self.auto_worker.isRunning():
            return
        self.auto_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.connection_label.setText("正在搜索调试固件……")
        self.auto_worker = AutoConnectWorker(parent=self)
        self.auto_worker.device_found.connect(self._auto_connected)
        self.auto_worker.progress.connect(self.connection_label.setText)
        self.auto_worker.failed.connect(self.connection_label.setText)
        self.auto_worker.finished.connect(self._auto_finished)
        self.auto_worker.start()

    def _auto_finished(self) -> None:
        self.auto_btn.setEnabled(True)
        self.connect_btn.setEnabled(True)

    def _auto_connected(self, port_object: object, port_name: str) -> None:
        if not isinstance(port_object, serial.Serial):
            self.connection_label.setText("自动连接返回无效串口")
            return
        self._adopt_connection(port_object, port_name)

    def _adopt_connection(self, port: serial.Serial, port_name: str) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = port
        self.rx_buffer.clear()
        self.last_rx_host = time.monotonic()
        self.connection_label.setText(f"已连接：{port_name}")
        self.connect_btn.setText("断开")
        self._log(f"连接到 {port_name}")
        self._send("RUN,0")
        self._send("PID,GET")

    def _toggle_connection(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self._disconnect()
            return
        port_name = self.port_combo.currentData()
        if not port_name:
            return
        if self.auto_worker and self.auto_worker.isRunning():
            return
        self.auto_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.connection_label.setText(f"正在连接 {port_name}……")
        self.auto_worker = AutoConnectWorker(requested_port=port_name, parent=self)
        self.auto_worker.device_found.connect(self._auto_connected)
        self.auto_worker.progress.connect(self.connection_label.setText)
        self.auto_worker.failed.connect(self.connection_label.setText)
        self.auto_worker.finished.connect(self._auto_finished)
        self.auto_worker.start()

    def _disconnect(self) -> None:
        self._stop_output()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"STOP\n")
            except (serial.SerialException, OSError):
                pass
            self.serial_port.close()
        self.serial_port = None
        self.rx_buffer.clear()
        self.connect_btn.setText("连接")
        self.connection_label.setText("未连接")
        self._log("串口已断开")

    def _send(self, line: str) -> bool:
        if not self.serial_port or not self.serial_port.is_open:
            return False
        try:
            self.serial_port.write((line + "\n").encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            self._log(f"发送失败：{exc}")
            self.serial_port.close()
            self.serial_port = None
            self.connection_label.setText("连接已丢失")
            self.connect_btn.setText("连接")
            self.active_mode = None
            return False

    def _poll_serial(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            return
        try:
            waiting = self.serial_port.in_waiting
            if waiting:
                self.rx_buffer.extend(self.serial_port.read(waiting))
            while b"\n" in self.rx_buffer:
                raw, _, remainder = self.rx_buffer.partition(b"\n")
                self.rx_buffer = bytearray(remainder)
                line = raw.decode("ascii", errors="ignore").strip("\r \t")
                if line:
                    self._handle_line(line)
            if len(self.rx_buffer) > 4096:
                self.rx_buffer.clear()
                self._log("串口接收缓冲区异常，已清空未完成数据")
        except (serial.SerialException, OSError) as exc:
            self._log(f"读取失败：{exc}")
            self._disconnect()

    def _handle_line(self, line: str) -> None:
        self.last_rx_host = time.monotonic()
        if line.startswith("TEL,"):
            self._handle_telemetry(line)
            return
        if line.startswith("PID,VALUE,"):
            parts = line.split(",")
            if len(parts) == 7:
                try:
                    values = [float(value) for value in parts[2:]]
                except ValueError:
                    return
                for key, value in zip(self.pid_spins, values):
                    self.pid_spins[key].setValue(value)
                self._log("已读取ESP32 PID参数")
            return
        if line.startswith("FAULT,"):
            self.active_mode = None
            self._cancel_test(send_stop=False)
            self._log(f"固件故障锁存：{line}")
            return
        if line.startswith(("ACK,", "ERR,", "BOOT,")):
            self._log(line)

    def _handle_telemetry(self, line: str) -> None:
        parts = line.split(",")[1:]
        if len(parts) != len(TELEMETRY_FIELDS):
            return
        try:
            values = {name: float(text) for name, text in zip(TELEMETRY_FIELDS, parts)}
        except ValueError:
            return

        self.latest = values
        device_time = values["time_ms"] * 0.001
        if self.first_device_time is None:
            self.first_device_time = device_time
        relative_time = device_time - self.first_device_time
        self.data["t"].append(relative_time)
        for name, value in values.items():
            self.data[name].append(value)
        sample = dict(values)
        sample["host_time"] = time.time()
        self.session_samples.append(sample)
        if len(self.session_samples) > 50000:
            del self.session_samples[:10000]

        state = int(values["state"])
        mode = int(values["mode"])
        self.state_label.setText(f"状态：{STATE_NAMES.get(state, state)}")
        self.mode_label.setText(f"模式：{MODE_NAMES.get(mode, mode)}")
        self.power_label.setText(f"动力电源：{values['battery_v']:.2f} V")
        self.sensor_label.setText(
            f"AS5600：{'正常' if values['sensor_ok'] else '异常'}  raw={int(values['raw_count'])}")
        self.loop_label.setText(f"FOC频率：{values['foc_hz']:.0f} Hz")
        self.error_label.setText(
            f"I²C错误：{int(values['i2c_errors'])}  堵转={int(values['stall_latched'])}")

        if state == 4:
            self.active_mode = None
        self._refresh_dial()

    def _refresh_dial(self) -> None:
        raw_deg = self.latest.get("raw_deg", 0.0)
        if self.front_offset_deg is not None:
            sign = float(self.sensor_sign_combo.currentData())
            actual = wrap180((raw_deg - self.front_offset_deg) * sign)
            calibrated = True
        else:
            actual = self.latest.get("position_deg", 0.0)
            calibrated = False
        self.dial.set_angles(actual, self.latest.get("target_position_deg", 0.0), calibrated)

    def _command_tick(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            return
        self._send("HB")
        self._update_auto_test()
        if self.active_mode is not None:
            self._send(f"SET,{self.active_target:.4f}")
            self._send("RUN,1")

    def _align_foc(self) -> None:
        answer = QtWidgets.QMessageBox.warning(
            self,
            "执行FOC对齐",
            "FOC对齐会让炮塔电机短暂转动。\n\n"
            "请确认：\n"
            "1. 炮塔周围没有手、线缆和障碍物；\n"
            "2. 已连接12V动力电源；\n"
            "3. 当前电压上限从1V开始；\n"
            "4. 可以立即切断12V电源。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer == QtWidgets.QMessageBox.Yes:
            self._stop_output()
            self._send("ALIGN")

    def _set_active(self, mode: str, target: float) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            return
        self.active_mode = mode
        self.active_target = target
        self._send(f"MODE,{mode}")
        self._send(f"SET,{target:.4f}")
        self._send("RUN,1")

    def _start_direct(self, sign: float) -> None:
        self._cancel_test(send_stop=False)
        magnitude = min(self.direct_voltage_spin.value(), self.voltage_limit_spin.value())
        self._set_active("VOLTAGE", sign * magnitude)

    def _start_velocity(self) -> None:
        self._cancel_test(send_stop=False)
        self._set_active("VELOCITY", self.velocity_spin.value())

    def _start_position(self) -> None:
        self._cancel_test(send_stop=False)
        self._set_active("POSITION", self.position_spin.value())

    def _stop_output(self) -> None:
        self.active_mode = None
        self.active_target = 0.0
        self._send("RUN,0")

    def _emergency_stop(self) -> None:
        self.active_mode = None
        self.active_target = 0.0
        self._cancel_test(send_stop=False)
        self._send("STOP")
        self._log("已发送紧急停止")

    def _apply_pid(self) -> None:
        values = [self.pid_spins[key].value() for key in self.pid_spins]
        self._send("PID,SET," + ",".join(f"{value:.5f}" for value in values))

    def _capture_front(self) -> None:
        self.front_offset_deg = self.latest.get("raw_deg", 0.0)
        self.front_label.setText(f"正前偏移：{self.front_offset_deg:.3f}°")
        self._refresh_dial()

    def _copy_config(self) -> None:
        if self.front_offset_deg is None:
            QtWidgets.QMessageBox.information(self, "尚未校准", "请先把炮塔指向正前并捕获原始角。")
            return
        sign = float(self.sensor_sign_combo.currentData())
        text = (
            f"const float TURRET_FRONT_SENSOR_OFFSET = {self.front_offset_deg:.3f}f;\n"
            f"const float TURRET_SENSOR_SIGN = {sign:.1f}f;"
        )
        QtWidgets.QApplication.clipboard().setText(text)
        self._log("已复制RobotConfig方位参数")

    def _start_step_test(self) -> None:
        answer = QtWidgets.QMessageBox.question(
            self,
            "位置阶跃测试",
            "工具将把当前位置设为0°，依次执行0/+15/0/-15/0°。\n"
            "确认炮塔至少可以在当前位置附近±20°自由转动。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self._send("ZERO")
        self.test_segments = [
            TestSegment("POSITION", 0.0, 1.0, "零点"),
            TestSegment("POSITION", 15.0, 2.5, "+15°"),
            TestSegment("POSITION", 0.0, 2.0, "回零1"),
            TestSegment("POSITION", -15.0, 2.5, "-15°"),
            TestSegment("POSITION", 0.0, 2.0, "回零2"),
        ]
        self._begin_test("位置阶跃测试")

    def _start_sweep_test(self) -> None:
        answer = QtWidgets.QMessageBox.question(
            self,
            "速度扫描测试",
            "工具将依次运行±5/10/15/22.5°/s，每档约2秒。\n"
            "请确认炮塔能够连续旋转，线缆和机构不会缠绕。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        targets = [0.0, 5.0, 10.0, 15.0, 22.5, 0.0, -5.0, -10.0, -15.0, -22.5, 0.0]
        self.test_segments = [
            TestSegment("VELOCITY", target, 1.0 if target == 0.0 else 2.0, f"{target:+.1f}°/s")
            for target in targets
        ]
        self._begin_test("速度扫描测试")

    def _begin_test(self, name: str) -> None:
        self._stop_output()
        self.test_index = 0
        self.test_segment_started = time.monotonic()
        self.test_windows = []
        self.test_sample_start = len(self.session_samples)
        first = self.test_segments[0]
        self.active_mode = first.mode
        self.active_target = first.target
        self._send(f"MODE,{first.mode}")
        self.test_result.setPlainText(f"{name}运行中……")
        self._log(f"开始{name}")

    def _update_auto_test(self) -> None:
        if self.test_index < 0 or not self.test_segments:
            return
        now = time.monotonic()
        segment = self.test_segments[self.test_index]
        elapsed = now - self.test_segment_started
        if elapsed < segment.duration_s:
            self.active_mode = segment.mode
            self.active_target = segment.target
            return

        end_device_ms = self.latest.get("time_ms", 0.0)
        start_device_ms = end_device_ms - elapsed * 1000.0
        self.test_windows.append((segment, start_device_ms, end_device_ms))
        self.test_index += 1
        if self.test_index >= len(self.test_segments):
            completed = self.test_segments[0].mode
            self.test_index = -1
            self.active_mode = None
            self.active_target = 0.0
            self._send("RUN,0")
            self._finish_test(completed)
            return

        next_segment = self.test_segments[self.test_index]
        if next_segment.mode != segment.mode:
            self._send(f"MODE,{next_segment.mode}")
        self.test_segment_started = now
        self.active_mode = next_segment.mode
        self.active_target = next_segment.target

    def _cancel_test(self, send_stop: bool = True) -> None:
        if self.test_index >= 0:
            self._log("自动测试已取消")
        self.test_index = -1
        self.test_segments = []
        self.test_windows = []
        if send_stop:
            self._stop_output()

    def _finish_test(self, test_mode: str) -> None:
        samples = self.session_samples[self.test_sample_start:]
        if not samples:
            self.test_result.setPlainText("测试结束，但没有收到遥测样本。")
            return
        if test_mode == "POSITION":
            text = self._position_test_report(samples)
        else:
            text = self._velocity_test_report(samples)
        self.test_result.setPlainText(text)
        self._log("自动性能测试完成")

    def _position_test_report(self, samples: list[dict[str, float]]) -> str:
        lines = ["位置阶跃结果："]
        for segment, start_ms, end_ms in self.test_windows:
            if abs(segment.target) < 0.1:
                continue
            selected = [s for s in samples if start_ms <= s["time_ms"] <= end_ms]
            if len(selected) < 4:
                continue
            start_actual = selected[0]["position_deg"]
            amplitude = segment.target - start_actual
            direction = 1.0 if amplitude >= 0.0 else -1.0
            threshold10 = start_actual + amplitude * 0.1
            threshold90 = start_actual + amplitude * 0.9
            t10 = next((s["time_ms"] for s in selected
                        if (s["position_deg"] - threshold10) * direction >= 0.0), None)
            t90 = next((s["time_ms"] for s in selected
                        if (s["position_deg"] - threshold90) * direction >= 0.0), None)
            rise = (t90 - t10) * 0.001 if t10 is not None and t90 is not None else math.nan
            overshoot = max(0.0, max((s["position_deg"] - segment.target) * direction for s in selected))
            tail = selected[max(0, len(selected) - 10):]
            steady_error = statistics.fmean(abs(s["position_error_deg"]) for s in tail)
            max_voltage = max(abs(s["voltage_v"]) for s in selected)
            rise_text = f"{rise:.3f}s" if math.isfinite(rise) else "未到90%"
            lines.append(
                f"{segment.label}: 上升={rise_text}  超调={overshoot:.2f}°  "
                f"末端误差={steady_error:.2f}°  峰值电压={max_voltage:.2f}V"
            )
        return "\n".join(lines)

    def _velocity_test_report(self, samples: list[dict[str, float]]) -> str:
        lines = ["速度扫描结果："]
        for segment, start_ms, end_ms in self.test_windows:
            if abs(segment.target) < 0.1:
                continue
            # 舍弃每档开始的0.5秒，统计相对稳定部分。
            selected = [
                s for s in samples
                if start_ms + 500.0 <= s["time_ms"] <= end_ms
            ]
            if len(selected) < 4:
                continue
            speeds = [s["actual_velocity_dps"] for s in selected]
            voltages = [abs(s["voltage_v"]) for s in selected]
            average = statistics.fmean(speeds)
            ripple = statistics.pstdev(speeds)
            average_voltage = statistics.fmean(voltages)
            lines.append(
                f"{segment.label}: 实际={average:+.2f}°/s  "
                f"误差={segment.target - average:+.2f}°/s  "
                f"波动σ={ripple:.2f}  平均电压={average_voltage:.2f}V"
            )
        return "\n".join(lines)

    def _update_plots(self) -> None:
        x = list(self.data["t"])
        if not x:
            return
        self.angle_target_curve.setData(x, list(self.data["target_position_deg"]))
        self.angle_actual_curve.setData(x, list(self.data["position_deg"]))
        self.angle_error_curve.setData(x, list(self.data["position_error_deg"]))
        self.speed_target_curve.setData(x, list(self.data["target_velocity_dps"]))
        self.speed_actual_curve.setData(x, list(self.data["actual_velocity_dps"]))
        self.voltage_curve.setData(x, list(self.data["voltage_v"]))
        self.battery_curve.setData(x, list(self.data["battery_v"]))

    def _clear_data(self) -> None:
        for values in self.data.values():
            values.clear()
        self.session_samples.clear()
        self.first_device_time = None
        self.test_result.clear()

    def _export_csv(self) -> None:
        if not self.session_samples:
            QtWidgets.QMessageBox.information(self, "没有数据", "当前还没有可导出的遥测数据。")
            return
        default_name = Path.cwd() / time.strftime("as5600_foc_%Y%m%d_%H%M%S.csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出CSV", str(default_name), "CSV files (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=["host_time", *TELEMETRY_FIELDS])
            writer.writeheader()
            writer.writerows(self.session_samples)
        self._log(f"已导出CSV：{path}")

    def event(self, event: QtCore.QEvent) -> bool:
        if event.type() == QtCore.QEvent.WindowDeactivate and hasattr(self, "active_mode"):
            if self.active_mode is not None:
                self._stop_output()
                self._cancel_test(send_stop=False)
                self._log("窗口失去焦点，输出已归零")
        return super().event(event)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.key() == QtCore.Qt.Key_Escape:
            self._emergency_stop()
            event.accept()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.auto_worker and self.auto_worker.isRunning():
            self.auto_worker.requestInterruption()
            self.auto_worker.wait(5000)
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"STOP\n")
            except (serial.SerialException, OSError):
                pass
            self.serial_port.close()
        event.accept()


def main() -> int:
    pg.setConfigOptions(antialias=True)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Chieftain AS5600 FOC Debug")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
