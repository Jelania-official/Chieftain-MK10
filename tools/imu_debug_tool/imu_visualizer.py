import sys
from collections import deque

import serial
from serial.tools import list_ports

from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

try:
    import pyqtgraph.opengl as gl
    OPENGL_AVAILABLE = True
except Exception:
    gl = None
    OPENGL_AVAILABLE = False


BAUD_RATE = 115200
MAX_POINTS = 1000


class AttitudeWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.setMinimumHeight(260)

    def set_attitude(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect()
        cx = rect.width() / 2
        cy = rect.height() / 2

        painter.fillRect(rect, QtGui.QColor(18, 22, 30))

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll)

        pitch_offset = max(-80.0, min(80.0, self.pitch * 2.0))
        sky = QtGui.QColor(64, 118, 180)
        ground = QtGui.QColor(122, 84, 45)
        painter.fillRect(QtCore.QRectF(-1000, -1000 + pitch_offset, 2000, 1000), sky)
        painter.fillRect(QtCore.QRectF(-1000, pitch_offset, 2000, 1000), ground)

        pen = QtGui.QPen(QtGui.QColor(240, 240, 240), 3)
        painter.setPen(pen)
        painter.drawLine(QtCore.QPointF(-220, pitch_offset), QtCore.QPointF(220, pitch_offset))

        painter.restore()

        painter.setPen(QtGui.QPen(QtGui.QColor(255, 210, 80), 3))
        painter.drawLine(QtCore.QPointF(cx - 45, cy), QtCore.QPointF(cx - 10, cy))
        painter.drawLine(QtCore.QPointF(cx + 10, cy), QtCore.QPointF(cx + 45, cy))
        painter.drawEllipse(QtCore.QPointF(cx, cy), 4, 4)

        painter.setPen(QtGui.QColor(235, 238, 245))
        painter.setFont(QtGui.QFont("Consolas", 12))
        painter.drawText(16, 28, f"Roll  {self.roll:7.2f}°")
        painter.drawText(16, 52, f"Pitch {self.pitch:7.2f}°")
        painter.drawText(16, 76, f"Yaw   {self.yaw:7.2f}°")
        painter.end()


class ImuModel3D(QtWidgets.QWidget):
    """用四元数驱动的简单 3D 车体模型。

    这里显示的不是完整坦克外观，而是一个带“车头方向”的姿态参考块：
    长轴代表车体前后方向，左右两侧履带块帮助判断 roll，炮管短杆帮助判断 yaw。
    """

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(260)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not OPENGL_AVAILABLE:
            label = QtWidgets.QLabel(
                "3D 视图不可用：缺少 OpenGL 依赖。\n"
                "请运行：python -m pip install -r tools\\imu_debug_tool\\requirements.txt"
            )
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setStyleSheet("color: #e8eef8; background: #12161e;")
            layout.addWidget(label)
            self.body_items = []
            return

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor(QtGui.QColor(18, 22, 30))
        self.view.opts["distance"] = 7
        self.view.opts["elevation"] = 25
        self.view.opts["azimuth"] = -45
        layout.addWidget(self.view)

        grid = gl.GLGridItem()
        grid.setSize(8, 8)
        grid.setSpacing(1, 1)
        self.view.addItem(grid)

        self.body_items = []
        self._add_box(size=(2.8, 1.4, 0.45), pos=(0, 0, 0.35), color=(0.30, 0.54, 0.34, 1.0))
        self._add_box(size=(2.9, 0.22, 0.28), pos=(0, 0.85, 0.22), color=(0.10, 0.12, 0.13, 1.0))
        self._add_box(size=(2.9, 0.22, 0.28), pos=(0, -0.85, 0.22), color=(0.10, 0.12, 0.13, 1.0))
        self._add_box(size=(0.9, 0.9, 0.35), pos=(0.15, 0, 0.82), color=(0.25, 0.45, 0.28, 1.0))
        self._add_box(size=(1.6, 0.14, 0.14), pos=(1.25, 0, 0.86), color=(0.18, 0.30, 0.20, 1.0))

        axis = gl.GLAxisItem()
        axis.setSize(2.5, 2.5, 2.5)
        self.view.addItem(axis)

        self.set_quaternion(1.0, 0.0, 0.0, 0.0)

    def _cube_mesh_data(self):
        vertices = [
            [-0.5, -0.5, -0.5],
            [ 0.5, -0.5, -0.5],
            [ 0.5,  0.5, -0.5],
            [-0.5,  0.5, -0.5],
            [-0.5, -0.5,  0.5],
            [ 0.5, -0.5,  0.5],
            [ 0.5,  0.5,  0.5],
            [-0.5,  0.5,  0.5],
        ]
        faces = [
            [0, 1, 2], [0, 2, 3],
            [4, 6, 5], [4, 7, 6],
            [0, 4, 5], [0, 5, 1],
            [1, 5, 6], [1, 6, 2],
            [2, 6, 7], [2, 7, 3],
            [3, 7, 4], [3, 4, 0],
        ]
        return gl.MeshData(vertexes=vertices, faces=faces)

    def _add_box(self, size, pos, color):
        mesh = gl.GLMeshItem(
            meshdata=self._cube_mesh_data(),
            smooth=False,
            color=color,
            shader="shaded",
            drawEdges=True,
            edgeColor=(0.02, 0.03, 0.04, 1.0),
        )
        self.view.addItem(mesh)
        self.body_items.append((mesh, size, pos))

    def set_quaternion(self, qw, qx, qy, qz):
        if not OPENGL_AVAILABLE:
            return

        quat = QtGui.QQuaternion(qw, qx, qy, qz).normalized()
        matrix = QtGui.QMatrix4x4()
        matrix.rotate(quat)

        for item, size, pos in self.body_items:
            transform = QtGui.QMatrix4x4(matrix)
            transform.translate(*pos)
            transform.scale(*size)
            item.setTransform(transform)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Chieftain MK10 - USB IMU Visualizer")
        self.serial_port = None
        self.rx_buffer = ""

        self.samples = deque(maxlen=MAX_POINTS)
        self.start_ms = None

        self._build_ui()
        self.refresh_ports()
        self.update_connection_state(False)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll_serial)
        self.timer.start(10)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)

        controls = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_button = QtWidgets.QPushButton("刷新串口")
        self.connect_button = QtWidgets.QPushButton("连接")
        self.zero_button = QtWidgets.QPushButton("姿态归零")
        self.cal_button = QtWidgets.QPushButton("重新校准")
        self.status_label = QtWidgets.QLabel("未连接")

        controls.addWidget(QtWidgets.QLabel("串口"))
        controls.addWidget(self.port_combo, 1)
        controls.addWidget(self.refresh_button)
        controls.addWidget(self.connect_button)
        controls.addWidget(self.zero_button)
        controls.addWidget(self.cal_button)
        controls.addWidget(self.status_label)
        layout.addLayout(controls)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        left_panel = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        self.attitude = AttitudeWidget()
        self.model_3d = ImuModel3D()
        left_layout.addWidget(self.attitude)
        left_layout.addWidget(self.model_3d)
        splitter.addWidget(left_panel)

        plots = QtWidgets.QWidget()
        plots_layout = QtWidgets.QVBoxLayout(plots)

        self.att_plot = pg.PlotWidget(title="姿态角 roll / pitch / yaw")
        self.att_plot.addLegend()
        self.att_plot.showGrid(x=True, y=True)
        self.roll_curve = self.att_plot.plot(pen=pg.mkPen("#ff6666", width=2), name="roll")
        self.pitch_curve = self.att_plot.plot(pen=pg.mkPen("#66ccff", width=2), name="pitch")
        self.yaw_curve = self.att_plot.plot(pen=pg.mkPen("#ffcc66", width=2), name="yaw")
        plots_layout.addWidget(self.att_plot)

        self.gyro_plot = pg.PlotWidget(title="陀螺仪 gx / gy / gz (deg/s)")
        self.gyro_plot.addLegend()
        self.gyro_plot.showGrid(x=True, y=True)
        self.gx_curve = self.gyro_plot.plot(pen=pg.mkPen("#ff6666", width=1), name="gx")
        self.gy_curve = self.gyro_plot.plot(pen=pg.mkPen("#66ccff", width=1), name="gy")
        self.gz_curve = self.gyro_plot.plot(pen=pg.mkPen("#ffcc66", width=1), name="gz")
        plots_layout.addWidget(self.gyro_plot)

        self.acc_plot = pg.PlotWidget(title="加速度 ax / ay / az (m/s²)")
        self.acc_plot.addLegend()
        self.acc_plot.showGrid(x=True, y=True)
        self.ax_curve = self.acc_plot.plot(pen=pg.mkPen("#ff6666", width=1), name="ax")
        self.ay_curve = self.acc_plot.plot(pen=pg.mkPen("#66ccff", width=1), name="ay")
        self.az_curve = self.acc_plot.plot(pen=pg.mkPen("#ffcc66", width=1), name="az")
        plots_layout.addWidget(self.acc_plot)

        splitter.addWidget(plots)
        splitter.setSizes([420, 860])
        layout.addWidget(splitter, 1)

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.zero_button.clicked.connect(lambda: self.send_command("ZERO"))
        self.cal_button.clicked.connect(lambda: self.send_command("CAL"))

        self.setCentralWidget(central)
        self.resize(1280, 820)

    def update_connection_state(self, connected):
        self.connect_button.setText("断开" if connected else "连接")
        self.zero_button.setEnabled(connected)
        self.cal_button.setEnabled(connected)
        if not connected:
            self.status_label.setText("未连接")

    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        for port in list_ports.comports():
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
        if current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.update_connection_state(False)
            return

        port = self.port_combo.currentData()
        if not port:
            QtWidgets.QMessageBox.warning(self, "没有串口", "请先选择 ESP32 对应的 USB 串口。")
            return

        try:
            self.serial_port = serial.Serial(
                port, 
                BAUD_RATE, 
                timeout=0,
                # 关键：禁用 DTR/RTS 防止 ESP32 重启
                dsrdtr=False,  # 禁用硬件流控
                rtscts=False
            )
            # 显式设置 DTR 和 RTS
            self.serial_port.dtr = False
            self.serial_port.rts = False
            
            self.samples.clear()
            self.start_ms = None
            self.rx_buffer = ""
            self.update_connection_state(True)
            self.status_label.setText(f"已连接 {port}")
            
            # 添加短暂的延迟让ESP32稳定
            import time
            time.sleep(0.5)
            
        except serial.SerialException as exc:
            QtWidgets.QMessageBox.critical(self, "连接失败", str(exc))

    def send_command(self, command):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write((command + "\n").encode("ascii"))

    def poll_serial(self):
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            chunk = self.serial_port.read(4096)
        except serial.SerialException as exc:
            self.status_label.setText(f"串口错误：{exc}")
            self.serial_port.close()
            self.serial_port = None
            self.update_connection_state(False)
            return

        if chunk:
            # 添加调试输出
            print(f"收到原始数据 ({len(chunk)} 字节): {chunk[:200]}")  # 只打印前200字节
            
        if not chunk:
            return

        self.rx_buffer += chunk.decode("utf-8", errors="ignore")
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            # 添加调试输出
            print(f"解析行: {line[:100]}")  # 只打印前100字符
            self.handle_line(line.strip())

    def handle_line(self, line):
        # 打印所有接收到的行
        print(f"处理行: '{line}'")
        
        if not line:
            return
        if line.startswith("INFO,"):
            print(f"INFO行: {line}")
            self.status_label.setText(line)
            return
        if line.startswith("ERR,"):
            print(f"ERR行: {line}")
            self.status_label.setText(line)
            return
        if not line.startswith("IMU,"):
            print(f"忽略非IMU行: {line[:50]}")
            return

        parts = line.split(",")
        print(f"IMU数据字段数: {len(parts)}, 期望: 17")
        if len(parts) != 17:
            print(f"字段数量不匹配: {len(parts)} != 17")
            print(f"字段内容: {parts}")
            return

        try:
            t_ms = float(parts[1])
            dt = float(parts[2])
            ax, ay, az = map(float, parts[3:6])
            gx, gy, gz = map(float, parts[6:9])
            qw, qx, qy, qz = map(float, parts[9:13])
            roll, pitch, yaw = map(float, parts[13:16])
            temp = float(parts[16])
            print(f"解析成功: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        except ValueError as e:
            print(f"数值解析错误: {e}")
            return

        if self.start_ms is None:
            self.start_ms = t_ms
            print(f"设置起始时间: {t_ms}")
        t = (t_ms - self.start_ms) / 1000.0

        self.samples.append({
            "t": t, "dt": dt,
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "qw": qw, "qx": qx, "qy": qy, "qz": qz,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "temp": temp,
        })
        self.update_view()

    def update_view(self):
        if not self.samples:
            return

        latest = self.samples[-1]
        self.attitude.set_attitude(latest["roll"], latest["pitch"], latest["yaw"])
        self.model_3d.set_quaternion(latest["qw"], latest["qx"], latest["qy"], latest["qz"])

        times = [s["t"] for s in self.samples]
        self.roll_curve.setData(times, [s["roll"] for s in self.samples])
        self.pitch_curve.setData(times, [s["pitch"] for s in self.samples])
        self.yaw_curve.setData(times, [s["yaw"] for s in self.samples])

        self.gx_curve.setData(times, [s["gx"] for s in self.samples])
        self.gy_curve.setData(times, [s["gy"] for s in self.samples])
        self.gz_curve.setData(times, [s["gz"] for s in self.samples])

        self.ax_curve.setData(times, [s["ax"] for s in self.samples])
        self.ay_curve.setData(times, [s["ay"] for s in self.samples])
        self.az_curve.setData(times, [s["az"] for s in self.samples])

        self.status_label.setText(
            f"t={latest['t']:.1f}s  temp={latest['temp']:.1f}°C  "
            f"q=({latest['qw']:.3f},{latest['qx']:.3f},{latest['qy']:.3f},{latest['qz']:.3f})"
        )

    def closeEvent(self, event):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().closeEvent(event)


def main():
    pg.setConfigOptions(antialias=True)
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
