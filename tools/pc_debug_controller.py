import argparse
import asyncio
import html
import json
import math
import queue
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QEvent, QTimer, QPoint, QPointF, QRectF, Signal
from PySide6.QtGui import (
    QPainter, QColor, QPen, QBrush, QPolygonF, QFont, QCursor,
    QDoubleValidator, QTextCursor
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QSlider, QLineEdit, QTextEdit,
    QScrollArea, QTabWidget, QFrame, QSplitter
)
from bleak import BleakClient, BleakScanner


SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# Bright industrial palette inspired by modern device-management utilities.
BG = "#EAF0F5"
SURFACE = "#FFFFFF"
CARD = "#FFFFFF"
ELEVATED = "#EEF3F7"
BORDER = "#CBD7E1"
TEXT = "#17212B"
MUTED = "#687687"
ACCENT = "#1677FF"
ACCENT_HOVER = "#4096FF"
CYAN = "#00A6C7"
GREEN = "#159B62"
YELLOW = "#D98400"
RED = "#D9363E"
GRID = "#E7EDF3"

QSS_THEME = f"""
QWidget {{
    color: {TEXT};
    font-family: "Segoe UI", "Microsoft YaHei UI", sans-serif;
    font-size: 13px;
}}
QWidget#AppRoot {{
    background-color: {BG};
}}
QFrame#TopBar, QFrame#Panel {{
    background-color: {CARD};
    border: 1px solid {BORDER};
    border-radius: 12px;
}}
QFrame#TopBar {{
    border: 1px solid #C4D3E1;
    border-bottom: 3px solid {ACCENT};
}}
QFrame#TelemetryPanel, QFrame#ParameterPanel {{
    background-color: #FFFFFF;
    border: 1px solid #B8CEE5;
    border-top: 3px solid {ACCENT};
    border-radius: 12px;
}}
QFrame#SafetyPanel {{
    background-color: #FFFCF5;
    border: 1px solid #E8C98E;
    border-top: 3px solid {YELLOW};
    border-radius: 12px;
}}
QFrame#ControlPanel {{
    background-color: #FBFCFE;
    border: 1px solid #C8D5E1;
    border-radius: 12px;
}}
QFrame#ConsolePanel {{
    background-color: #F8FBFF;
    border: 1px solid #B9CADD;
    border-top: 3px solid #7D96B2;
    border-radius: 12px;
}}
QFrame#ConsoleAccent {{
    background-color: {ACCENT};
    border: none;
    border-radius: 2px;
}}
QFrame#Inset {{
    background-color: {SURFACE};
    border: 1px solid {BORDER};
    border-radius: 9px;
}}
QFrame#TelemetryTile {{
    background-color: #F8FBFF;
    border: 1px solid #C4D7EB;
    border-radius: 8px;
}}
QFrame#ParameterRow {{
    background: transparent;
    border-bottom: 1px solid {BORDER};
}}
QFrame#ParameterRow[dirty="true"] {{
    background-color: #FFF9E8;
    border-left: 3px solid {YELLOW};
}}
QFrame#ParameterRow[dirty="true"] QLabel#ParameterTitle {{
    color: {YELLOW};
}}
QLabel#Title {{
    font-weight: 700;
    color: {TEXT};
    font-size: 15px;
}}
QLabel#HeroTitle {{
    font-weight: 800;
    color: #101C28;
    font-size: 18px;
}}
QLabel#SectionTitle {{
    font-weight: 750;
    color: #142230;
    font-size: 16px;
}}
QLabel#SectionHint {{
    color: {MUTED};
    font-size: 10px;
}}
QLabel#StatusStrong {{
    font-weight: 750;
    font-size: 13px;
}}
QLabel#SubTitle {{
    font-weight: 600;
    color: {MUTED};
    font-size: 11px;
}}
QLabel#LargeValue {{
    font-family: "Segoe UI Variable Display", "Segoe UI";
    font-weight: 700;
    font-size: 18px;
    color: {TEXT};
}}
QLabel#Mono {{
    font-family: "Cascadia Mono", "Consolas";
}}
QLabel#ParameterTitle {{
    font-weight: 600;
    color: {TEXT};
    font-size: 13px;
}}
QPushButton {{
    background-color: {ELEVATED};
    border: 1px solid {BORDER};
    border-radius: 7px;
    padding: 7px 12px;
    color: {TEXT};
    font-weight: 600;
}}
QPushButton:hover {{
    background-color: #E2EBF4;
    border-color: #B8C7D5;
}}
QPushButton:pressed {{
    background-color: #D7E3EE;
}}
QPushButton#Primary {{
    background-color: {ACCENT};
    border-color: {ACCENT};
    color: #FFFFFF;
}}
QPushButton#Primary:hover {{
    background-color: {ACCENT_HOVER};
    border-color: {ACCENT_HOVER};
}}
QPushButton#Danger {{
    background-color: #FFF1F0;
    border-color: #FFCCC7;
    color: {RED};
}}
QPushButton#Danger:hover {{
    background-color: {RED};
    border-color: {RED};
    color: #FFFFFF;
}}
QPushButton#Quiet {{
    background-color: #FFFFFF;
    border-color: #CBD7E2;
    color: {MUTED};
}}
QPushButton#Secondary {{
    background-color: #EDF4FB;
    border-color: #C5D7E8;
    color: #27445F;
}}
QPushButton#Secondary:hover {{
    background-color: #DFECF8;
    border-color: #AFC7DC;
}}
QPushButton:disabled {{
    background-color: #F3F5F7;
    border-color: #E4E9EE;
    color: #A4AFBA;
}}
QSlider::groove:horizontal {{
    height: 4px;
    background: #DCE5ED;
    border-radius: 2px;
}}
QSlider::sub-page:horizontal {{
    background: {ACCENT};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {TEXT};
    border: 2px solid {ACCENT};
    width: 14px;
    height: 14px;
    margin: -6px 0;
    border-radius: 8px;
}}
QSlider::handle:horizontal:hover {{
    background: #FFFFFF;
}}
QTabWidget::pane {{
    border: 1px solid {BORDER};
    border-radius: 9px;
    background: {SURFACE};
    top: -1px;
}}
QTabBar::tab {{
    background: transparent;
    color: {MUTED};
    padding: 9px 8px;
    font-size: 11px;
    font-weight: 600;
    border-bottom: 2px solid transparent;
}}
QTabBar::tab:selected {{
    color: {TEXT};
    border-bottom: 2px solid {ACCENT};
}}
QTabBar::tab:hover {{
    color: {TEXT};
}}
QLineEdit, QTextEdit {{
    background-color: {SURFACE};
    border: 1px solid {BORDER};
    border-radius: 7px;
    padding: 7px 9px;
    font-family: "Cascadia Mono", "Consolas";
    color: {TEXT};
    selection-background-color: {ACCENT};
}}
QLineEdit:focus, QTextEdit:focus {{
    border-color: {ACCENT};
}}
QTextEdit {{
    font-size: 11px;
    color: #435162;
}}
QTextEdit#ConsoleLog {{
    background-color: #FFFFFF;
    border: 2px solid #C5D4E3;
    border-radius: 8px;
    padding: 9px 11px;
}}
QTextEdit#ConsoleLog:focus {{
    border-color: {ACCENT};
}}
QLineEdit#ConsoleCommand {{
    background-color: #FFFFFF;
    border: 2px solid #C5D4E3;
    border-radius: 8px;
    padding: 7px 10px;
}}
QLineEdit#ConsoleCommand:focus {{
    border-color: {ACCENT};
}}
QScrollArea {{
    border: none;
    background: transparent;
}}
QScrollArea > QWidget > QWidget {{
    background: {SURFACE};
}}
QScrollBar:vertical {{
    border: none;
    background: {SURFACE};
    width: 9px;
    margin: 3px;
}}
QScrollBar::handle:vertical {{
    background: #B8C5D1;
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QSplitter::handle {{
    background: transparent;
}}
QSplitter::handle:vertical {{
    height: 8px;
}}
QSplitter::handle:horizontal {{
    width: 8px;
}}
QToolTip {{
    color: {TEXT};
    background-color: {ELEVATED};
    border: 1px solid {BORDER};
    padding: 5px;
}}
"""

GROUP_ORDER = ["炮塔 Yaw", "炮管 Pitch", "虚拟惯量", "底盘动力学", "履带速度环", "其他"]

PARAM_META = {
    "REAL_TURRET_VEL": ("炮塔 Yaw", "炮塔目标速度", "鼠标满输入时的炮塔目标角速度"),
    "YAW_OUTER_KP": ("炮塔 Yaw", "Yaw 外环 Kp", "方位角误差到目标角速度的比例增益"),
    "YAW_OUTER_KD": ("炮塔 Yaw", "Yaw 外环 Kd", "抑制方位外环过冲，过大可能放大目标变化冲击"),
    "YAW_INNER_KP": ("炮塔 Yaw", "Yaw 内环 Kp", "角速度误差到 FOC 电压命令的比例增益"),
    "YAW_INNER_KI": ("炮塔 Yaw", "Yaw 内环 Ki", "克服持续摩擦和小稳态误差"),
    "YAW_INNER_KD": ("炮塔 Yaw", "Yaw 内环 Kd", "角速度环阻尼，过大容易放大陀螺噪声"),
    "YAW_OUTER_RATE_MAX": ("炮塔 Yaw", "Yaw 外环限速", "外环允许给内环的最大目标角速度"),
    "YAW_VOLTAGE_MAX": ("炮塔 Yaw", "Yaw 电压上限", "给 SimpleFOC torque/voltage 目标的总限幅"),
    "YAW_CHASSIS_FF_GAIN": ("炮塔 Yaw", "底盘 Yaw 前馈", "底盘转动时炮塔提前反向补偿的比例"),

    "PITCH_ACC_TAU": ("炮管 Pitch", "Pitch 纠漂时间常数", "越大越信任陀螺，越不受起步和刹车线性加速度影响"),
    "PITCH_STAB_KP": ("炮管 Pitch", "Pitch 比例增益", "炮管角度误差到舵机目标角速度的比例"),
    "PITCH_STAB_KD": ("炮管 Pitch", "Pitch 角速度阻尼", "压制炮管回正时的过冲和抖动"),
    "PITCH_CHASSIS_FF": ("炮管 Pitch", "底盘 Pitch 前馈", "底盘抬头低头时炮管的提前补偿比例"),
    "PITCH_SERVO_RATE_DEADZONE_DPS": ("炮管 Pitch", "舵机速度死区", "小于该速度命令时不刷新舵机，降低静止嗡鸣"),

    "V_INERTIA_PWM_GAIN": ("虚拟惯量", "Pitch 惯量增益", "底盘俯仰角加速度对应的反向 PWM 增益"),
    "V_INERTIA_PWM_MAX": ("虚拟惯量", "Pitch 惯量上限", "底盘俯仰虚拟惯量允许的最大 PWM 修正"),
    "YAW_INERTIA_PWM_GAIN": ("虚拟惯量", "Yaw 惯量增益", "底盘 yaw 角加速度对应的反向差速 PWM 增益"),
    "YAW_INERTIA_PWM_MAX": ("虚拟惯量", "Yaw 惯量上限", "底盘 yaw 虚拟惯量允许的最大差速 PWM"),

    "REAL_ACCEL": ("底盘动力学", "发动机加速度", "满油门时的真车等效加速度"),
    "REAL_BRAKE": ("底盘动力学", "制动减速度", "满制动时的真车等效减速度"),
    "LINEAR_JERK_ACCEL": ("底盘动力学", "动力建立 Jerk", "越小动力建立越沉重"),
    "LINEAR_JERK_BRAKE": ("底盘动力学", "制动建立 Jerk", "越大制动建立越快"),
    "SLOPE_GRAVITY_MAX": ("底盘动力学", "坡度重力强度", "坡度对纵向加速度的最大影响"),
    "GRADE_PITCH_TAU": ("底盘动力学", "坡度滤波时间常数", "越大越能滤除车身点头和地面冲击"),
    "YAW_SENSITIVITY": ("底盘动力学", "底盘转向灵敏度", "低速时最大差速目标"),
    "SPEED_SENS_K": ("底盘动力学", "随速转向衰减", "越大高速时转向越不敏感"),

    "TRACK_FF_KS_START": ("履带速度环", "起步静摩擦前馈", "履带从静止起转时克服静摩擦的 PWM"),
    "TRACK_FF_KS_RUN": ("履带速度环", "保持静摩擦前馈", "履带已经转起来后用于维持运动的 PWM"),
    "TRACK_FF_KV": ("履带速度环", "速度前馈 Kv", "目标速度对应的 PWM 前馈"),
    "TRACK_FF_KA": ("履带速度环", "加速度前馈 Ka", "目标加速度对应的 PWM 前馈"),
    "TRACK_FF_KSLOPE": ("履带速度环", "坡度前馈", "坡道保持所需的 PWM 前馈"),
    "TRACK_PI_KP": ("履带速度环", "速度环 Kp", "编码器速度误差的比例修正"),
    "TRACK_PI_KI": ("履带速度环", "速度环 Ki", "编码器速度误差的积分修正"),
}

PARAM_PATTERN = re.compile(r"^([A-Z0-9_]+)=([-+0-9.eE]+)\s+\[([-+0-9.eE]+),([-+0-9.eE]+)\]$")
SET_PATTERN = re.compile(r"^OK\s+([A-Z0-9_]+)=([-+0-9.eE]+)")

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class PadState:
    mouse_sensitivity: float = 0.12
    mouse_response_time: float = 0.18
    drive_scale: float = 1.0
    turn_key_scale: float = 0.3
    turret_rate_deg_s: float = 22.5
    keys: set[str] = field(default_factory=set)
    pending_yaw_deg: float = 0.0
    pending_pitch_deg: float = 0.0
    mouse_dx: float = 0.0
    mouse_dy: float = 0.0
    last_update: float = field(default_factory=time.monotonic)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def key_down(self, key: str) -> bool:
        with self.lock:
            is_new = key not in self.keys
            self.keys.add(key)
            return is_new

    def key_up(self, key: str) -> None:
        with self.lock:
            self.keys.discard(key)

    def clear(self) -> None:
        with self.lock:
            self.keys.clear()
            self.pending_yaw_deg = 0.0
            self.pending_pitch_deg = 0.0
            self.mouse_dx = 0.0
            self.mouse_dy = 0.0

    def add_mouse_delta(self, dx: float, dy: float) -> None:
        with self.lock:
            self.mouse_dx += dx
            self.mouse_dy += dy
            self.pending_yaw_deg = clamp(self.pending_yaw_deg + dx * self.mouse_sensitivity, -90.0, 90.0)
            self.pending_pitch_deg = clamp(self.pending_pitch_deg - dy * self.mouse_sensitivity, -45.0, 45.0)

    def set_mouse_sensitivity(self, value: float) -> None:
        with self.lock: self.mouse_sensitivity = value

    def set_drive_scale(self, value: float) -> None:
        with self.lock: self.drive_scale = value

    def set_mouse_response_time(self, value: float) -> None:
        with self.lock: self.mouse_response_time = max(value, 0.02)

    def set_turret_rate(self, value: float) -> None:
        with self.lock: self.turret_rate_deg_s = max(value, 1.0)

    @staticmethod
    def joystick_from_effective(value: float) -> float:
        value = clamp(value, -1.0, 1.0)
        if abs(value) < 1e-4: return 0.0
        return math.copysign(0.15 + 0.85 * abs(value), value)

    @staticmethod
    def consume_pending(pending: float, rate_deg_s: float, dt: float) -> float:
        consumed = rate_deg_s * dt
        if abs(consumed) >= abs(pending): return 0.0
        return pending - consumed

    def snapshot(self) -> tuple[str, dict[str, float]]:
        with self.lock:
            now = time.monotonic()
            dt = clamp(now - self.last_update, 0.001, 0.05)
            self.last_update = now
            keys = set(self.keys)
            drive_scale = self.drive_scale
            turn_key_scale = self.turn_key_scale

            throttle = drive_scale if "w" in keys else 0.0
            brake = drive_scale if "s" in keys else 0.0
            left = -turn_key_scale if "a" in keys else 0.0
            right = turn_key_scale if "d" in keys else 0.0
            joy_lx = clamp(left + right, -1.0, 1.0)
            stabilizer_button = 1 if "space" in keys else 0

            yaw_rate_cmd = clamp(self.pending_yaw_deg / self.mouse_response_time, -self.turret_rate_deg_s, self.turret_rate_deg_s)
            pitch_rate_cmd = clamp(self.pending_pitch_deg / self.mouse_response_time, -self.turret_rate_deg_s, self.turret_rate_deg_s)
            joy_rx = self.joystick_from_effective(yaw_rate_cmd / self.turret_rate_deg_s)
            joy_ry = self.joystick_from_effective(-pitch_rate_cmd / self.turret_rate_deg_s)

            self.pending_yaw_deg = self.consume_pending(self.pending_yaw_deg, yaw_rate_cmd, dt)
            self.pending_pitch_deg = self.consume_pending(self.pending_pitch_deg, pitch_rate_cmd, dt)

            sent_brake, sent_throttle = round(brake, 3), round(throttle, 3)
            sent_left_x, sent_right_x, sent_right_y = round(joy_lx, 3), round(joy_rx, 3), round(joy_ry, 3)

            values = {
                "trigger_left": sent_brake, "trigger_right": sent_throttle,
                "left_x": sent_left_x, "left_y": 0.0,
                "right_x": sent_right_x, "right_y": sent_right_y,
                "button_a": float(stabilizer_button),
                "mouse_dx": self.mouse_dx, "mouse_dy": self.mouse_dy,
                "pending_yaw_deg": self.pending_yaw_deg, "pending_pitch_deg": self.pending_pitch_deg,
            }
            self.mouse_dx = self.mouse_dy = 0.0
            command = f"pad tl={sent_brake:.3f} tr={sent_throttle:.3f} jlx={sent_left_x:.3f} jrx={sent_right_x:.3f} jry={sent_right_y:.3f} a={stabilizer_button}\n"
            return command, values


class QVirtualGamepadView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.state = {
            key: 0.0
            for key in [
                "trigger_left", "trigger_right", "left_x", "left_y",
                "right_x", "right_y", "button_a", "mouse_dx", "mouse_dy",
                "pending_yaw_deg", "pending_pitch_deg",
            ]
        }
        self.setMinimumSize(220, 130)

    def update_state(self, values):
        self.state.update(values)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        painter.fillRect(self.rect(), QColor(SURFACE))

        self.draw_trigger(
            painter, QRectF(12, 14, 72, 7), self.state["trigger_left"], "制动"
        )
        self.draw_trigger(
            painter, QRectF(w - 84, 14, 72, 7), self.state["trigger_right"], "油门"
        )

        radius = min(36.0, max(29.0, h * 0.25))
        self.draw_stick(
            painter, w * 0.34, h * 0.48, radius,
            self.state["left_x"], self.state["left_y"], "底盘",
        )
        self.draw_stick(
            painter, w * 0.68, h * 0.48, radius,
            self.state["right_x"], self.state["right_y"], "炮塔",
        )

        button_active = self.state["button_a"] >= 0.5
        button_center = QPointF(w - 28, h * 0.48)
        painter.setBrush(QColor(GREEN if button_active else ELEVATED))
        painter.setPen(QPen(QColor("#79EAB2" if button_active else BORDER), 1))
        painter.drawEllipse(button_center, 12, 12)
        painter.setPen(QColor(BG if button_active else MUTED))
        painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
        painter.drawText(
            QRectF(button_center.x() - 12, button_center.y() - 12, 24, 24),
            Qt.AlignCenter,
            "A",
        )

        painter.setPen(QColor(MUTED))
        painter.setFont(QFont("Cascadia Mono", 8))
        detail = (
            f"DX {self.state['mouse_dx']:+.0f}  DY {self.state['mouse_dy']:+.0f}"
            f"    YAW {self.state['pending_yaw_deg']:+.1f}°"
            f"  PITCH {self.state['pending_pitch_deg']:+.1f}°"
        )
        painter.drawText(QRectF(10, h - 24, w - 20, 16), Qt.AlignCenter, detail)

    def draw_trigger(self, painter, rect, value, label):
        value = clamp(value, 0.0, 1.0)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(ELEVATED))
        painter.drawRoundedRect(rect, 3.5, 3.5)
        if value > 0:
            fill = QRectF(rect.x(), rect.y(), rect.width() * value, rect.height())
            painter.setBrush(QColor(ACCENT))
            painter.drawRoundedRect(fill, 3.5, 3.5)
        painter.setPen(QColor(MUTED))
        painter.setFont(QFont("Microsoft YaHei UI", 8))
        painter.drawText(
            QRectF(rect.x(), rect.y() + 9, rect.width(), 14),
            Qt.AlignCenter,
            f"{label} {value:.2f}",
        )

    def draw_stick(self, painter, cx, cy, r, x, y, label):
        x = clamp(x, -1.0, 1.0)
        y = clamp(y, -1.0, 1.0)
        painter.setBrush(QBrush(QColor(BG)))
        painter.setPen(QPen(QColor(BORDER), 2))
        painter.drawEllipse(QPointF(cx, cy), r, r)

        painter.setPen(QPen(QColor(GRID), 1))
        painter.drawLine(QPointF(cx - r * 0.65, cy), QPointF(cx + r * 0.65, cy))
        painter.drawLine(QPointF(cx, cy - r * 0.65), QPointF(cx, cy + r * 0.65))

        kx = cx + x * r * 0.8
        ky = cy + y * r * 0.8

        painter.setBrush(QBrush(QColor(ACCENT)))
        painter.setPen(QPen(QColor("#8FC3FF"), 1))
        painter.drawEllipse(QPointF(kx, ky), 7, 7)

        painter.setPen(QColor(MUTED))
        painter.setFont(QFont("Microsoft YaHei UI", 8, QFont.Bold))
        painter.drawText(
            QRectF(cx - r, cy + r + 5, r * 2, 17), Qt.AlignCenter, label
        )


class QTelemetryChartView(QWidget):
    capture_requested = Signal()
    mouse_position_changed = Signal(float, float)

    SERIES = {
        "chassis_yaw_deg": ("车体", "#6B7C93"),
        "turret_yaw_deg": ("炮塔", ACCENT),
        "target_yaw_deg": ("目标", CYAN),
        "chassis_pitch_deg": ("车体", "#6B7C93"),
        "gun_pitch_deg": ("炮管", "#7A5AF8"),
        "target_pitch_deg": ("目标", CYAN),
        "yaw_voltage": ("Yaw 电压", RED),
        "servo_command_deg": ("舵机命令", GREEN),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.state = {
            key: 0.0
            for key in [
                "chassis_yaw_deg", "chassis_pitch_deg", "turret_yaw_deg",
                "turret_relative_yaw_deg", "gun_pitch_deg", "target_yaw_deg",
                "target_pitch_deg", "yaw_voltage", "servo_command_deg",
                "stabilizer_enabled", "imu_healthy", "yaw_sensor_healthy",
            ]
        }
        self.telemetry_stale = True
        self.mouse_captured = False
        self.timestamps = deque(maxlen=300)
        self.history = {
            key: deque(maxlen=300)
            for key in self.SERIES
        }
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMinimumHeight(270)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        painter.fillRect(self.rect(), QColor(SURFACE))

        painter.setPen(QColor(TEXT))
        painter.setFont(QFont("Microsoft YaHei UI", 13, QFont.Bold))
        painter.drawText(18, 29, "关键运动遥测")
        painter.setPen(QColor(MUTED))
        painter.setFont(QFont("Microsoft YaHei UI", 9))
        hint = (
            "鼠标已接管，按 Esc 急停并释放"
            if self.mouse_captured
            else "30 秒滚动窗口 · 点击曲线区接管炮塔鼠标输入"
        )
        painter.setPen(QColor(ACCENT if self.mouse_captured else MUTED))
        painter.drawText(18, 48, hint)

        status_color = (
            RED if self.telemetry_stale
            else GREEN if self.state["stabilizer_enabled"]
            else YELLOW
        )
        status_text = (
            "遥测离线" if self.telemetry_stale
            else "双稳已开启" if self.state["stabilizer_enabled"]
            else "手动模式"
        )
        self.draw_status_chip(
            painter, QRectF(w - 118, 14, 100, 28), status_text, status_color
        )

        chart_top = 62
        gap = 8
        chart_width = (w - 36 - gap) / 2
        chart_height = (h - chart_top - 14 - gap) / 2
        charts = [
            (
                QRectF(14, chart_top, chart_width, chart_height),
                "Yaw 角度",
                ["chassis_yaw_deg", "turret_yaw_deg", "target_yaw_deg"],
                "°",
            ),
            (
                QRectF(14 + chart_width + gap, chart_top, chart_width, chart_height),
                "Pitch 角度",
                ["chassis_pitch_deg", "gun_pitch_deg", "target_pitch_deg"],
                "°",
            ),
            (
                QRectF(14, chart_top + chart_height + gap, chart_width, chart_height),
                "Yaw 电压输出",
                ["yaw_voltage"],
                "V",
            ),
            (
                QRectF(
                    14 + chart_width + gap,
                    chart_top + chart_height + gap,
                    chart_width,
                    chart_height,
                ),
                "舵机命令",
                ["servo_command_deg"],
                "°",
            ),
        ]
        for rect, title, keys, unit in charts:
            self.draw_chart(painter, rect, title, keys, unit)

    def draw_chart(self, painter, rect, title, keys, unit):
        painter.setPen(QPen(QColor(BORDER), 1))
        painter.setBrush(QColor("#FAFCFE"))
        painter.drawRoundedRect(rect, 8, 8)

        painter.setPen(QColor(TEXT))
        painter.setFont(QFont("Microsoft YaHei UI", 9, QFont.Bold))
        painter.drawText(rect.adjusted(10, 6, -8, 0), Qt.AlignTop, title)

        legend_x = rect.right() - 8
        painter.setFont(QFont("Microsoft YaHei UI", 7))
        for key in reversed(keys):
            label, color = self.SERIES[key]
            text_width = painter.fontMetrics().horizontalAdvance(label) + 16
            legend_x -= text_width
            painter.setPen(QPen(QColor(color), 2))
            painter.drawLine(
                QPointF(legend_x, rect.top() + 13),
                QPointF(legend_x + 8, rect.top() + 13),
            )
            painter.setPen(QColor(MUTED))
            painter.drawText(
                QRectF(legend_x + 11, rect.top() + 5, text_width - 11, 16),
                Qt.AlignVCenter,
                label,
            )

        plot = rect.adjusted(36, 27, -10, -20)
        painter.setPen(QPen(QColor(GRID), 1))
        for index in range(5):
            y = plot.top() + plot.height() * index / 4
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
        for index in range(7):
            x = plot.left() + plot.width() * index / 6
            painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))

        values = [
            value
            for key in keys
            for value in self.history[key]
        ]
        if values:
            low = min(min(values), 0.0)
            high = max(max(values), 0.0)
        else:
            low, high = -1.0, 1.0
        if math.isclose(low, high):
            low -= 1.0
            high += 1.0
        padding = max((high - low) * 0.12, 0.25)
        low -= padding
        high += padding

        painter.setPen(QColor(MUTED))
        painter.setFont(QFont("Cascadia Mono", 7))
        painter.drawText(
            QRectF(rect.left() + 4, plot.top() - 6, 30, 14),
            Qt.AlignRight,
            f"{high:.1f}",
        )
        painter.drawText(
            QRectF(rect.left() + 4, plot.bottom() - 7, 30, 14),
            Qt.AlignRight,
            f"{low:.1f}",
        )
        painter.drawText(
            QRectF(plot.left(), plot.bottom() + 2, plot.width(), 14),
            Qt.AlignRight,
            f"{self.state[keys[-1]]:+.2f}{unit}",
        )

        if len(self.timestamps) < 2:
            painter.setPen(QColor(MUTED))
            painter.setFont(QFont("Microsoft YaHei UI", 8))
            painter.drawText(plot, Qt.AlignCenter, "等待遥测数据")
            return

        end_time = self.timestamps[-1]
        duration = 30.0
        start_time = end_time - duration
        painter.setPen(QColor(MUTED))
        painter.setFont(QFont("Cascadia Mono", 7))
        painter.drawText(
            QRectF(plot.left(), plot.bottom() + 2, 40, 14),
            Qt.AlignLeft,
            "-30s",
        )
        painter.setClipRect(plot)
        for key in keys:
            samples = self.history[key]
            if len(samples) < 2:
                continue
            points = []
            offset = len(self.timestamps) - len(samples)
            for index, value in enumerate(samples):
                timestamp = self.timestamps[index + offset]
                if timestamp < start_time:
                    continue
                x = plot.left() + (timestamp - start_time) / duration * plot.width()
                y = plot.bottom() - (value - low) / (high - low) * plot.height()
                points.append(QPointF(x, y))
            if len(points) < 2:
                continue
            pen = QPen(QColor(self.SERIES[key][1]), 1.7)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            painter.setPen(pen)
            painter.drawPolyline(QPolygonF(points))
        painter.setClipping(False)

    @staticmethod
    def draw_status_chip(painter, rect, text, color):
        fill = QColor(color)
        fill.setAlpha(38)
        painter.setBrush(fill)
        painter.setPen(QPen(QColor(color), 1))
        painter.drawRoundedRect(rect, 14, 14)
        painter.setPen(QColor(color))
        painter.setFont(QFont("Microsoft YaHei UI", 8, QFont.Bold))
        painter.drawText(rect, Qt.AlignCenter, text)

    def update_telemetry(self, values):
        self.state.update(values)
        self.timestamps.append(time.monotonic())
        for key in self.history:
            self.history[key].append(float(self.state[key]))
        self.telemetry_stale = False
        self.update()

    def set_telemetry_stale(self, stale):
        if self.telemetry_stale != stale:
            self.telemetry_stale = stale
            self.update()

    def set_mouse_captured(self, captured):
        self.mouse_captured = bool(captured)
        self.setCursor(Qt.BlankCursor if captured else Qt.ArrowCursor)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.capture_requested.emit()
            event.accept()

    def mouseMoveEvent(self, event):
        self.mouse_position_changed.emit(event.position().x(), event.position().y())
        event.accept()


class QParameterRow(QFrame):
    def __init__(self, name, value, minimum, maximum, label, description, send_callback, parent=None):
        super().__init__(parent)
        self.setObjectName("ParameterRow")
        self.setProperty("dirty", False)
        self.name = name
        self.minimum = minimum
        self.maximum = maximum
        self.send_callback = send_callback
        self.dirty = False
        self.variable = value
        self._syncing = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 11)
        layout.setSpacing(6)

        header = QHBoxLayout()
        header.setSpacing(8)

        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        self.lbl_title = QLabel(label)
        self.lbl_title.setObjectName("ParameterTitle")
        self.lbl_name = QLabel(name)
        self.lbl_name.setObjectName("Mono")
        self.lbl_name.setStyleSheet(f"color: {MUTED}; font-size: 10px;")
        title_box.addWidget(self.lbl_title)
        title_box.addWidget(self.lbl_name)
        header.addLayout(title_box)
        header.addStretch()

        self.entry = QLineEdit(self.format_value(value))
        self.entry.setValidator(QDoubleValidator(minimum, maximum, 6, self.entry))
        self.entry.setFixedWidth(92)
        self.entry.setAlignment(Qt.AlignRight)
        self.entry.textChanged.connect(self.mark_dirty)
        self.entry.returnPressed.connect(self.apply)
        header.addWidget(self.entry)

        self.btn_apply = QPushButton("应用")
        self.btn_apply.setFocusPolicy(Qt.NoFocus)
        self.btn_apply.setFixedWidth(58)
        self.btn_apply.setEnabled(False)
        self.btn_apply.clicked.connect(self.apply)
        header.addWidget(self.btn_apply)
        layout.addLayout(header)

        desc = QLabel(description)
        desc.setStyleSheet(f"color: {MUTED}; font-size: 11px;")
        desc.setWordWrap(True)
        layout.addWidget(desc)

        range_row = QHBoxLayout()
        range_row.setSpacing(8)
        min_label = QLabel(self.format_value(minimum))
        min_label.setObjectName("Mono")
        min_label.setStyleSheet(f"color: {MUTED}; font-size: 9px;")
        min_label.setFixedWidth(58)
        min_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        range_row.addWidget(min_label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 10000)
        self.slider.setValue(int((value - minimum) / (maximum - minimum) * 10000) if maximum != minimum else 0)
        self.slider.valueChanged.connect(self.on_slider_moved)
        range_row.addWidget(self.slider)

        max_label = QLabel(self.format_value(maximum))
        max_label.setObjectName("Mono")
        max_label.setStyleSheet(f"color: {MUTED}; font-size: 9px;")
        max_label.setFixedWidth(58)
        max_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        range_row.addWidget(max_label)
        layout.addLayout(range_row)

    @staticmethod
    def format_value(v): return f"{v:.2f}" if abs(v) >= 100 else f"{v:.3f}" if abs(v) >= 10 else f"{v:.5f}"

    def on_slider_moved(self, pos):
        val = self.minimum + (pos / 10000.0) * (self.maximum - self.minimum)
        self.entry.setText(self.format_value(val))

    def mark_dirty(self):
        if self._syncing:
            return
        self.dirty = True
        self.setProperty("dirty", True)
        self.btn_apply.setEnabled(True)
        self.style().unpolish(self)
        self.style().polish(self)

    def set_value(self, value):
        self.variable = clamp(value, self.minimum, self.maximum)
        self._syncing = True
        self.entry.blockSignals(True)
        self.entry.setText(self.format_value(self.variable))
        self.entry.blockSignals(False)
        self.slider.blockSignals(True)
        self.slider.setValue(int((self.variable - self.minimum) / (self.maximum - self.minimum) * 10000) if self.maximum != self.minimum else 0)
        self.slider.blockSignals(False)
        self._syncing = False
        self.dirty = False
        self.setProperty("dirty", False)
        self.btn_apply.setEnabled(False)
        self.style().unpolish(self)
        self.style().polish(self)

    def apply(self):
        try: val = clamp(float(self.entry.text()), self.minimum, self.maximum)
        except ValueError: val = self.variable
        self.set_value(val)
        self.send_callback(self.name, val)

    def get_value(self):
        try: return clamp(float(self.entry.text()), self.minimum, self.maximum)
        except ValueError: return self.variable


class QConsoleMainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.pad = PadState(mouse_sensitivity=args.mouse_sensitivity)
        self.command_queue, self.ui_queue = queue.Queue(), queue.Queue()
        self.stop_event = threading.Event()
        self.connected = False
        self.emergency_stopped = False
        self.latest_pad_command = "pad tl=0.000 tr=0.000 jlx=0.000 jrx=0.000 jry=0.000 a=0\n"
        self.pad_command_lock = threading.Lock()
        self.mouse_captured = False
        self.mouse_warp_pending = False
        self.rx_buffer, self.parameter_rows = "", {}
        self.last_telemetry_at = 0.0
        self._closing = False

        self.setWindowTitle("Chieftain MK10 Control Center")
        self.resize(1380, 900)
        self.setMinimumSize(1180, 800)
        self.setStyleSheet(QSS_THEME)

        self.build_ui()
        self.bind_inputs()

        self.timer_ui = QTimer()
        self.timer_ui.timeout.connect(self.process_ui_events)
        self.timer_ui.start(30)
        self.timer_pad = QTimer()
        self.timer_pad.timeout.connect(self.update_local_pad)
        self.timer_pad.start(20)

    def build_ui(self):
        central = QWidget(self)
        central.setObjectName("AppRoot")
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(10)

        header = QFrame(objectName="TopBar")
        header.setFixedHeight(64)
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(18, 9, 14, 9)
        h_layout.setSpacing(12)

        brand_box = QVBoxLayout()
        brand_box.setSpacing(0)
        logo = QLabel("CHIEFTAIN")
        logo.setObjectName("HeroTitle")
        logo.setStyleSheet(
            f"font-size: 17px; font-weight: 800; color: {TEXT}; letter-spacing: 2px;"
        )
        descriptor = QLabel("MK10  ·  DEVICE CONTROL CENTER")
        descriptor.setObjectName("Mono")
        descriptor.setStyleSheet(f"font-size: 9px; color: {MUTED};")
        brand_box.addWidget(logo)
        brand_box.addWidget(descriptor)
        h_layout.addLayout(brand_box)
        h_layout.addStretch()

        device_label = QLabel(self.args.name)
        device_label.setObjectName("Mono")
        device_label.setStyleSheet(f"font-size: 10px; color: {MUTED};")
        h_layout.addWidget(device_label)

        status_box = QFrame(objectName="Inset")
        status_box.setFixedHeight(36)
        status_layout = QHBoxLayout(status_box)
        status_layout.setContentsMargins(11, 0, 12, 0)
        status_layout.setSpacing(7)
        self.connection_dot = QLabel("●")
        self.connection_dot.setStyleSheet(f"color: {YELLOW}; font-size: 12px;")
        self.lbl_status = QLabel("正在扫描")
        self.lbl_status.setStyleSheet(f"font-weight: 700; color: {YELLOW};")
        status_layout.addWidget(self.connection_dot)
        status_layout.addWidget(self.lbl_status)
        h_layout.addWidget(status_box)

        btn_read_header = QPushButton("读取设备参数", objectName="Primary")
        btn_read_header.setFocusPolicy(Qt.NoFocus)
        btn_read_header.clicked.connect(lambda: self.enqueue_command("get"))
        h_layout.addWidget(btn_read_header)
        main_layout.addWidget(header)

        root_splitter = QSplitter(Qt.Vertical)
        root_splitter.setChildrenCollapsible(False)

        workspace = QWidget()
        pane_layout = QHBoxLayout()
        pane_layout.setContentsMargins(0, 0, 0, 0)
        pane_layout.setSpacing(10)
        workspace.setLayout(pane_layout)

        left_pane = QVBoxLayout()
        left_pane.setSpacing(10)

        visual_panel = QFrame(objectName="TelemetryPanel")
        visual_layout = QVBoxLayout(visual_panel)
        visual_layout.setContentsMargins(0, 0, 0, 0)
        visual_layout.setSpacing(0)
        self.telemetry_view = QTelemetryChartView()
        visual_layout.addWidget(self.telemetry_view, stretch=1)

        tel_widget = QWidget()
        tel_grid = QGridLayout(tel_widget)
        tel_grid.setContentsMargins(10, 8, 10, 10)
        tel_grid.setHorizontalSpacing(7)
        tel_grid.setVerticalSpacing(7)
        self.telemetry_labels = {}
        fields = [
            ("CHASSIS_Y", "车体 YAW"), ("CHASSIS_P", "车体 PITCH"),
            ("TURRET_ABS", "炮塔 YAW"), ("GUN_P", "炮管 PITCH"),
            ("TARGET_Y", "目标 YAW"), ("TARGET_P", "目标 PITCH"),
        ]
        for idx, (k, l) in enumerate(fields):
            tile = QFrame(objectName="TelemetryTile")
            tile_layout = QVBoxLayout(tile)
            tile_layout.setContentsMargins(10, 6, 10, 7)
            tile_layout.setSpacing(0)
            tile_layout.addWidget(QLabel(l, objectName="SubTitle"))
            val = QLabel("+0.00°", objectName="LargeValue")
            tile_layout.addWidget(val)
            tel_grid.addWidget(tile, idx // 3, idx % 3)
            self.telemetry_labels[k] = val
        visual_layout.addWidget(tel_widget)
        left_pane.addWidget(visual_panel, stretch=3)

        ctrl_card = QFrame(objectName="ControlPanel")
        ctrl_layout = QHBoxLayout(ctrl_card)
        ctrl_layout.setContentsMargins(14, 12, 14, 12)
        ctrl_layout.setSpacing(14)

        kb_layout = QVBoxLayout()
        kb_layout.setSpacing(6)
        kb_layout.addWidget(QLabel("键盘输入", objectName="SectionTitle"))
        kb_hint = QLabel("W/S 动力 · A/D 转向 · Space 双稳")
        kb_hint.setStyleSheet(f"color: {MUTED}; font-size: 10px;")
        kb_layout.addWidget(kb_hint)
        grid = QGridLayout()
        grid.setSpacing(5)
        self.key_labels = {}
        for k, r, c, cs in [("W",0,1,1), ("A",1,0,1), ("S",1,1,1), ("D",1,2,1), ("SPACE",2,0,3)]:
            lbl = QLabel(k)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setMinimumHeight(27)
            lbl.setStyleSheet(self.key_style(False))
            grid.addWidget(lbl, r, c, 1, cs)
            self.key_labels[k.lower()] = lbl
        kb_layout.addLayout(grid)
        ctrl_layout.addLayout(kb_layout)

        divider_one = QFrame()
        divider_one.setFrameShape(QFrame.VLine)
        divider_one.setStyleSheet(f"color: {BORDER};")
        ctrl_layout.addWidget(divider_one)

        gamepad_layout = QVBoxLayout()
        gamepad_layout.setSpacing(4)
        gamepad_layout.addWidget(QLabel("虚拟手柄输出", objectName="SectionTitle"))
        self.gamepad_view = QVirtualGamepadView()
        gamepad_layout.addWidget(self.gamepad_view)
        ctrl_layout.addLayout(gamepad_layout, stretch=1)

        divider_two = QFrame()
        divider_two.setFrameShape(QFrame.VLine)
        divider_two.setStyleSheet(f"color: {BORDER};")
        ctrl_layout.addWidget(divider_two)

        tuning_layout = QVBoxLayout()
        tuning_layout.setSpacing(5)
        tuning_layout.addWidget(QLabel("输入调节", objectName="SectionTitle"))
        self.add_adjustment(
            tuning_layout, "键盘最大输入", 0.1, 1.0, 1.0,
            self.pad.set_drive_scale,
        )
        self.add_adjustment(
            tuning_layout, "鼠标角度灵敏度", 0.02, 0.50,
            self.args.mouse_sensitivity, self.pad.set_mouse_sensitivity,
        )
        self.add_adjustment(
            tuning_layout, "鼠标转换时间", 0.05, 0.60, 0.18,
            self.pad.set_mouse_response_time,
        )
        tuning_layout.addStretch()
        ctrl_layout.addLayout(tuning_layout)

        left_pane.addWidget(ctrl_card)
        pane_layout.addLayout(left_pane, 6)

        right_pane = QVBoxLayout()
        right_pane.setSpacing(10)

        safe_card = QFrame(objectName="SafetyPanel")
        safe_layout = QGridLayout(safe_card)
        safe_layout.setContentsMargins(14, 12, 14, 12)
        safe_layout.setHorizontalSpacing(10)
        safe_layout.setVerticalSpacing(7)

        safety_title = QLabel("安全与使能", objectName="SectionTitle")
        safe_layout.addWidget(safety_title, 0, 0)
        self.lbl_safety = QLabel("等待设备连接", objectName="StatusStrong")
        self.lbl_safety.setStyleSheet(f"color: {MUTED}; font-weight: 700;")
        self.lbl_safety.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        safe_layout.addWidget(self.lbl_safety, 0, 1)

        safety_hint = QLabel("Esc 可立即急停；断链后 ESP32 将按固件策略超时停车。")
        safety_hint.setStyleSheet(f"color: {MUTED}; font-size: 10px;")
        safe_layout.addWidget(safety_hint, 1, 0, 1, 2)

        safe_btns = QHBoxLayout()
        btn_stop = QPushButton("紧急停车", objectName="Danger")
        btn_stop.setFocusPolicy(Qt.NoFocus)
        btn_stop.clicked.connect(self.emergency_stop)
        btn_arm = QPushButton("解除急停 / 使能", objectName="Primary")
        btn_arm.setFocusPolicy(Qt.NoFocus)
        btn_arm.clicked.connect(self.release_emergency_stop)
        safe_btns.addWidget(btn_stop)
        safe_btns.addWidget(btn_arm)
        safe_layout.addLayout(safe_btns, 2, 0, 1, 2)
        right_pane.addWidget(safe_card)

        param_card = QFrame(objectName="ParameterPanel")
        p_layout = QVBoxLayout(param_card)
        p_layout.setContentsMargins(10, 10, 10, 10)
        p_layout.setSpacing(8)

        p_header = QHBoxLayout()
        parameter_titles = QVBoxLayout()
        parameter_titles.setSpacing(0)
        parameter_titles.addWidget(QLabel("运行参数", objectName="SectionTitle"))
        parameter_hint = QLabel("实时读取、修改并持久化 ESP32 控制参数")
        parameter_hint.setStyleSheet(f"color: {MUTED}; font-size: 10px;")
        parameter_titles.addWidget(parameter_hint)
        p_header.addLayout(parameter_titles)
        p_header.addStretch()
        self.parameter_status = QLabel("尚未读取")
        self.parameter_status.setStyleSheet(f"color: {MUTED}; font-weight: 700;")
        p_header.addWidget(self.parameter_status)
        p_layout.addLayout(p_header)

        primary_tools = QHBoxLayout()
        primary_tools.setSpacing(6)
        btn_get = QPushButton("从设备读取", objectName="Primary")
        btn_get.clicked.connect(lambda: self.enqueue_command("get"))
        btn_apply_all = QPushButton("推送修改", objectName="Secondary")
        btn_apply_all.clicked.connect(self.apply_all_parameters)
        btn_save = QPushButton("写入 Flash", objectName="Secondary")
        btn_save.clicked.connect(lambda: self.enqueue_command("save"))
        for button in (btn_get, btn_apply_all, btn_save):
            button.setFocusPolicy(Qt.NoFocus)
            primary_tools.addWidget(button)
        p_layout.addLayout(primary_tools)

        secondary_tools = QHBoxLayout()
        secondary_tools.setSpacing(6)
        btn_load = QPushButton("加载保存值", objectName="Quiet")
        btn_load.clicked.connect(self.load_saved_parameters)
        btn_defaults = QPushButton("恢复默认", objectName="Quiet")
        btn_defaults.clicked.connect(self.restore_defaults)
        btn_export = QPushButton("导出参数", objectName="Quiet")
        btn_export.clicked.connect(self.export_parameters)
        for button in (btn_load, btn_defaults, btn_export):
            button.setFocusPolicy(Qt.NoFocus)
            secondary_tools.addWidget(button)
        p_layout.addLayout(secondary_tools)

        self.tab_widget = QTabWidget()
        self.tab_widget.setDocumentMode(True)
        self.tab_widget.tabBar().setExpanding(True)
        self.tab_widget.tabBar().setUsesScrollButtons(False)
        self.tab_widget.tabBar().setElideMode(Qt.ElideRight)
        self.tab_pages = {}
        self.tab_empty_labels = {}
        for group in GROUP_ORDER:
            scroll = QScrollArea(widgetResizable=True)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            content = QWidget()
            layout = QVBoxLayout(content)
            layout.setContentsMargins(0, 0, 0, 8)
            layout.setSpacing(0)
            empty_label = QLabel("连接设备后点击“从设备读取”")
            empty_label.setAlignment(Qt.AlignCenter)
            empty_label.setStyleSheet(f"color: {MUTED}; padding: 40px;")
            layout.addWidget(empty_label)
            layout.addStretch()
            scroll.setWidget(content)
            self.tab_pages[group] = layout
            self.tab_empty_labels[group] = empty_label
            self.tab_widget.addTab(scroll, group)
        p_layout.addWidget(self.tab_widget)
        right_pane.addWidget(param_card, stretch=1)

        pane_layout.addLayout(right_pane, 4)
        root_splitter.addWidget(workspace)

        console_panel = QFrame(objectName="ConsolePanel")
        console_layout = QVBoxLayout(console_panel)
        console_layout.setContentsMargins(12, 9, 12, 12)
        console_layout.setSpacing(7)

        console_header = QHBoxLayout()
        console_header.setSpacing(8)
        console_accent = QFrame(objectName="ConsoleAccent")
        console_accent.setFixedSize(4, 22)
        console_header.addWidget(console_accent)
        console_header.addWidget(QLabel("通信终端", objectName="SectionTitle"))
        console_header.addStretch()
        btn_clear_log = QPushButton("清空日志", objectName="Quiet")
        btn_clear_log.setFocusPolicy(Qt.NoFocus)
        btn_clear_log.clicked.connect(self.clear_log)
        console_header.addWidget(btn_clear_log)
        console_layout.addLayout(console_header)

        self.log = QTextEdit()
        self.log.setObjectName("ConsoleLog")
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(74)
        self.log.setPlaceholderText("BLE 扫描、设备响应和参数操作会显示在这里。")
        console_layout.addWidget(self.log, stretch=1)

        command_row = QHBoxLayout()
        command_label = QLabel("命令")
        command_label.setStyleSheet(f"color: {MUTED};")
        command_row.addWidget(command_label)
        self.command_entry = QLineEdit()
        self.command_entry.setObjectName("ConsoleCommand")
        self.command_entry.setPlaceholderText("例如：get、save、set REAL_TURRET_VEL 22.5")
        self.command_entry.returnPressed.connect(self.submit_manual_command)
        command_row.addWidget(self.command_entry, stretch=1)
        btn_send = QPushButton("发送", objectName="Primary")
        btn_send.setFocusPolicy(Qt.NoFocus)
        btn_send.clicked.connect(self.submit_manual_command)
        command_row.addWidget(btn_send)
        console_layout.addLayout(command_row)

        root_splitter.addWidget(console_panel)
        root_splitter.setStretchFactor(0, 5)
        root_splitter.setStretchFactor(1, 1)
        root_splitter.setSizes([680, 145])
        main_layout.addWidget(root_splitter, stretch=1)

    @staticmethod
    def key_style(active):
        if active:
            return (
                f"background: {ACCENT}; border: 1px solid {ACCENT_HOVER};"
                f" border-radius: 6px; padding: 4px; font-weight: 700;"
                f" font-family: 'Cascadia Mono', 'Consolas'; color: white;"
            )
        return (
            f"background: {SURFACE}; border: 1px solid {BORDER};"
            f" border-radius: 6px; padding: 4px; font-weight: 700;"
            f" font-family: 'Cascadia Mono', 'Consolas'; color: {MUTED};"
        )

    def add_adjustment(self, parent_layout, label, minimum, maximum, value, callback):
        row = QVBoxLayout()
        row.setSpacing(1)
        label_row = QHBoxLayout()
        name_label = QLabel(label)
        name_label.setStyleSheet(f"color: {MUTED}; font-size: 10px;")
        value_label = QLabel(f"{value:.3f}")
        value_label.setObjectName("Mono")
        value_label.setStyleSheet(f"color: {TEXT}; font-size: 10px;")
        label_row.addWidget(name_label)
        label_row.addStretch()
        label_row.addWidget(value_label)
        row.addLayout(label_row)

        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 1000)
        if maximum != minimum:
            slider.setValue(round((value - minimum) / (maximum - minimum) * 1000))

        def on_change(position):
            current = minimum + position / 1000.0 * (maximum - minimum)
            value_label.setText(f"{current:.3f}")
            callback(current)

        slider.valueChanged.connect(on_change)
        row.addWidget(slider)
        parent_layout.addLayout(row)

    def bind_inputs(self):
        self.telemetry_view.capture_requested.connect(self.capture_mouse)
        self.telemetry_view.mouse_position_changed.connect(self.on_mouse_motion)
        QApplication.instance().installEventFilter(self)

    def eventFilter(self, watched, event):
        if event.type() == QEvent.ApplicationDeactivate:
            self.pad.clear()
            for label in self.key_labels.values():
                label.setStyleSheet(self.key_style(False))
            self.release_mouse()
            return False

        if event.type() not in (QEvent.KeyPress, QEvent.KeyRelease):
            return super().eventFilter(watched, event)
        if event.isAutoRepeat():
            return True

        if event.key() == Qt.Key_Escape and event.type() == QEvent.KeyPress:
            self.emergency_stop()
            return True

        focus_w = QApplication.focusWidget()
        if isinstance(focus_w, (QLineEdit, QTextEdit)):
            return super().eventFilter(watched, event)

        key_map = {Qt.Key_W: "w", Qt.Key_A: "a", Qt.Key_S: "s", Qt.Key_D: "d", Qt.Key_Space: "space"}
        key = key_map.get(event.key())
        if key:
            if event.type() == QEvent.KeyPress:
                is_new = self.pad.key_down(key)
                self.key_labels[key].setStyleSheet(self.key_style(True))
                if key == "space" and is_new:
                    self.append_log("Space 已映射为手柄 A 键，等待设备遥测确认。\n", ACCENT)
            else:
                self.pad.key_up(key)
                self.key_labels[key].setStyleSheet(self.key_style(False))
            return True
        return super().eventFilter(watched, event)

    def capture_mouse(self):
        if self.emergency_stopped:
            self.append_log("急停已锁存，请先解除急停。\n", RED)
            return
        self.mouse_captured = True
        self.telemetry_view.set_mouse_captured(True)
        self.telemetry_view.setMouseTracking(True)
        self.telemetry_view.setFocus(Qt.MouseFocusReason)
        self.telemetry_view.grabMouse()

    def release_mouse(self):
        if self.mouse_captured:
            self.telemetry_view.releaseMouse()
            self.mouse_captured = False
            self.telemetry_view.set_mouse_captured(False)
            self.mouse_warp_pending = False

    def on_mouse_motion(self, x, y):
        if not self.mouse_captured: return
        cx, cy = self.telemetry_view.width() // 2, self.telemetry_view.height() // 2
        dx, dy = x - cx, y - cy

        if self.mouse_warp_pending:
            if abs(dx) <= 2 and abs(dy) <= 2:
                self.mouse_warp_pending = False
                return

        if dx or dy:
            self.pad.add_mouse_delta(dx, dy)
            self.mouse_warp_pending = True
            QCursor.setPos(self.telemetry_view.mapToGlobal(QPoint(cx, cy)))

    def emergency_stop(self):
        self.pad.clear()
        self.release_mouse()
        for label in self.key_labels.values():
            label.setStyleSheet(self.key_style(False))

        self.emergency_stopped = True
        self.lbl_safety.setText("急停已锁存")
        self.lbl_safety.setStyleSheet(f"color: {RED}; font-weight: bold; font-size: 13px;")
        self.enqueue_command("stop")

    def release_emergency_stop(self):
        self.pad.clear()
        self.emergency_stopped = False
        self.lbl_safety.setText("系统已使能")
        self.lbl_safety.setStyleSheet(f"color: {GREEN}; font-weight: bold; font-size: 13px;")
        self.enqueue_command("arm")

    def enqueue_command(self, command, show_in_log=True):
        command = command.strip()
        if not command:
            return
        self.command_queue.put(command + "\n")
        if show_in_log:
            self.append_log(f"> {command}\n", ACCENT)

    def submit_manual_command(self):
        command = self.command_entry.text()
        self.command_entry.clear()
        self.enqueue_command(command)

    def append_log(self, text, color=TEXT):
        escaped = html.escape(text).replace("\n", "<br>")
        self.log.moveCursor(QTextCursor.End)
        self.log.insertHtml(
            f'<span style="color:{color}; font-family:Consolas;">{escaped}</span>'
        )
        self.log.moveCursor(QTextCursor.End)

    def clear_log(self):
        self.log.clear()

    def process_ui_events(self):
        while True:
            try: event_type, payload = self.ui_queue.get_nowait()
            except queue.Empty: break

            if event_type == "status":
                self.set_connection_status(str(payload))
            elif event_type == "rx":
                self.consume_rx_text(str(payload))
            elif event_type == "log":
                self.append_log(str(payload), MUTED)

        telemetry_stale = (self.last_telemetry_at == 0.0 or time.monotonic() - self.last_telemetry_at > 0.5)
        self.telemetry_view.set_telemetry_stale(telemetry_stale)

    def set_connection_status(self, status):
        labels = {
            "扫描中": "正在扫描",
            "已连接": "设备在线",
            "连接断开": "连接断开",
            "连接失败": "连接失败",
        }
        color = GREEN if status == "已连接" else YELLOW if status == "扫描中" else RED
        self.lbl_status.setText(labels.get(status, status))
        self.lbl_status.setStyleSheet(f"color: {color}; font-weight: 700;")
        self.connection_dot.setStyleSheet(f"color: {color}; font-size: 12px;")

        if status == "已连接":
            self.connected = True
            if not self.emergency_stopped:
                self.lbl_safety.setText("输入链路正常")
                self.lbl_safety.setStyleSheet(f"color: {GREEN}; font-weight: 700;")
            return

        self.connected = False
        self.last_telemetry_at = 0.0
        self.rx_buffer = ""
        if not self.emergency_stopped:
            if status == "扫描中":
                self.lbl_safety.setText("等待设备连接")
                self.lbl_safety.setStyleSheet(f"color: {YELLOW}; font-weight: 700;")
            else:
                self.lbl_safety.setText("链路中断")
                self.lbl_safety.setStyleSheet(f"color: {RED}; font-weight: 700;")

    def update_local_pad(self):
        pad_command, values = self.pad.snapshot()
        with self.pad_command_lock: self.latest_pad_command = pad_command
        self.gamepad_view.update_state(values)

    def get_latest_pad_command(self) -> str:
        with self.pad_command_lock: return self.latest_pad_command

    def consume_rx_text(self, text):
        self.rx_buffer += text
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            self.process_rx_line(line.strip())

    def process_rx_line(self, line):
        if not line:
            return
        if line == "PARAMS":
            self.parameter_status.setText("正在读取...")
            self.parameter_status.setStyleSheet(f"color: {YELLOW}; font-weight: 700;")
            return
        if line == "END_PARAMS":
            self.parameter_status.setText(f"已读取 {len(self.parameter_rows)} 项")
            self.parameter_status.setStyleSheet(f"color: {GREEN}; font-weight: 700;")
            self.append_log("参数列表读取完成。\n", GREEN)
            return
        if line.startswith("STATE ESTOP="):
            self.emergency_stopped = line.endswith("1")
            if self.emergency_stopped:
                self.lbl_safety.setText("急停已锁存")
                self.lbl_safety.setStyleSheet(f"color: {RED}; font-weight: 700;")
            else:
                self.lbl_safety.setText("输入链路正常")
                self.lbl_safety.setStyleSheet(f"color: {GREEN}; font-weight: 700;")
            return
        if line.startswith("TEL "):
            self.parse_telemetry(line)
            return

        match = PARAM_PATTERN.match(line)
        if match:
            self.update_parameter_row(match.group(1), float(match.group(2)), float(match.group(3)), float(match.group(4)))
            return

        set_match = SET_PATTERN.match(line)
        if set_match:
            name, val = set_match.group(1), float(set_match.group(2))
            if name in self.parameter_rows: self.parameter_rows[name].set_value(val)
            if name == "REAL_TURRET_VEL": self.pad.set_turret_rate(val)

        if "emergency_stop_latched" in line:
            self.emergency_stopped = True
            self.lbl_safety.setText("急停已锁存")
            self.lbl_safety.setStyleSheet(f"color: {RED}; font-weight: 700;")
        elif "emergency_stop_released" in line:
            self.emergency_stopped = False
            self.lbl_safety.setText("系统已使能")
            self.lbl_safety.setStyleSheet(f"color: {GREEN}; font-weight: 700;")

        color = RED if line.startswith("ERR") else GREEN if line.startswith("OK") else MUTED
        self.append_log(line + "\n", color)

    def parse_telemetry(self, line):
        telemetry = {}
        for token in line.split()[1:]:
            if "=" not in token: continue
            k, v = token.split("=", 1)
            try: telemetry[k] = float(v)
            except ValueError: return

        required = {"cy", "cp", "ty", "tr", "gp", "yt", "pt", "yv", "sv", "st", "ih", "yh"}
        if not required.issubset(telemetry): return
        self.last_telemetry_at = time.monotonic()

        data = {
            "chassis_yaw_deg": telemetry["cy"], "chassis_pitch_deg": telemetry["cp"],
            "turret_yaw_deg": telemetry["ty"], "turret_relative_yaw_deg": telemetry["tr"],
            "gun_pitch_deg": telemetry["gp"], "target_yaw_deg": telemetry["yt"], "target_pitch_deg": telemetry["pt"],
            "yaw_voltage": telemetry["yv"], "servo_command_deg": telemetry["sv"],
            "stabilizer_enabled": telemetry["st"], "imu_healthy": telemetry["ih"], "yaw_sensor_healthy": telemetry["yh"],
        }
        self.telemetry_view.update_telemetry(data)
        self.update_telemetry_ui(data)

    def update_telemetry_ui(self, data):
        self.telemetry_labels["CHASSIS_Y"].setText(f"{data['chassis_yaw_deg']:+.2f}°")
        self.telemetry_labels["CHASSIS_P"].setText(f"{data['chassis_pitch_deg']:+.2f}°")
        self.telemetry_labels["TURRET_ABS"].setText(f"{data['turret_yaw_deg']:+.2f}°")
        self.telemetry_labels["GUN_P"].setText(f"{data['gun_pitch_deg']:+.2f}°")
        self.telemetry_labels["TARGET_Y"].setText(f"{data['target_yaw_deg']:+.2f}°")
        self.telemetry_labels["TARGET_P"].setText(f"{data['target_pitch_deg']:+.2f}°")

    def update_parameter_row(self, name, value, lo, hi):
        if name in self.parameter_rows:
            self.parameter_rows[name].set_value(value)
            if name == "REAL_TURRET_VEL": self.pad.set_turret_rate(value)
            return

        group, label, desc = PARAM_META.get(name, ("其他", name, "ESP32 运行时可调参数"))
        row = QParameterRow(name, value, lo, hi, label, desc, self.send_parameter)
        self.parameter_rows[name] = row
        self.tab_empty_labels[group].hide()
        self.tab_pages[group].insertWidget(self.tab_pages[group].count() - 1, row)
        if name == "REAL_TURRET_VEL": self.pad.set_turret_rate(value)

    def send_parameter(self, name, val):
        self.parameter_status.setText("正在更新...")
        self.parameter_status.setStyleSheet(f"color: {YELLOW}; font-weight: 700;")
        self.enqueue_command(f"set {name} {val:.6f}")

    def apply_all_parameters(self):
        dirty = [row for row in self.parameter_rows.values() if row.dirty]
        if not dirty:
            self.append_log("没有待推送的参数改动。\n", MUTED)
            return
        for row in dirty:
            row.apply()
        self.parameter_status.setText(f"已推送 {len(dirty)} 项")
        self.parameter_status.setStyleSheet(f"color: {YELLOW}; font-weight: 700;")

    def load_saved_parameters(self):
        self.enqueue_command("load")
        self.enqueue_command("get", show_in_log=False)

    def restore_defaults(self):
        self.enqueue_command("defaults")
        self.enqueue_command("get", show_in_log=False)

    def export_parameters(self):
        if not self.parameter_rows:
            self.append_log("当前没有可导出的参数，请先读取设备参数。\n", YELLOW)
            return

        exported_at = datetime.now().astimezone()
        timestamp = exported_at.strftime("%Y%m%d_%H%M%S")
        export_dir = Path(__file__).resolve().parent / "parameter_exports"
        export_dir.mkdir(parents=True, exist_ok=True)
        json_path = export_dir / f"chieftain_parameters_{timestamp}.json"
        text_path = export_dir / f"chieftain_parameters_{timestamp}.txt"

        parameters = {}
        for name in sorted(self.parameter_rows):
            row = self.parameter_rows[name]
            group, label, description = PARAM_META.get(
                name, ("其他", name, "ESP32 运行时可调参数")
            )
            parameters[name] = {
                "value": row.get_value(),
                "minimum": row.minimum,
                "maximum": row.maximum,
                "group": group,
                "label": label,
                "description": description,
                "pending_unsent_edit": row.dirty,
            }

        document = {
            "format_version": 1,
            "exported_at": exported_at.isoformat(timespec="seconds"),
            "ble_device": self.args.name,
            "esp32_connected": self.connected,
            "parameter_count": len(parameters),
            "note": "pending_unsent_edit=true 表示该界面值尚未发送给 ESP32",
            "parameters": parameters,
        }
        json_path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        text_lines = [
            "Chieftain MK10 参数导出",
            f"导出时间: {document['exported_at']}",
            f"BLE 设备: {self.args.name}",
            f"导出时已连接: {'是' if self.connected else '否'}",
            f"参数数量: {len(parameters)}",
            "",
            "下面的命令可在调试控制台中逐行发送：",
        ]
        for name, item in parameters.items():
            dirty_mark = "  # 注意：界面中尚未推送" if item["pending_unsent_edit"] else ""
            text_lines.append(f"set {name} {item['value']:.6f}{dirty_mark}")

        text_lines.extend(["", "参数清单："])
        for name, item in parameters.items():
            text_lines.append(
                f"{name}={item['value']:.6f} "
                f"[{item['minimum']:.6f}, {item['maximum']:.6f}] "
                f"{item['group']} / {item['label']}"
            )
        text_path.write_text("\n".join(text_lines) + "\n", encoding="utf-8")

        dirty_count = sum(
            1 for item in parameters.values() if item["pending_unsent_edit"]
        )
        self.parameter_status.setText(f"已导出 {len(parameters)} 项")
        self.parameter_status.setStyleSheet(f"color: {GREEN}; font-weight: 700;")
        self.append_log(f"参数已导出：{json_path}\n", GREEN)
        self.append_log(f"命令清单：{text_path}\n", GREEN)
        if dirty_count:
            self.append_log(f"其中 {dirty_count} 项尚未推送到设备。\n", YELLOW)

    async def find_device(self):
        self.ui_queue.put(("status", "扫描中"))
        self.ui_queue.put(("log", f"正在扫描 BLE 设备：{self.args.name}\n"))

        def match(device, advertisement_data):
            names = {device.name, advertisement_data.local_name}
            service_uuids = {
                uuid.lower() for uuid in (advertisement_data.service_uuids or [])
            }
            return (
                self.args.name in names
                or SERVICE_UUID.lower() in service_uuids
            )

        device = await BleakScanner.find_device_by_filter(match, timeout=12.0)
        if device is None:
            raise RuntimeError("未找到 Chieftain MK10 调试 BLE 设备")
        return device

    async def ble_main(self):
        while not self.stop_event.is_set():
            try:
                device = await self.find_device()
                self.ui_queue.put(("log", f"正在连接：{device.name} {device.address}\n"))
                async with BleakClient(device) as client:
                    self.ui_queue.put(("status", "已连接"))
                    self.ui_queue.put(("log", "BLE 已连接，开始以 50Hz 发送控制输入。\n"))

                    def on_notify(_, data):
                        self.ui_queue.put(("rx", data.decode(errors="replace")))

                    await client.start_notify(TX_UUID, on_notify)
                    await client.write_gatt_char(RX_UUID, b"get\n", response=False)

                    while not self.stop_event.is_set() and client.is_connected:
                        await client.write_gatt_char(RX_UUID, self.get_latest_pad_command().encode(), response=False)
                        while True:
                            try: cmd = self.command_queue.get_nowait()
                            except queue.Empty: break
                            await client.write_gatt_char(RX_UUID, cmd.encode(), response=False)
                        await asyncio.sleep(0.02)
                self.ui_queue.put(("status", "连接断开"))
            except Exception as exc:
                self.ui_queue.put(("status", "连接失败"))
                self.ui_queue.put(("log", f"BLE 错误：{exc}\n2 秒后重试...\n"))
                await asyncio.sleep(2.0)

    def closeEvent(self, event):
        if self._closing:
            event.accept()
            return
        self._closing = True
        event.ignore()
        self.emergency_stop()
        QTimer.singleShot(150, self.finish_close)

    def finish_close(self):
        self.stop_event.set()
        self.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="ChieftainMK10-Debug")
    parser.add_argument("--mouse-sensitivity", type=float, default=0.12)
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = QConsoleMainWindow(args)
    gui.showNormal()
    threading.Thread(target=lambda: asyncio.run(gui.ble_main()), daemon=True).start()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
