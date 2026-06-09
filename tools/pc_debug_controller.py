import argparse
import asyncio
import json
import math
import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import ttk

from bleak import BleakClient, BleakScanner


SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BG = "#0b1017"
PANEL = "#131b25"
PANEL_ALT = "#192432"
TEXT = "#e8eef7"
MUTED = "#8fa3b8"
ACCENT = "#42a5f5"
GREEN = "#39d98a"
YELLOW = "#f6c85f"
RED = "#ff5d6c"
GRID = "#314154"

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

    "TRACK_FF_KS": ("履带速度环", "静摩擦前馈 Ks", "履带开始运动时克服静摩擦的 PWM"),
    "TRACK_FF_KV": ("履带速度环", "速度前馈 Kv", "目标速度对应的 PWM 前馈"),
    "TRACK_FF_KA": ("履带速度环", "加速度前馈 Ka", "目标加速度对应的 PWM 前馈"),
    "TRACK_FF_KSLOPE": ("履带速度环", "坡度前馈", "坡道保持所需的 PWM 前馈"),
    "TRACK_PI_KP": ("履带速度环", "速度环 Kp", "编码器速度误差的比例修正"),
    "TRACK_PI_KI": ("履带速度环", "速度环 Ki", "编码器速度误差的积分修正"),
}

PARAM_PATTERN = re.compile(
    r"^([A-Z0-9_]+)=([-+0-9.eE]+)\s+\[([-+0-9.eE]+),([-+0-9.eE]+)\]$"
)
SET_PATTERN = re.compile(r"^OK\s+([A-Z0-9_]+)=([-+0-9.eE]+)")


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def is_text_input(widget) -> bool:
    return isinstance(widget, (tk.Entry, tk.Text, ttk.Entry, ttk.Spinbox))


@dataclass
class PadState:
    mouse_sensitivity: float = 0.12
    mouse_response_time: float = 0.18
    drive_scale: float = 1.0
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
            self.pending_yaw_deg = clamp(
                self.pending_yaw_deg + dx * self.mouse_sensitivity,
                -90.0,
                90.0,
            )
            self.pending_pitch_deg = clamp(
                self.pending_pitch_deg - dy * self.mouse_sensitivity,
                -45.0,
                45.0,
            )

    def set_mouse_sensitivity(self, value: float) -> None:
        with self.lock:
            self.mouse_sensitivity = value

    def set_drive_scale(self, value: float) -> None:
        with self.lock:
            self.drive_scale = value

    def set_mouse_response_time(self, value: float) -> None:
        with self.lock:
            self.mouse_response_time = max(value, 0.02)

    def set_turret_rate(self, value: float) -> None:
        with self.lock:
            self.turret_rate_deg_s = max(value, 1.0)

    @staticmethod
    def joystick_from_effective(value: float) -> float:
        value = clamp(value, -1.0, 1.0)
        if abs(value) < 1e-4:
            return 0.0
        return math.copysign(0.15 + 0.85 * abs(value), value)

    @staticmethod
    def consume_pending(pending: float, rate_deg_s: float, dt: float) -> float:
        consumed = rate_deg_s * dt
        if abs(consumed) >= abs(pending):
            return 0.0
        return pending - consumed

    def snapshot(self) -> tuple[str, dict[str, float]]:
        with self.lock:
            now = time.monotonic()
            dt = clamp(now - self.last_update, 0.001, 0.05)
            self.last_update = now
            keys = set(self.keys)
            drive_scale = self.drive_scale

            throttle = drive_scale if "w" in keys else 0.0
            brake = drive_scale if "s" in keys else 0.0
            left = -drive_scale if "a" in keys else 0.0
            right = drive_scale if "d" in keys else 0.0
            joy_lx = clamp(left + right, -1.0, 1.0)
            stabilizer_button = 1 if "space" in keys else 0

            yaw_rate_cmd = clamp(
                self.pending_yaw_deg / self.mouse_response_time,
                -self.turret_rate_deg_s,
                self.turret_rate_deg_s,
            )
            pitch_rate_cmd = clamp(
                self.pending_pitch_deg / self.mouse_response_time,
                -self.turret_rate_deg_s,
                self.turret_rate_deg_s,
            )
            joy_rx = self.joystick_from_effective(
                yaw_rate_cmd / self.turret_rate_deg_s
            )
            joy_ry = self.joystick_from_effective(
                -pitch_rate_cmd / self.turret_rate_deg_s
            )

            self.pending_yaw_deg = self.consume_pending(
                self.pending_yaw_deg,
                yaw_rate_cmd,
                dt,
            )
            self.pending_pitch_deg = self.consume_pending(
                self.pending_pitch_deg,
                pitch_rate_cmd,
                dt,
            )
            sent_brake = round(brake, 3)
            sent_throttle = round(throttle, 3)
            sent_left_x = round(joy_lx, 3)
            sent_right_x = round(joy_rx, 3)
            sent_right_y = round(joy_ry, 3)

            values = {
                "trigger_left": sent_brake,
                "trigger_right": sent_throttle,
                "left_x": sent_left_x,
                "left_y": 0.0,
                "right_x": sent_right_x,
                "right_y": sent_right_y,
                "button_a": float(stabilizer_button),
                "mouse_dx": self.mouse_dx,
                "mouse_dy": self.mouse_dy,
                "pending_yaw_deg": self.pending_yaw_deg,
                "pending_pitch_deg": self.pending_pitch_deg,
            }
            self.mouse_dx = 0.0
            self.mouse_dy = 0.0
            command = (
                f"pad tl={sent_brake:.3f} tr={sent_throttle:.3f} "
                f"jlx={sent_left_x:.3f} jrx={sent_right_x:.3f} jry={sent_right_y:.3f} "
                f"a={stabilizer_button}\n"
            )
            return command, values


class VirtualGamepadView(tk.Canvas):
    def __init__(self, master, **kwargs):
        super().__init__(
            master,
            bg=PANEL,
            highlightthickness=1,
            highlightbackground=GRID,
            **kwargs,
        )
        self.state = {
            "trigger_left": 0.0,
            "trigger_right": 0.0,
            "left_x": 0.0,
            "left_y": 0.0,
            "right_x": 0.0,
            "right_y": 0.0,
            "button_a": 0.0,
            "mouse_dx": 0.0,
            "mouse_dy": 0.0,
            "pending_yaw_deg": 0.0,
            "pending_pitch_deg": 0.0,
        }
        self.bind("<Configure>", lambda _: self.render())

    def update_state(self, values):
        self.state.update(values)
        self.render()

    def draw_trigger(self, x0, y0, width, height, value, label):
        value = clamp(value, 0.0, 1.0)
        self.create_rectangle(x0, y0, x0 + width, y0 + height, fill=BG, outline=GRID)
        fill_height = height * value
        if fill_height > 0.5:
            self.create_rectangle(
                x0 + 2,
                y0 + height - fill_height,
                x0 + width - 2,
                y0 + height - 2,
                fill=ACCENT,
                outline="",
            )
        self.create_text(
            x0 + width / 2,
            y0 - 9,
            text=f"{label} {value:.2f}",
            fill=TEXT,
            font=("Consolas", 9),
        )

    def draw_stick(self, cx, cy, radius, x_value, y_value, label):
        x_value = clamp(x_value, -1.0, 1.0)
        y_value = clamp(y_value, -1.0, 1.0)
        self.create_oval(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            fill=BG,
            outline=GRID,
            width=2,
        )
        self.create_line(cx - radius, cy, cx + radius, cy, fill="#263849")
        self.create_line(cx, cy - radius, cx, cy + radius, fill="#263849")
        knob_x = cx + x_value * radius * 0.72
        knob_y = cy + y_value * radius * 0.72
        self.create_oval(
            knob_x - 10,
            knob_y - 10,
            knob_x + 10,
            knob_y + 10,
            fill=ACCENT,
            outline="#b9ddff",
        )
        self.create_text(
            cx,
            cy + radius + 15,
            text=f"{label}  X {x_value:+.2f}  Y {y_value:+.2f}",
            fill=MUTED,
            font=("Consolas", 8),
        )

    def render(self):
        self.delete("all")
        width = max(self.winfo_width(), 10)
        height = max(self.winfo_height(), 10)

        self.create_text(
            12,
            10,
            anchor="nw",
            text="键鼠 → 虚拟手柄（实际发送值）",
            fill=TEXT,
            font=("Microsoft YaHei UI", 11, "bold"),
        )
        self.draw_trigger(28, 54, 28, 72, self.state["trigger_left"], "LT")
        self.draw_trigger(
            width - 56,
            54,
            28,
            72,
            self.state["trigger_right"],
            "RT",
        )

        stick_radius = min(48, max(34, width * 0.11))
        self.draw_stick(
            width * 0.31,
            100,
            stick_radius,
            self.state["left_x"],
            self.state["left_y"],
            "左摇杆",
        )
        self.draw_stick(
            width * 0.69,
            100,
            stick_radius,
            self.state["right_x"],
            self.state["right_y"],
            "右摇杆",
        )

        button_x = width * 0.84
        button_y = 36
        button_active = self.state["button_a"] >= 0.5
        self.create_oval(
            button_x - 13,
            button_y - 13,
            button_x + 13,
            button_y + 13,
            fill=GREEN if button_active else PANEL_ALT,
            outline="#a9ffc9" if button_active else GRID,
            width=2,
        )
        self.create_text(
            button_x,
            button_y,
            text="A",
            fill="white" if button_active else MUTED,
            font=("Consolas", 11, "bold"),
        )

        detail = (
            f"鼠标 ΔX {self.state['mouse_dx']:+.0f}  ΔY {self.state['mouse_dy']:+.0f}    "
            f"待转换 Yaw {self.state['pending_yaw_deg']:+.2f}°  "
            f"Pitch {self.state['pending_pitch_deg']:+.2f}°"
        )
        self.create_text(
            width / 2,
            height - 14,
            text=detail,
            fill=MUTED,
            font=("Consolas", 8),
        )


class ThirdPersonTankView(tk.Canvas):
    BOX_FACES = [
        (0, 1, 2, 3),
        (4, 7, 6, 5),
        (0, 4, 5, 1),
        (1, 5, 6, 2),
        (2, 6, 7, 3),
        (4, 0, 3, 7),
    ]

    def __init__(self, master, **kwargs):
        super().__init__(
            master,
            bg="#07111c",
            highlightthickness=1,
            highlightbackground=GRID,
            cursor="crosshair",
            **kwargs,
        )
        self.state = {
            "chassis_yaw_deg": 0.0,
            "chassis_pitch_deg": 0.0,
            "turret_yaw_deg": 0.0,
            "turret_relative_yaw_deg": 0.0,
            "gun_pitch_deg": 0.0,
            "target_yaw_deg": 0.0,
            "target_pitch_deg": 0.0,
            "yaw_voltage": 0.0,
            "servo_command_deg": 90.0,
            "stabilizer_enabled": 0.0,
            "imu_healthy": 0.0,
            "yaw_sensor_healthy": 0.0,
        }
        self.telemetry_stale = True
        self.mouse_captured = False
        self.bind("<Configure>", lambda _: self.render())

    @staticmethod
    def add(a, b):
        return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

    @staticmethod
    def sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    @staticmethod
    def mul(a, scalar):
        return (a[0] * scalar, a[1] * scalar, a[2] * scalar)

    @staticmethod
    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def cross(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @classmethod
    def normalize(cls, vector):
        length = math.sqrt(cls.dot(vector, vector))
        if length < 1e-6:
            return (0.0, 0.0, 1.0)
        return cls.mul(vector, 1.0 / length)

    @staticmethod
    def orientation_basis(yaw_deg, pitch_deg=0.0):
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        right = (math.cos(yaw), 0.0, -math.sin(yaw))
        forward = (
            math.sin(yaw) * math.cos(pitch),
            math.sin(pitch),
            math.cos(yaw) * math.cos(pitch),
        )
        up = ThirdPersonTankView.cross(forward, right)
        return right, up, forward

    @classmethod
    def local_to_world(cls, center, local, yaw_deg, pitch_deg=0.0):
        right, up, forward = cls.orientation_basis(yaw_deg, pitch_deg)
        return (
            center[0] + right[0] * local[0] + up[0] * local[1] + forward[0] * local[2],
            center[1] + right[1] * local[0] + up[1] * local[1] + forward[1] * local[2],
            center[2] + right[2] * local[0] + up[2] * local[1] + forward[2] * local[2],
        )

    @staticmethod
    def wrap_angle(angle_deg):
        return (angle_deg + 180.0) % 360.0 - 180.0

    def update_telemetry(self, values):
        self.state.update(values)
        self.telemetry_stale = False
        self.render()

    def set_telemetry_stale(self, stale):
        if self.telemetry_stale != stale:
            self.telemetry_stale = stale
            self.render()

    def set_mouse_captured(self, captured: bool):
        self.mouse_captured = captured
        self.configure(cursor="none" if captured else "crosshair")
        self.render()

    def camera_basis(self):
        body = (0.0, 0.0, 0.0)
        _, _, body_forward = self.orientation_basis(
            self.state["chassis_yaw_deg"],
            0.0,
        )
        camera = self.add(
            self.sub(body, self.mul(body_forward, 8.0)),
            (0.0, 5.3, 0.0),
        )
        target = self.add(body, self.add(self.mul(body_forward, 1.3), (0.0, 0.8, 0.0)))
        camera_forward = self.normalize(self.sub(target, camera))
        camera_right = self.normalize(self.cross((0.0, 1.0, 0.0), camera_forward))
        camera_up = self.normalize(self.cross(camera_forward, camera_right))
        return camera, camera_right, camera_up, camera_forward

    def project(self, point, camera_basis):
        camera, right, up, forward = camera_basis
        relative = self.sub(point, camera)
        depth = self.dot(relative, forward)
        if depth <= 0.15:
            return None
        width = max(self.winfo_width(), 10)
        height = max(self.winfo_height(), 10)
        focal = min(width, height) * 1.05
        x = width / 2 + self.dot(relative, right) * focal / depth
        y = height * 0.54 - self.dot(relative, up) * focal / depth
        return (x, y, depth)

    @staticmethod
    def shade(color, factor):
        color = color.lstrip("#")
        red = int(color[0:2], 16)
        green = int(color[2:4], 16)
        blue = int(color[4:6], 16)
        red = int(clamp(red * factor, 0, 255))
        green = int(clamp(green * factor, 0, 255))
        blue = int(clamp(blue * factor, 0, 255))
        return f"#{red:02x}{green:02x}{blue:02x}"

    def box_faces(self, center, size, yaw_deg, pitch_deg, color, camera_basis):
        half_x, half_y, half_z = size[0] / 2, size[1] / 2, size[2] / 2
        local_vertices = [
            (-half_x, -half_y, -half_z),
            (half_x, -half_y, -half_z),
            (half_x, half_y, -half_z),
            (-half_x, half_y, -half_z),
            (-half_x, -half_y, half_z),
            (half_x, -half_y, half_z),
            (half_x, half_y, half_z),
            (-half_x, half_y, half_z),
        ]
        world_vertices = [
            self.local_to_world(center, vertex, yaw_deg, pitch_deg)
            for vertex in local_vertices
        ]
        projected = [self.project(vertex, camera_basis) for vertex in world_vertices]
        faces = []
        shade_factors = [0.55, 0.85, 0.70, 0.62, 1.0, 0.72]
        for face_index, indices in enumerate(self.BOX_FACES):
            points = [projected[index] for index in indices]
            if any(point is None for point in points):
                continue
            coords = [(point[0], point[1]) for point in points]
            depth = sum(point[2] for point in points) / len(points)
            faces.append((depth, coords, self.shade(color, shade_factors[face_index])))
        return faces

    def draw_grid(self, camera_basis):
        spacing = 2.0
        extent = 18
        for index in range(-extent, extent + 1):
            x = index * spacing
            p1 = self.project((x, 0.0, -extent * spacing), camera_basis)
            p2 = self.project((x, 0.0, extent * spacing), camera_basis)
            if p1 and p2:
                self.create_line(p1[0], p1[1], p2[0], p2[1], fill="#163149")

            z = index * spacing
            p1 = self.project((-extent * spacing, 0.0, z), camera_basis)
            p2 = self.project((extent * spacing, 0.0, z), camera_basis)
            if p1 and p2:
                self.create_line(p1[0], p1[1], p2[0], p2[1], fill="#163149")

    def draw_direction_line(self, origin, yaw_deg, pitch_deg, length, color, camera_basis, dash=None, width=2):
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        direction = (
            math.sin(yaw) * math.cos(pitch),
            math.sin(pitch),
            math.cos(yaw) * math.cos(pitch),
        )
        endpoint = self.add(origin, self.mul(direction, length))
        start_2d = self.project(origin, camera_basis)
        end_2d = self.project(endpoint, camera_basis)
        if not start_2d or not end_2d:
            return None
        self.create_line(
            start_2d[0],
            start_2d[1],
            end_2d[0],
            end_2d[1],
            fill=color,
            width=width,
            dash=dash,
        )
        return end_2d

    def render(self):
        self.delete("all")
        width = max(self.winfo_width(), 10)
        height = max(self.winfo_height(), 10)
        camera_basis = self.camera_basis()
        self.draw_grid(camera_basis)

        body_center = (0.0, 0.55, 0.0)
        body_yaw = self.state["chassis_yaw_deg"]
        body_pitch = self.state["chassis_pitch_deg"]

        face_queue = []
        face_queue.extend(
            self.box_faces(
                body_center,
                (3.1, 0.75, 5.0),
                body_yaw,
                body_pitch,
                "#627348",
                camera_basis,
            )
        )
        for side in (-1.0, 1.0):
            track_center = self.local_to_world(
                body_center,
                (side * 1.75, -0.08, 0.0),
                body_yaw,
                body_pitch,
            )
            face_queue.extend(
                self.box_faces(
                    track_center,
                    (0.55, 0.55, 5.25),
                    body_yaw,
                    body_pitch,
                    "#30363b",
                    camera_basis,
                )
            )

        turret_center = self.local_to_world(
            body_center,
            (0.0, 0.80, 0.0),
            body_yaw,
            body_pitch,
        )
        face_queue.extend(
            self.box_faces(
                turret_center,
                (2.25, 0.78, 2.35),
                self.state["turret_yaw_deg"],
                body_pitch,
                "#748955",
                camera_basis,
            )
        )

        for _, points, color in sorted(face_queue, key=lambda item: item[0], reverse=True):
            flat = [coordinate for point in points for coordinate in point]
            self.create_polygon(*flat, fill=color, outline="#202b32")

        gun_origin = self.add(turret_center, (0.0, 0.18, 0.0))
        barrel_tip = self.draw_direction_line(
            gun_origin,
            self.state["turret_yaw_deg"],
            self.state["gun_pitch_deg"],
            4.7,
            "#172027",
            camera_basis,
            width=9,
        )
        self.draw_direction_line(
            gun_origin,
            self.state["turret_yaw_deg"],
            self.state["gun_pitch_deg"],
            4.7,
            "#b2a86a",
            camera_basis,
            width=5,
        )

        target_tip = self.draw_direction_line(
            gun_origin,
            self.state["target_yaw_deg"],
            self.state["target_pitch_deg"],
            6.4,
            ACCENT,
            camera_basis,
            dash=(6, 4),
            width=2,
        )
        if target_tip:
            x, y = target_tip[0], target_tip[1]
            self.create_oval(x - 9, y - 9, x + 9, y + 9, outline=ACCENT, width=2)
            self.create_line(x - 14, y, x + 14, y, fill=ACCENT)
            self.create_line(x, y - 14, x, y + 14, fill=ACCENT)
        if barrel_tip:
            self.create_oval(
                barrel_tip[0] - 3,
                barrel_tip[1] - 3,
                barrel_tip[0] + 3,
                barrel_tip[1] + 3,
                fill=YELLOW,
                outline="",
            )

        sensors_ok = self.state["imu_healthy"] and self.state["yaw_sensor_healthy"]
        status_color = RED if self.telemetry_stale else GREEN if sensors_ok else YELLOW
        self.create_rectangle(0, 0, width, 42, fill="#07111c", outline="")
        self.create_text(
            12,
            11,
            anchor="nw",
            text="第三人称实时遥测视图",
            fill=TEXT,
            font=("Microsoft YaHei UI", 11, "bold"),
        )
        self.create_text(
            width - 12,
            11,
            anchor="ne",
            text=(
                "遥测过期"
                if self.telemetry_stale
                else "双稳 ON"
                if self.state["stabilizer_enabled"]
                else "双稳 OFF"
            ),
            fill=status_color,
            font=("Microsoft YaHei UI", 10, "bold"),
        )

        yaw_error = self.wrap_angle(
            self.state["target_yaw_deg"] - self.state["turret_yaw_deg"]
        )
        pitch_error = self.state["target_pitch_deg"] - self.state["gun_pitch_deg"]
        info = (
            f"车体 Y {self.state['chassis_yaw_deg']:+.1f}° / P {body_pitch:+.1f}°   "
            f"炮塔世界 {self.state['turret_yaw_deg']:+.1f}°   "
            f"相对 {self.state['turret_relative_yaw_deg']:+.1f}°\n"
            f"炮管 {self.state['gun_pitch_deg']:+.1f}°   "
            f"目标 Y {self.state['target_yaw_deg']:+.1f}° / P {self.state['target_pitch_deg']:+.1f}°   "
            f"误差 {yaw_error:+.1f}° / {pitch_error:+.1f}°\n"
            f"Yaw 输出 {self.state['yaw_voltage']:+.3f}V   "
            f"舵机命令 {self.state['servo_command_deg']:.1f}°   "
            f"IMU {'OK' if self.state['imu_healthy'] else 'ERR'} / "
            f"AS5600 {'OK' if self.state['yaw_sensor_healthy'] else 'ERR'}"
        )
        self.create_rectangle(8, height - 70, width - 8, height - 8, fill="#07111c", outline=GRID)
        self.create_text(
            15,
            height - 65,
            anchor="nw",
            text=info,
            fill=MUTED,
            font=("Consolas", 8),
        )
        self.create_text(
            12,
            49,
            anchor="nw",
            text="鼠标已捕获：输入只转换为右摇杆，模型仅跟随 ESP32 遥测"
            if self.mouse_captured
            else "点击视窗捕获鼠标；蓝色虚线为 ESP32 当前稳定目标",
            fill=GREEN if self.mouse_captured else MUTED,
            font=("Microsoft YaHei UI", 9),
        )


class ParameterRow:
    def __init__(
        self,
        master,
        name: str,
        value: float,
        minimum: float,
        maximum: float,
        label: str,
        description: str,
        send_callback,
    ):
        self.name = name
        self.minimum = minimum
        self.maximum = maximum
        self.send_callback = send_callback
        self.dirty = False
        self.variable = tk.DoubleVar(value=value)
        self.entry_variable = tk.StringVar(value=self.format_value(value))

        self.frame = tk.Frame(master, bg=PANEL, padx=8, pady=7)
        self.frame.pack(fill="x", padx=5, pady=3)

        top = tk.Frame(self.frame, bg=PANEL)
        top.pack(fill="x")
        self.name_label = tk.Label(
            top,
            text=label,
            bg=PANEL,
            fg=TEXT,
            anchor="w",
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        self.name_label.pack(side="left")
        tk.Label(
            top,
            text=name,
            bg=PANEL,
            fg=MUTED,
            anchor="w",
            font=("Consolas", 8),
        ).pack(side="left", padx=(8, 0))

        self.entry = ttk.Entry(top, textvariable=self.entry_variable, width=11)
        self.entry.pack(side="right", padx=(5, 0))
        ttk.Button(top, text="应用", width=6, command=self.apply).pack(side="right")

        tk.Label(
            self.frame,
            text=description,
            bg=PANEL,
            fg=MUTED,
            anchor="w",
            justify="left",
            wraplength=650,
            font=("Microsoft YaHei UI", 8),
        ).pack(fill="x", pady=(2, 3))

        scale_row = tk.Frame(self.frame, bg=PANEL)
        scale_row.pack(fill="x")
        tk.Label(scale_row, text=self.format_value(minimum), bg=PANEL, fg=MUTED, width=9).pack(side="left")
        self.scale = ttk.Scale(
            scale_row,
            from_=minimum,
            to=maximum,
            variable=self.variable,
            command=self.on_scale,
        )
        self.scale.pack(side="left", fill="x", expand=True)
        tk.Label(scale_row, text=self.format_value(maximum), bg=PANEL, fg=MUTED, width=9).pack(side="left")

        self.entry.bind("<Return>", lambda _: self.apply())
        self.entry.bind("<KeyRelease>", lambda _: self.mark_dirty())

    @staticmethod
    def format_value(value: float) -> str:
        if abs(value) >= 100:
            return f"{value:.2f}"
        if abs(value) >= 10:
            return f"{value:.3f}"
        return f"{value:.5f}"

    def on_scale(self, raw_value: str) -> None:
        value = float(raw_value)
        self.entry_variable.set(self.format_value(value))
        self.mark_dirty()

    def mark_dirty(self) -> None:
        self.dirty = True
        self.name_label.configure(fg=YELLOW)

    def set_value(self, value: float) -> None:
        value = clamp(value, self.minimum, self.maximum)
        self.variable.set(value)
        self.entry_variable.set(self.format_value(value))
        self.dirty = False
        self.name_label.configure(fg=TEXT)

    def get_value(self) -> float:
        try:
            return clamp(float(self.entry_variable.get()), self.minimum, self.maximum)
        except ValueError:
            return self.variable.get()

    def apply(self) -> None:
        value = self.get_value()
        self.set_value(value)
        self.send_callback(self.name, value)


class ScrollableFrame(tk.Frame):
    def __init__(self, master):
        super().__init__(master, bg=BG)
        canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.content = tk.Frame(canvas, bg=BG)
        window = canvas.create_window((0, 0), window=self.content, anchor="nw")

        self.content.bind("<Configure>", lambda _: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")


class DebugGui:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.pad = PadState(mouse_sensitivity=args.mouse_sensitivity)
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.ui_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.stop_event = threading.Event()
        self.connected = False
        self.emergency_stopped = False
        self.latest_pad_command = "pad tl=0.000 tr=0.000 jlx=0.000 jrx=0.000 jry=0.000 a=0\n"
        self.pad_command_lock = threading.Lock()
        self.mouse_captured = False
        self.mouse_warp_pending = False
        self.rx_buffer = ""
        self.parameter_rows: dict[str, ParameterRow] = {}
        self.last_telemetry_at = 0.0

        self.root = tk.Tk()
        self.root.title("Chieftain MK10 调试控制台")
        self.root.geometry("1360x850")
        self.root.minsize(1100, 700)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.configure_styles()
        self.build_header()
        self.build_main_area()
        self.build_log_area()
        self.bind_inputs()

    def configure_styles(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=PANEL, foreground=TEXT, fieldbackground=PANEL_ALT)
        style.configure("TButton", background=PANEL_ALT, foreground=TEXT, padding=6)
        style.map("TButton", background=[("active", ACCENT)])
        style.configure("TEntry", fieldbackground=PANEL_ALT, foreground=TEXT, insertcolor=TEXT)
        style.configure("TScale", background=PANEL)
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL_ALT, foreground=TEXT, padding=(12, 7))
        style.map("TNotebook.Tab", background=[("selected", ACCENT)])

    def build_header(self) -> None:
        header = tk.Frame(self.root, bg=PANEL, height=64)
        header.pack(fill="x")

        tk.Label(
            header,
            text="CHIEFTAIN MK10",
            bg=PANEL,
            fg=TEXT,
            font=("Consolas", 18, "bold"),
        ).pack(side="left", padx=(16, 8), pady=10)
        tk.Label(
            header,
            text="PC 调试控制台",
            bg=PANEL,
            fg=MUTED,
            font=("Microsoft YaHei UI", 11),
        ).pack(side="left")

        self.connection_dot = tk.Label(header, text="●", bg=PANEL, fg=YELLOW, font=("Arial", 18))
        self.connection_dot.pack(side="right", padx=(6, 16))
        self.connection_label = tk.Label(
            header,
            text="等待扫描",
            bg=PANEL,
            fg=MUTED,
            font=("Microsoft YaHei UI", 10),
        )
        self.connection_label.pack(side="right")
        ttk.Button(header, text="读取参数", command=lambda: self.enqueue_command("get")).pack(
            side="right", padx=5, pady=10
        )

    def build_main_area(self) -> None:
        paned = tk.PanedWindow(
            self.root,
            orient="horizontal",
            bg=BG,
            sashwidth=6,
            sashrelief="flat",
            bd=0,
        )
        paned.pack(fill="both", expand=True, padx=10, pady=10)

        controls = tk.Frame(paned, bg=BG, width=430)
        parameters = tk.Frame(paned, bg=BG)
        paned.add(controls, minsize=390)
        paned.add(parameters, minsize=620)

        control_scroll = ScrollableFrame(controls)
        control_scroll.pack(fill="both", expand=True)
        self.build_control_panel(control_scroll.content)
        self.build_parameter_panel(parameters)

    def build_control_panel(self, parent) -> None:
        status_card = tk.Frame(parent, bg=PANEL, padx=12, pady=10)
        status_card.pack(fill="x", pady=(0, 8))
        tk.Label(
            status_card,
            text="键鼠驾驶",
            bg=PANEL,
            fg=TEXT,
            font=("Microsoft YaHei UI", 13, "bold"),
        ).pack(anchor="w")
        tk.Label(
            status_card,
            text="W 油门  S 制动/倒车  A/D 转向  鼠标映射右摇杆  Space 映射 A 键",
            bg=PANEL,
            fg=MUTED,
            justify="left",
            wraplength=390,
        ).pack(anchor="w", pady=(3, 6))

        key_row = tk.Frame(status_card, bg=PANEL)
        key_row.pack()
        self.key_labels = {}
        for key in ["W", "A", "S", "D", "SPACE"]:
            label = tk.Label(
                key_row,
                text=key,
                width=7 if key == "SPACE" else 3,
                height=2,
                bg=PANEL_ALT,
                fg=MUTED,
                font=("Consolas", 10, "bold"),
            )
            label.pack(side="left", padx=3)
            self.key_labels[key.lower()] = label

        self.gamepad_view = VirtualGamepadView(
            parent,
            height=185,
        )
        self.gamepad_view.pack(fill="x", pady=(0, 8))

        self.tank_view = ThirdPersonTankView(
            parent,
            height=330,
        )
        self.tank_view.pack(fill="x", pady=(0, 8))
        self.tank_view.bind("<Button-1>", self.capture_mouse)
        self.tank_view.bind("<Motion>", self.on_mouse_motion)

        adjust_card = tk.Frame(parent, bg=PANEL, padx=12, pady=8)
        adjust_card.pack(fill="x", pady=(0, 8))

        self.drive_scale_var = tk.DoubleVar(value=1.0)
        self.mouse_sensitivity_var = tk.DoubleVar(value=self.args.mouse_sensitivity)
        self.mouse_response_var = tk.DoubleVar(value=0.18)
        self.add_adjustment(
            adjust_card,
            "键盘最大输入",
            self.drive_scale_var,
            0.1,
            1.0,
            lambda value: self.pad.set_drive_scale(float(value)),
        )
        self.add_adjustment(
            adjust_card,
            "鼠标角度灵敏度",
            self.mouse_sensitivity_var,
            0.02,
            0.50,
            lambda value: self.pad.set_mouse_sensitivity(float(value)),
        )
        self.add_adjustment(
            adjust_card,
            "鼠标转换时间",
            self.mouse_response_var,
            0.05,
            0.60,
            lambda value: self.pad.set_mouse_response_time(float(value)),
        )

        safety = tk.Frame(parent, bg=BG)
        safety.pack(fill="x")
        self.stop_button = tk.Button(
            safety,
            text="紧急停车",
            bg=RED,
            fg="white",
            activebackground="#d94352",
            activeforeground="white",
            relief="flat",
            font=("Microsoft YaHei UI", 12, "bold"),
            command=self.emergency_stop,
        )
        self.stop_button.pack(side="left", fill="x", expand=True, padx=(0, 4), ipady=8)
        self.arm_button = tk.Button(
            safety,
            text="解除急停",
            bg=PANEL_ALT,
            fg=TEXT,
            activebackground=GREEN,
            relief="flat",
            font=("Microsoft YaHei UI", 11),
            command=self.release_emergency_stop,
        )
        self.arm_button.pack(side="left", fill="x", expand=True, padx=(4, 0), ipady=8)

        self.safety_label = tk.Label(
            parent,
            text="安全状态：等待连接",
            bg=BG,
            fg=MUTED,
            font=("Microsoft YaHei UI", 9),
        )
        self.safety_label.pack(anchor="w", pady=(5, 0))

    def add_adjustment(self, parent, label, variable, minimum, maximum, callback) -> None:
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", pady=3)
        tk.Label(row, text=label, bg=PANEL, fg=MUTED, width=12, anchor="w").pack(side="left")
        ttk.Scale(row, from_=minimum, to=maximum, variable=variable, command=callback).pack(
            side="left", fill="x", expand=True
        )
        value_label = tk.Label(row, bg=PANEL, fg=TEXT, width=7, font=("Consolas", 9))
        value_label.pack(side="right")

        def refresh(*_):
            value_label.configure(text=f"{variable.get():.3f}")

        variable.trace_add("write", refresh)
        refresh()

    def build_parameter_panel(self, parent) -> None:
        toolbar = tk.Frame(parent, bg=PANEL, padx=8, pady=8)
        toolbar.pack(fill="x", pady=(0, 8))
        ttk.Button(toolbar, text="从 ESP32 读取", command=lambda: self.enqueue_command("get")).pack(side="left", padx=3)
        ttk.Button(toolbar, text="应用全部改动", command=self.apply_all_parameters).pack(side="left", padx=3)
        ttk.Button(toolbar, text="保存到 ESP32", command=lambda: self.enqueue_command("save")).pack(side="left", padx=3)
        ttk.Button(toolbar, text="重新加载保存值", command=self.load_saved_parameters).pack(side="left", padx=3)
        ttk.Button(toolbar, text="恢复默认值", command=self.restore_defaults).pack(side="left", padx=3)
        ttk.Button(toolbar, text="一键导出", command=self.export_parameters).pack(side="left", padx=3)

        self.parameter_status = tk.Label(
            toolbar,
            text="尚未读取参数",
            bg=PANEL,
            fg=MUTED,
            font=("Microsoft YaHei UI", 9),
        )
        self.parameter_status.pack(side="right", padx=6)

        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill="both", expand=True)
        self.group_frames = {}
        for group in GROUP_ORDER:
            frame = ScrollableFrame(self.notebook)
            self.notebook.add(frame, text=group)
            self.group_frames[group] = frame.content

    def build_log_area(self) -> None:
        container = tk.Frame(self.root, bg=PANEL, padx=8, pady=7)
        container.pack(fill="x", padx=10, pady=(0, 10))

        command_row = tk.Frame(container, bg=PANEL)
        command_row.pack(fill="x")
        tk.Label(command_row, text="手动命令", bg=PANEL, fg=MUTED).pack(side="left")
        self.command_entry = ttk.Entry(command_row)
        self.command_entry.pack(side="left", fill="x", expand=True, padx=7)
        self.command_entry.bind("<Return>", self.submit_manual_command)
        ttk.Button(command_row, text="发送", command=self.submit_manual_command).pack(side="left")
        ttk.Button(command_row, text="清空日志", command=self.clear_log).pack(side="left", padx=(5, 0))

        self.log = tk.Text(
            container,
            height=8,
            bg="#080d13",
            fg="#b9c8d8",
            insertbackground=TEXT,
            relief="flat",
            font=("Consolas", 9),
            state="disabled",
        )
        self.log.pack(fill="x", pady=(6, 0))

    def bind_inputs(self) -> None:
        self.root.bind_all("<KeyPress>", self.on_key_press)
        self.root.bind_all("<KeyRelease>", self.on_key_release)
        self.root.bind_all("<Escape>", self.on_escape)
        self.root.bind("<FocusOut>", self.on_window_focus_out)

    def start_ble_thread(self) -> None:
        thread = threading.Thread(target=lambda: asyncio.run(self.ble_main()), daemon=True)
        thread.start()

    def run(self) -> None:
        self.start_ble_thread()
        self.root.after(40, self.process_ui_events)
        self.root.after(20, self.update_local_pad)
        self.root.mainloop()

    def close(self) -> None:
        self.emergency_stop()
        self.root.after(120, self.finish_close)

    def finish_close(self) -> None:
        self.stop_event.set()
        self.root.destroy()

    def enqueue_command(self, command: str, show_in_log: bool = True) -> None:
        command = command.strip()
        if not command:
            return
        self.command_queue.put(command + "\n")
        if show_in_log:
            self.append_log(f"> {command}\n", ACCENT)

    def submit_manual_command(self, event=None) -> None:
        command = self.command_entry.get()
        self.command_entry.delete(0, "end")
        self.enqueue_command(command)

    def append_log(self, text: str, color: str = TEXT) -> None:
        self.log.configure(state="normal")
        tag = f"color_{color}"
        if tag not in self.log.tag_names():
            self.log.tag_configure(tag, foreground=color)
        self.log.insert("end", text, tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def process_ui_events(self) -> None:
        while True:
            try:
                event_type, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break

            if event_type == "status":
                self.set_connection_status(str(payload))
            elif event_type == "rx":
                self.consume_rx_text(str(payload))
            elif event_type == "log":
                self.append_log(str(payload), MUTED)

        telemetry_stale = (
            self.last_telemetry_at == 0.0
            or time.monotonic() - self.last_telemetry_at > 0.5
        )
        self.tank_view.set_telemetry_stale(telemetry_stale)

        if not self.stop_event.is_set():
            self.root.after(40, self.process_ui_events)

    def update_local_pad(self) -> None:
        pad_command, values = self.pad.snapshot()
        with self.pad_command_lock:
            self.latest_pad_command = pad_command
        self.update_control_visuals(values)

        if not self.stop_event.is_set():
            self.root.after(20, self.update_local_pad)

    def get_latest_pad_command(self) -> str:
        with self.pad_command_lock:
            return self.latest_pad_command

    def set_connection_status(self, status: str) -> None:
        self.connection_label.configure(text=status)
        if status == "已连接":
            self.connected = True
            self.connection_dot.configure(fg=GREEN)
            self.safety_label.configure(text="安全状态：输入链路正常", fg=GREEN)
        elif status == "扫描中":
            self.connected = False
            self.last_telemetry_at = 0.0
            self.rx_buffer = ""
            self.connection_dot.configure(fg=YELLOW)
        else:
            self.connected = False
            self.last_telemetry_at = 0.0
            self.rx_buffer = ""
            self.connection_dot.configure(fg=RED)
            self.safety_label.configure(text="安全状态：连接断开，ESP32 将超时停车", fg=RED)

    def consume_rx_text(self, text: str) -> None:
        self.rx_buffer += text
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            self.process_rx_line(line.strip())

    def process_rx_line(self, line: str) -> None:
        if not line:
            return
        if line == "PARAMS":
            self.parameter_status.configure(text="正在读取参数...", fg=YELLOW)
            return
        if line == "END_PARAMS":
            self.parameter_status.configure(text=f"已读取 {len(self.parameter_rows)} 个参数", fg=GREEN)
            self.append_log("参数列表读取完成\n", GREEN)
            return
        if line.startswith("STATE ESTOP="):
            self.emergency_stopped = line.endswith("1")
            if self.emergency_stopped:
                self.safety_label.configure(text="安全状态：急停已锁存", fg=RED)
            else:
                self.safety_label.configure(text="安全状态：输入链路正常", fg=GREEN)
            return
        if line.startswith("TEL "):
            telemetry = {}
            for token in line.split()[1:]:
                if "=" not in token:
                    continue
                key, value = token.split("=", 1)
                try:
                    telemetry[key] = float(value)
                except ValueError:
                    return

            required = {"cy", "cp", "ty", "tr", "gp", "yt", "pt", "yv", "sv", "st", "ih", "yh"}
            if not required.issubset(telemetry):
                return

            self.last_telemetry_at = time.monotonic()
            self.tank_view.update_telemetry(
                {
                    "chassis_yaw_deg": telemetry["cy"],
                    "chassis_pitch_deg": telemetry["cp"],
                    "turret_yaw_deg": telemetry["ty"],
                    "turret_relative_yaw_deg": telemetry["tr"],
                    "gun_pitch_deg": telemetry["gp"],
                    "target_yaw_deg": telemetry["yt"],
                    "target_pitch_deg": telemetry["pt"],
                    "yaw_voltage": telemetry["yv"],
                    "servo_command_deg": telemetry["sv"],
                    "stabilizer_enabled": telemetry["st"],
                    "imu_healthy": telemetry["ih"],
                    "yaw_sensor_healthy": telemetry["yh"],
                }
            )
            return

        match = PARAM_PATTERN.match(line)
        if match:
            name = match.group(1)
            self.update_parameter(
                name,
                float(match.group(2)),
                float(match.group(3)),
                float(match.group(4)),
            )
            return

        set_match = SET_PATTERN.match(line)
        if set_match:
            name = set_match.group(1)
            value = float(set_match.group(2))
            if name in self.parameter_rows:
                self.parameter_rows[name].set_value(value)
            if name == "REAL_TURRET_VEL":
                self.pad.set_turret_rate(value)

        if "emergency_stop_latched" in line:
            self.emergency_stopped = True
            self.safety_label.configure(text="安全状态：急停已锁存", fg=RED)
        elif "emergency_stop_released" in line:
            self.emergency_stopped = False
            self.safety_label.configure(text="安全状态：急停已解除", fg=GREEN)

        color = RED if line.startswith("ERR") else GREEN if line.startswith("OK") else MUTED
        self.append_log(line + "\n", color)

    def update_parameter(self, name: str, value: float, minimum: float, maximum: float) -> None:
        if name in self.parameter_rows:
            self.parameter_rows[name].set_value(value)
            if name == "REAL_TURRET_VEL":
                self.pad.set_turret_rate(value)
            return

        group, label, description = PARAM_META.get(name, ("其他", name, "ESP32 运行时可调参数"))
        row = ParameterRow(
            self.group_frames[group],
            name,
            value,
            minimum,
            maximum,
            label,
            description,
            self.send_parameter,
        )
        self.parameter_rows[name] = row
        if name == "REAL_TURRET_VEL":
            self.pad.set_turret_rate(value)

    def send_parameter(self, name: str, value: float) -> None:
        self.enqueue_command(f"set {name} {value:.6f}")

    def apply_all_parameters(self) -> None:
        dirty_rows = [row for row in self.parameter_rows.values() if row.dirty]
        if not dirty_rows:
            self.append_log("没有待应用的参数改动\n", MUTED)
            return
        for row in dirty_rows:
            row.apply()
        self.parameter_status.configure(text=f"已发送 {len(dirty_rows)} 项修改", fg=YELLOW)

    def load_saved_parameters(self) -> None:
        self.enqueue_command("load")
        self.enqueue_command("get", show_in_log=False)

    def restore_defaults(self) -> None:
        self.enqueue_command("defaults")
        self.enqueue_command("get", show_in_log=False)

    def export_parameters(self) -> None:
        if not self.parameter_rows:
            self.append_log("当前没有可导出的参数，请先连接 ESP32 并读取参数\n", YELLOW)
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
                name,
                ("其他", name, "ESP32 运行时可调参数"),
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
            "note": "pending_unsent_edit=true 表示该界面值尚未点击应用发送给 ESP32",
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
            "下面的命令可在 PC 调试控制台中逐行发送：",
        ]
        for name, item in parameters.items():
            dirty_mark = "  # 注意：界面中尚未应用" if item["pending_unsent_edit"] else ""
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
        self.parameter_status.configure(
            text=f"已导出 {len(parameters)} 个参数",
            fg=GREEN,
        )
        self.append_log(f"参数已导出：{json_path}\n", GREEN)
        self.append_log(f"命令清单：{text_path}\n", GREEN)
        if dirty_count:
            self.append_log(
                f"注意：导出内容中有 {dirty_count} 项尚未点击应用\n",
                YELLOW,
            )

    def emergency_stop(self) -> None:
        self.pad.clear()
        self.release_mouse()
        self.emergency_stopped = True
        self.safety_label.configure(text="安全状态：正在请求急停", fg=RED)
        self.enqueue_command("stop")

    def release_emergency_stop(self) -> None:
        self.pad.clear()
        self.enqueue_command("arm")

    def normalize_key(self, event) -> str:
        if event.keysym == "space":
            return "space"
        return event.keysym.lower()

    def on_key_press(self, event) -> None:
        if is_text_input(event.widget):
            return
        key = self.normalize_key(event)
        if key not in {"w", "a", "s", "d", "space"}:
            return
        is_new = self.pad.key_down(key)
        self.set_key_indicator(key, True)
        if key == "space" and is_new:
            self.append_log("已映射手柄 A 键；双稳结果等待 ESP32 遥测确认\n", ACCENT)

    def on_key_release(self, event) -> None:
        key = self.normalize_key(event)
        if key in {"w", "a", "s", "d", "space"}:
            self.pad.key_up(key)
            self.set_key_indicator(key, False)

    def set_key_indicator(self, key: str, active: bool) -> None:
        label = self.key_labels.get(key)
        if label:
            label.configure(bg=ACCENT if active else PANEL_ALT, fg="white" if active else MUTED)

    def clear_key_indicators(self) -> None:
        for key in ["w", "a", "s", "d", "space"]:
            self.set_key_indicator(key, False)

    def on_escape(self, event=None) -> None:
        self.emergency_stop()

    def on_window_focus_out(self, event) -> None:
        self.root.after(20, self.clear_if_window_inactive)

    def clear_if_window_inactive(self) -> None:
        if self.root.focus_displayof() is None:
            self.pad.clear()
            self.clear_key_indicators()
            self.release_mouse()

    def capture_mouse(self, event=None) -> None:
        if self.emergency_stopped:
            self.append_log("急停锁存中，请先解除急停\n", RED)
            return
        self.mouse_captured = True
        self.tank_view.set_mouse_captured(True)
        self.tank_view.grab_set()
        self.tank_view.focus_set()
        self.root.after_idle(self.warp_mouse_to_center)

    def release_mouse(self) -> None:
        if self.mouse_captured:
            try:
                self.tank_view.grab_release()
            except tk.TclError:
                pass
        self.mouse_captured = False
        self.tank_view.set_mouse_captured(False)

    def warp_mouse_to_center(self) -> None:
        if not self.mouse_captured:
            return
        width = max(self.tank_view.winfo_width(), 10)
        height = max(self.tank_view.winfo_height(), 10)
        self.mouse_warp_pending = True
        self.tank_view.event_generate("<Motion>", warp=True, x=width // 2, y=height // 2)

    def on_mouse_motion(self, event) -> None:
        if not self.mouse_captured:
            return
        width = max(self.tank_view.winfo_width(), 10)
        height = max(self.tank_view.winfo_height(), 10)
        center_x = width / 2
        center_y = height / 2

        if self.mouse_warp_pending and abs(event.x - center_x) <= 2 and abs(event.y - center_y) <= 2:
            self.mouse_warp_pending = False
            return

        dx = event.x - center_x
        dy = event.y - center_y
        if dx or dy:
            self.pad.add_mouse_delta(dx, dy)
        self.root.after_idle(self.warp_mouse_to_center)

    def update_control_visuals(self, values: dict[str, float]) -> None:
        self.gamepad_view.update_state(values)

    async def find_device(self):
        self.ui_queue.put(("status", "扫描中"))
        self.ui_queue.put(("log", f"正在扫描 BLE 设备：{self.args.name}\n"))

        def match(device, advertisement_data):
            names = {device.name, advertisement_data.local_name}
            service_uuids = {uuid.lower() for uuid in advertisement_data.service_uuids}
            return self.args.name in names or SERVICE_UUID.lower() in service_uuids

        device = await BleakScanner.find_device_by_filter(match, timeout=12.0)
        if device is None:
            raise RuntimeError("未找到 Chieftain MK10 调试 BLE 设备")
        return device

    async def ble_main(self) -> None:
        while not self.stop_event.is_set():
            try:
                device = await self.find_device()
                self.ui_queue.put(("log", f"正在连接：{device.name} {device.address}\n"))
                async with BleakClient(device) as client:
                    self.ui_queue.put(("status", "已连接"))
                    self.ui_queue.put(("log", "BLE 已连接，开始以 50Hz 发送控制输入\n"))

                    def on_notify(_, data: bytearray):
                        self.ui_queue.put(("rx", data.decode(errors="replace")))

                    await client.start_notify(TX_UUID, on_notify)
                    await client.write_gatt_char(RX_UUID, b"get\n", response=False)

                    while not self.stop_event.is_set() and client.is_connected:
                        pad_command = self.get_latest_pad_command()
                        await client.write_gatt_char(RX_UUID, pad_command.encode(), response=False)

                        while True:
                            try:
                                command = self.command_queue.get_nowait()
                            except queue.Empty:
                                break
                            await client.write_gatt_char(RX_UUID, command.encode(), response=False)
                        await asyncio.sleep(0.02)

                self.ui_queue.put(("status", "连接断开"))
            except Exception as exc:
                self.ui_queue.put(("status", "连接失败"))
                self.ui_queue.put(("log", f"BLE 错误：{exc}\n2 秒后重试...\n"))
                await asyncio.sleep(2.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Chieftain MK10 PC BLE debug controller")
    parser.add_argument("--name", default="ChieftainMK10-Debug", help="ESP32 BLE device name")
    parser.add_argument(
        "--mouse-sensitivity",
        type=float,
        default=0.12,
        help="virtual angular displacement generated per mouse pixel",
    )
    args = parser.parse_args()

    gui = DebugGui(args)
    gui.run()


if __name__ == "__main__":
    main()
