#pragma once

#include <Arduino.h>

// ==========================================
// 0. 固件输入模式
// ==========================================
// ESP32 同时编译 Xbox BLE 和 Classic BluetoothSerial 会占用大量 Flash。
// 这里用编译期开关做“二选一”：
// - ROBOT_INPUT_MODE_XBOX：正常遥控模式，只启用 Xbox 手柄；
// - ROBOT_INPUT_MODE_PC_DEBUG：调车模式，只启用 PC 蓝牙串口调试工具。
// 修改下面这一行后重新烧录即可切换模式。
#define ROBOT_INPUT_MODE_XBOX 1
#define ROBOT_INPUT_MODE_PC_DEBUG 2
#define ROBOT_INPUT_MODE ROBOT_INPUT_MODE_PC_DEBUG

#if ROBOT_INPUT_MODE != ROBOT_INPUT_MODE_XBOX && ROBOT_INPUT_MODE != ROBOT_INPUT_MODE_PC_DEBUG
  #error "ROBOT_INPUT_MODE must be ROBOT_INPUT_MODE_XBOX or ROBOT_INPUT_MODE_PC_DEBUG"
#endif

// ==========================================
// 1. 全局配置参数
// ==========================================
// 这个文件只放“参数”和“引脚”，不放控制逻辑。
// 调车时优先来这里改数值；对应算法实现分别在 chassis/control/turret/hardware 中。
namespace Config {
    // ---------- 通讯 ----------
    const uint32_t USB_SERIAL_BAUD = 115200;                 // USB启动日志与运行日志统一波特率
    const char* const XBOX_MAC = "28:ea:0b:d9:0b:9f"; // Xbox 手柄蓝牙 MAC 地址
    const char* const DEBUG_BT_NAME = "ChieftainMK10-Debug"; // PC 调试模式下显示的蓝牙串口名称
    const uint32_t DEBUG_INPUT_TIMEOUT_MS = 300;             // PC 输入超过该时间未刷新就停车
    const uint32_t DEBUG_TELEMETRY_MS = 50;                  // PC 遥测发送周期，50ms = 20Hz
    const float DEBUG_DIRECT_PWM_MAX = 180.0f;               // 固定 PWM 硬件诊断限幅

    // ---------- I2C 总线 ----------
    const uint8_t I2C_FOC_SDA = 17, I2C_FOC_SCL = 16; // AS5600/FOC 相关 I2C
    const uint8_t I2C_IMU_SDA = 22, I2C_IMU_SCL = 21; // 两颗 MPU6050 使用的 I2C

    // ---------- 电池电压检测 ----------
    // 使用 ADC1，避免 ESP32 蓝牙/Wi-Fi 占用 ADC2 带来的冲突。
    const uint8_t VBAT_ADC_PIN = 39;
    const float VBAT_DIVIDER_R1 = 100000.0f;
    const float VBAT_DIVIDER_R2 = 33000.0f;
    // 同时实测：ESP32=12.24V、万用表电池端=12.05V、VN=2.97V。
    // 对完整 ADC + 分压链路做整体增益校准：12.05 / 12.24 = 0.9845。
    const float VBAT_CALIBRATION_GAIN = 0.9845f;
    const float VBAT_LPF = 0.1f;
    const float VBAT_WARN = 10.8f;             // 3S 低压预警
    const float VBAT_CUTOFF = 10.2f;           // 3S 低压截止
    const bool ENABLE_BATTERY_MONITOR = true;  // 已按实测分压确认；同时启用遥测、预警和低压截止

    // ---------- 底盘电机和 PWM ----------
    const uint8_t R_IN1 = 25, R_IN2 = 33, R_PWM = 32;
    const uint8_t L_IN1 = 14, L_IN2 = 27, L_PWM = 26;
    const uint8_t PWM_CH_R = 8, PWM_CH_L = 9;
    const uint32_t PWM_FREQ = 10000;
    const uint8_t PWM_RES = 8;
    const float MOTOR_PWM_DEADZONE = 1.0f;

    // ---------- 履带速度控制 ----------
    const float TRACK_STOP_DEADZONE_KMH = 0.10f;
    // 11.57V、整车落地固定 PWM 标定。运行基础动力由速度-PWM表提供，
    // 这里只保留静止起步门槛，避免与表格中的运行摩擦重复相加。
    const float TRACK_FF_REFERENCE_VOLTAGE = 11.57f;
    // 起步/维持门槛原测于12.11V，已等效换算到11.57V并向上取整。
    const float TRACK_FF_START_FORWARD_PWM = 35.0f;
    const float TRACK_FF_START_REVERSE_PWM = 36.0f;
    const float TRACK_FF_RUN_FORWARD_PWM = 30.0f;
    const float TRACK_FF_RUN_REVERSE_PWM = 31.0f;
    const float TRACK_FF_BATTERY_SCALE_MIN = 0.85f;
    const float TRACK_FF_BATTERY_SCALE_MAX = 1.20f;
    // 机构最低连续速度约2.8km/h。低于该速度不让PI反复启停，而由开环起步/停车状态接管。
    const float TRACK_MIN_CLOSED_LOOP_SPEED_KMH = 2.8f;
    // 起步分三段：先快速跨过无效 PWM 区，再柔和建立静摩擦扭矩，最后在确实未起步时增力。
    const uint32_t TRACK_LAUNCH_RUN_RAMP_MS = 100;
    const uint32_t TRACK_LAUNCH_START_RAMP_MS = 250;
    // 若起步门槛不足，先有限增力；仍无任何有效运动则卸载并等待松开输入后重试。
    const uint32_t TRACK_LAUNCH_BOOST_DELAY_MS = 350;
    const uint32_t TRACK_LAUNCH_BOOST_RAMP_MS = 700;
    const uint32_t TRACK_LAUNCH_FAIL_TIMEOUT_MS = 1800;
    const float TRACK_LAUNCH_MOTION_SPEED_KMH = 0.8f;
    const float TRACK_LAUNCH_STABLE_SPEED_KMH = 1.5f;
    const uint32_t TRACK_LAUNCH_STABLE_EVIDENCE_MS = 160;
    const float TRACK_LAUNCH_DROP_SPEED_KMH = 0.4f;
    const uint32_t TRACK_LAUNCH_DROP_RESET_MS = 100;
    const float TRACK_LAUNCH_MAX_FORWARD_PWM = 52.0f;
    const float TRACK_LAUNCH_MAX_REVERSE_PWM = 55.0f;
    // 原地转向的横向搓地阻力远大于直线滚动阻力，使用独立的开环起转门槛。
    const float TRACK_PIVOT_START_PWM = 65.0f;
    const float TRACK_PIVOT_MAX_PWM = 90.0f;

    // PID 反馈。KD 默认关闭；确认 PI 仍有超调/振荡后再逐步增加。
    const float TRACK_PID_KP = 3.0f;
    const float TRACK_PID_KI = 0.5f;
    const float TRACK_PID_KD = 0.0f;
    const float TRACK_PID_I_MAX_PWM = 50.0f;
    const float TRACK_PID_D_FILTER_TAU_S = 0.080f;
    const float TRACK_PID_D_MAX_PWM = 30.0f;
    // IMU pitch/yaw 虚拟惯量最终都会从这个入口叠加到单侧履带。
    // 限制总权限，避免两项同向叠加后主导速度闭环。
    const float TRACK_EXTERNAL_PWM_MAX = 12.0f;

    // ---------- 履带堵转保护 ----------
    const float TRACK_STALL_TARGET_MIN_KMH = 1.5f;
    const float TRACK_STALL_PWM_MIN = 95.0f;
    const float TRACK_STALL_ACTUAL_MAX_KMH = 0.08f;
    const uint32_t TRACK_STALL_GRACE_MS = 1000;
    const float TRACK_STALL_CLEAR_TARGET_KMH = 0.12f;

    // ---------- 编码器 ----------
    // PCNT 每10ms结算一次增量；最近四个增量组成40ms重叠测速窗口，
    // 因而以固定100Hz发布控制速度，窗口平均延迟仍约20ms。
    const uint32_t ENCODER_UPDATE_US = 10000;
    const uint8_t ENCODER_ROLLING_BINS = 4;
    const uint32_t ENCODER_STOP_TIMEOUT_US = 150000;
    // 实测5.8km/h时约8Hz，对应主动轮每圈约9次啮合扰动；PI支路只抑制该9阶。
    // 快速速度仍供停车和堵转使用，不经过陷波。
    const bool ENCODER_NOTCH_ENABLED = true;
    const float ENCODER_NOTCH_Q = 3.0f;
    const float ENCODER_NOTCH_ORDER = 9.0f;
    const float ENCODER_NOTCH_CENTER_TAU_S = 0.080f;
    const float ENCODER_NOTCH_FADE_START_KMH = 1.5f;
    const float ENCODER_NOTCH_FADE_FULL_KMH = 2.0f;
    // 50Hz采样的奈奎斯特频率为25Hz；9阶在高速区接近该边界，提前渐退。
    const float ENCODER_NOTCH_HIGH_FADE_START_KMH = 14.0f;
    const float ENCODER_NOTCH_HIGH_FADE_END_KMH = 16.0f;
    const float ENCODER_NOTCH_MAX_MIX = 1.0f;
    const float ENCODER_NOTCH_BLEND_TAU_S = 0.10f;
    // 由17mm主动轮周长和1:35速度映射得到：每1 km/h真车等效速度约0.1486圈/秒。
    const float ENCODER_SPROCKET_HZ_PER_KMH = 0.14862f;
    const float ENCODER_DISPLAY_FILTER_TAU_S = 0.120f;
    const float ENCODER_MAX_VALID_SPEED_KMH = 80.0f;
    // 最近一圈采用重叠窗口平均；历史不足一圈时自动退回当前短窗口测速。
    const uint8_t ENCODER_REV_HISTORY_SIZE = 128;
    const uint8_t R_ENCA = 35, R_ENCB = 34;
    const uint8_t L_ENCA = 23, L_ENCB = 4;
    const int8_t R_ENCODER_SIGN = 1;
    const int8_t L_ENCODER_SIGN = -1;

    // ---------- 1:35 真车速度映射 ----------
    const int SCALE = 35;
    const int ENCODER_PULSES_PER_MOTOR_REV = 7;
    // PCNT 同时统计 A 相上升沿和下降沿，因此每个完整脉冲对应两个计数。
    const int ENCODER_COUNTS_PER_MOTOR_REV = ENCODER_PULSES_PER_MOTOR_REV * 2;
    const int GEAR_RATIO = 59;
    const int ENCODER_COUNTS_PER_SPROCKET_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO;
    const float WHEEL_D = 0.017f;
    const float RPM_TO_REAL_KMH = (WHEEL_D * PI * 60.0f / 1000.0f) * SCALE;

    // ---------- 纵向动力学 ----------
    const float REAL_V_MAX = 48.0f;
    const float REAL_V_REV_MAX = 11.0f;
    // REAL_ACCEL 是归一化后的发动机牵引项，不等同于车辆瞬时加速度。
    // 与下方阻力项组合后，平路全油门约 28.5 s 到 38 km/h，并渐近 48 km/h。
    const float REAL_ACCEL = 2.78f;
    const float REAL_BRAKE = 8.0f;
    const float TRIGGER_DEADZONE = 0.2f;
    const float LINEAR_JERK_ACCEL = 0.4f;
    const float LINEAR_JERK_BRAKE = 2.5f;
    // 无驾驶输入时的履带/传动静阻力。小坡不自行滑动，坡度重力超过它后才开始溜车。
    // 单位与其他纵向力一致，均为真车等效 km/h/s。
    const float COAST_STATIC_RESIST_ACCEL = 0.8f;
    const float ROLL_RESIST_ACCEL = 0.8f;
    const float AIR_RESIST_COEFF = 0.00086f;

    // 预留的换挡模拟参数；当前底盘算法没有使用它们。
    const float SHIFT_12_REAL_KMH = 15.0f;
    const float SHIFT_23_REAL_KMH = 30.0f;
    const float SHIFT_CUT_FACTOR = 0.15f;
    const float SHIFT_MIN_THROTTLE = 0.2f;
    const uint32_t SHIFT_CUT_TIME_MS = 100;

    // ---------- 转向动力学 ----------
    const float YAW_SENSITIVITY = 25.0f;
    // 死区后采用线性/平方混合：保留中心微操，同时比纯平方映射更灵敏。
    const float TURN_INPUT_LINEAR_BLEND = 0.65f;
    // TN12 中心转向的独立履带等效速度上限。按约 2.9 m 真车履带中心距估算，
    // 3.3 km/h 对应稳态 360 度约 10 秒；首次从静止还会更慢一些。
    const float PIVOT_SPIN_MAX_KMH = 3.3f;
    const float SPEED_SENS_K = 0.08f;
    // 满转向不应像纵向整车加速那样缓慢：原地约0.55s建立到最大差速，
    // 行进转向通常约1s量级建立；回中/反向更快，避免转向拖尾。
    const float TURN_ACCEL_PIVOT = 6.0f;
    const float TURN_BRAKE_PIVOT = 12.0f;
    const float TURN_ACCEL_MOVING = 12.0f;
    const float TURN_BRAKE_MOVING = 14.0f;
    const float TURN_MOVING_BLEND_KMH = 8.0f;
    const uint32_t DIRECTION_CHANGE_HOLD_MS = 2000;
    const float DIRECTION_CHANGE_STOP_SPEED_KMH = 0.3f;

    // ---------- yaw 虚拟惯量 ----------
    const float YAW_INERTIA_ALPHA_TAU = 0.04f;
    const float YAW_INERTIA_ALPHA_DEADZONE_DPS2 = 25.0f;
    const float YAW_INERTIA_ALPHA_MAX_DPS2 = 500.0f;
    // 最大有效角加速度下约输出 6.7 PWM，并以 7 PWM 为独立硬限幅。
    const float YAW_INERTIA_PWM_GAIN = 0.014f;
    const float YAW_INERTIA_PWM_MAX = 7.0f;
    const float YAW_INERTIA_PWM_SIGN = 1.0f;

    // ---------- pitch 虚拟惯量 ----------
    const float V_INERTIA_ALPHA_TAU = 0.035f;
    const float V_INERTIA_ALPHA_DEADZONE_DPS2 = 18.0f;
    const float V_INERTIA_ALPHA_MAX_DPS2 = 450.0f;
    // 最大有效角加速度下约输出 5.2 PWM，并以 5 PWM 为独立硬限幅。
    const float V_INERTIA_PWM_GAIN = 0.012f;
    const float V_INERTIA_PWM_MAX = 5.0f;
    const float V_INERTIA_PWM_SIGN = 1.0f;

    // ---------- 坡度补偿 ----------
    const float SLOPE_GRAVITY_MAX = 12.0f;
    const float GRADE_PITCH_TAU = 0.35f;
    const float CHASSIS_PITCH_ACC_TAU_S = 0.050f;

    // ---------- 炮塔硬件 ----------
    const uint8_t SERVO_PIN = 15;
    const uint8_t FOC_PWM_A = 19, FOC_PWM_B = 18, FOC_PWM_C = 5;
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t AS5600_ANGLE_REG = 0x0C;

    // ---------- 炮塔 yaw 控制 ----------
    const float REAL_TURRET_VEL = 22.5f;
    const float YAW_OUTER_KP = 2.2f;
    const float YAW_OUTER_KD = 0.5f;
    const float YAW_INNER_KP = 0.18f;
    const float YAW_INNER_KI = 0.01f;
    const float YAW_INNER_KD = 0.002f;
    const float YAW_OUTER_RATE_MAX = 25.0f;
    const float YAW_VOLTAGE_MAX = 6.0f;
    const float YAW_CHASSIS_FF_GAIN = 0.6f;

    // ---------- 炮塔 yaw 堵转保护 ----------
    // 无电流采样时，用“高输出 + 大位置误差 + 编码器几乎不动”联合判断。
    // 三个条件连续满足确认时间后锁存停机，必须回中摇杆并重新按 A 才恢复。
    const float YAW_STALL_VOLTAGE_MIN = 5.0f;
    const float YAW_STALL_ERROR_MIN_DEG = 3.0f;
    const float YAW_STALL_MAX_TRAVEL_DEG = 0.5f;
    const uint32_t YAW_STALL_CONFIRM_MS = 500;
    const float YAW_STALL_REARM_JOY_MAX = 0.15f;

    // ---------- 炮管 pitch 控制 ----------
    const int IMU_CALIB_SAMPLES = 2000;
    const float GUN_PITCH_MIN = -10.0f;
    const float GUN_PITCH_MAX = 20.0f;
    const float SERVO_CMD_MIN = 45.0f;
    const float SERVO_CMD_MAX = 135.0f;
    const float PITCH_ACC_TAU = 0.6f;
    const float PITCH_STAB_KP = 70.0f;
    const float PITCH_STAB_KD = 0.35f;
    const float PITCH_CHASSIS_FF = 1.0f;
    const float PITCH_SERVO_RATE_DEADZONE_DPS = 1.5f;
    const float PITCH_RATE_CMD_MAX = 180.0f;

    // ---------- 后甲板避让 ----------
    const float REAR_DECK_CENTER_YAW = 180.0f;
    const float REAR_DECK_AVOID_START = 15.0f;
    const float REAR_DECK_AVOID_FULL = 10.0f;
    const float REAR_DECK_SAFE_PITCH = 0.0f;
    const float REAR_DECK_BLEND_EXP = 1.0f;
    const float TURRET_FRONT_SENSOR_OFFSET = 0.0f;
    const float TURRET_SENSOR_SIGN = 1.0f;

    // ---------- 健康检查/任务节奏 ----------
    // 两颗 MPU6050 与底盘稳定控制统一为 200Hz；对 44Hz DLPF 和模型机构已经足够，
    // 同时给蓝牙、遥测和底盘控制留出更充足的 Core 1 时间余量。
    const uint32_t IMU_UPDATE_US = 5000;
    const float IMU_MAX_DT = 0.05f;
    const float IMU_GYRO_SANITY_DPS = 550.0f;
    const uint16_t IMU_I2C_TIMEOUT_MS = 3;
    const uint8_t IMU_FAILURE_CONFIRM_SAMPLES = 3;
    const uint8_t IMU_RECOVERY_CONFIRM_SAMPLES = 10;
    const uint32_t IMU_STALE_US = 30000;
    // 加速度模长的宽松健康范围：排除 I2C 读失败常见的全零数据，
    // 同时保留履带冲击和 MPU6050 ±4g 量程内的动态余量。
    const float IMU_ACCEL_NORM_MIN_MPS2 = 1.0f;
    const float IMU_ACCEL_NORM_MAX_MPS2 = 50.0f;
    // 加速度越偏离 1g，越可能包含直线加减速或冲击，融合时应降低其权重。
    const float IMU_ACCEL_TRUST_FULL_DEVIATION_G = 0.05f;
    const float IMU_ACCEL_TRUST_ZERO_DEVIATION_G = 0.20f;
    // 加速度倾角和陀螺预测差异过大时，同样逐步拒绝本次加速度修正。
    const float IMU_ACCEL_INNOVATION_FULL_TRUST_DEG = 3.0f;
    const float IMU_ACCEL_INNOVATION_ZERO_TRUST_DEG = 12.0f;
    const uint32_t CONTROLLER_TIMEOUT_MS = 300;
    const uint32_t YAW_SENSOR_STALE_US = 50000;
    const uint32_t YAW_SENSOR_CHECK_US = 10000;
    const uint32_t VBAT_SAMPLE_MS = 100;
}
