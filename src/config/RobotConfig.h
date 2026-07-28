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
    const float VBAT_LPF = 0.1f;
    const float VBAT_WARN = 10.8f;             // 3S 低压预警
    const float VBAT_CUTOFF = 10.2f;           // 3S 低压截止
    const bool ENABLE_BATTERY_MONITOR = false; // false 时使用外置低压报警器

    // ---------- 底盘电机和 PWM ----------
    const uint8_t R_IN1 = 25, R_IN2 = 33, R_PWM = 32;
    const uint8_t L_IN1 = 14, L_IN2 = 27, L_PWM = 26;
    const uint8_t PWM_CH_R = 8, PWM_CH_L = 9;
    const uint32_t PWM_FREQ = 10000;
    const uint8_t PWM_RES = 8;
    const float MOTOR_PWM_DEADZONE = 1.0f;

    // ---------- 履带速度控制 ----------
    const float TRACK_STOP_DEADZONE_KMH = 0.10f;
    // 两级摩擦补偿只跨过电机/履带死区，剩余输出全部交给 PID。
    // 1.5V 空载启动电压在约 11.35V 母线下对应约 34 PWM；整车装配后需分别实测。
    const float TRACK_FF_KS_START_LEFT = 45.0f;
    const float TRACK_FF_KS_RUN_LEFT = 30.0f;
    const float TRACK_FF_KS_START_RIGHT = 45.0f;
    const float TRACK_FF_KS_RUN_RIGHT = 30.0f;
    const float TRACK_START_RELEASE_SPEED_KMH = 0.25f;
    const float TRACK_START_REENTER_SPEED_KMH = 0.08f;
    const uint32_t TRACK_START_RELEASE_CONFIRM_MS = 60;
    const uint32_t TRACK_START_REENTER_CONFIRM_MS = 100;
    const uint32_t TRACK_START_BLEND_DOWN_MS = 100;

    // PID 反馈。KD 默认关闭；确认 PI 仍有超调/振荡后再逐步增加。
    const float TRACK_PID_KP = 5.0f;
    const float TRACK_PID_KI = 0.8f;
    const float TRACK_PID_KD = 0.0f;
    const float TRACK_PID_I_MAX_PWM = 200.0f;
    const float TRACK_PID_D_FILTER_TAU_S = 0.080f;
    const float TRACK_PID_D_MAX_PWM = 30.0f;
    const float TRACK_EXTERNAL_PWM_MAX = 70.0f;

    // ---------- 履带堵转保护 ----------
    const float TRACK_STALL_TARGET_MIN_KMH = 1.5f;
    const float TRACK_STALL_PWM_MIN = 95.0f;
    const float TRACK_STALL_ACTUAL_MAX_KMH = 0.08f;
    const uint32_t TRACK_STALL_GRACE_MS = 1000;
    const float TRACK_STALL_CLEAR_TARGET_KMH = 0.12f;

    // ---------- 编码器 ----------
    // 编码器采用自适应计数窗口：高速时按最短周期刷新，低速时累计更多脉冲，
    // 最迟在最大窗口到达时给出一次估计。长时间无脉冲则明确判为停车。
    const uint32_t ENCODER_MIN_SAMPLE_US = 20000;
    const uint32_t ENCODER_MAX_SAMPLE_US = 100000;
    const uint8_t ENCODER_MIN_COUNTS = 3;
    const uint32_t ENCODER_STOP_TIMEOUT_US = 150000;
    const float ENCODER_CONTROL_FILTER_TAU_S = 0.060f;
    const float ENCODER_DISPLAY_FILTER_TAU_S = 0.120f;
    const float ENCODER_MAX_VALID_SPEED_KMH = 80.0f;
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
    // TN12 中心转向的独立履带等效速度上限。按约 2.9 m 真车履带中心距估算，
    // 3.3 km/h 对应稳态 360 度约 10 秒；首次从静止还会更慢一些。
    const float PIVOT_SPIN_MAX_KMH = 3.3f;
    const float SPEED_SENS_K = 0.08f;
    const float TURN_ACCEL_PIVOT = 1.0f;
    const float TURN_BRAKE_PIVOT = 8.0f;
    const float TURN_ACCEL_MOVING = 7.0f;
    const float TURN_BRAKE_MOVING = 12.0f;
    const float TURN_MOVING_BLEND_KMH = 8.0f;

    // ---------- yaw 虚拟惯量 ----------
    const float YAW_INERTIA_ALPHA_TAU = 0.04f;
    const float YAW_INERTIA_ALPHA_DEADZONE_DPS2 = 25.0f;
    const float YAW_INERTIA_ALPHA_MAX_DPS2 = 500.0f;
    const float YAW_INERTIA_PWM_GAIN = 0.05f;
    const float YAW_INERTIA_PWM_MAX = 28.0f;
    const float YAW_INERTIA_PWM_SIGN = 1.0f;

    // ---------- pitch 虚拟惯量 ----------
    const float V_INERTIA_ALPHA_TAU = 0.035f;
    const float V_INERTIA_ALPHA_DEADZONE_DPS2 = 18.0f;
    const float V_INERTIA_ALPHA_MAX_DPS2 = 450.0f;
    const float V_INERTIA_PWM_GAIN = 0.08f;
    const float V_INERTIA_PWM_MAX = 45.0f;
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
