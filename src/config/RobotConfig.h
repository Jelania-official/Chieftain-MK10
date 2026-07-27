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
    const char* const XBOX_MAC = "28:ea:0b:d9:0b:9f"; // Xbox 手柄蓝牙 MAC 地址
    const char* const DEBUG_BT_NAME = "ChieftainMK10-Debug"; // PC 调试模式下显示的蓝牙串口名称
    const uint32_t DEBUG_INPUT_TIMEOUT_MS = 300;             // PC 输入超过该时间未刷新就停车
    const uint32_t DEBUG_TELEMETRY_MS = 50;                  // PC 遥测发送周期，50ms = 20Hz

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
    const float TRACK_FF_KS_START = 110.0f;
    const float TRACK_FF_KS_RUN = 45.0f;
    const uint32_t TRACK_START_BOOST_MAX_MS = 220;
    const float TRACK_START_RELEASE_RATIO = 0.55f;
    const float TRACK_START_RELEASE_MIN_KMH = 0.18f;
    const float TRACK_FF_KV = 4.4f;
    const float TRACK_FF_KA = 0.8f;
    const float TRACK_FF_MAX_ACCEL = 80.0f;
    const float TRACK_FF_ACCEL_LPF = 0.25f;
    const float TRACK_PI_KP = 3.0f;
    const float TRACK_PI_KI = 0.25f;
    const float TRACK_PI_MAX_I = 80.0f;
    const float TRACK_PI_MAX_CORRECTION = 90.0f;
    const float TRACK_EXTERNAL_PWM_MAX = 70.0f;

    // ---------- 履带堵转保护 ----------
    const float TRACK_STALL_TARGET_MIN_KMH = 0.45f;
    const float TRACK_STALL_PWM_MIN = 95.0f;
    const float TRACK_STALL_ACTUAL_MAX_KMH = 0.08f;
    const uint32_t TRACK_STALL_GRACE_MS = 650;
    const float TRACK_STALL_CLEAR_TARGET_KMH = 0.12f;

    // ---------- 编码器 ----------
    const uint32_t ENCODER_SAMPLE_US = 5000;
    const float ENCODER_SPEED_LPF = 0.35f;
    const uint8_t R_ENCA = 35, R_ENCB = 34;
    const uint8_t L_ENCA = 23, L_ENCB = 4;

    // ---------- 1:32 真车速度映射 ----------
    const int SCALE = 32;
    const int ENCODER_PPR = 7;
    const int GEAR_RATIO = 59;
    const float WHEEL_D = 0.017f;
    const float RPM_TO_REAL_KMH = (WHEEL_D * PI * 60.0f / 1000.0f) * SCALE;

    // ---------- 纵向动力学 ----------
    const float REAL_V_MAX = 48.0f;
    const float REAL_V_REV_MAX = 11.0f;
    const float REAL_ACCEL = 2.5f;
    const float REAL_BRAKE = 8.0f;
    const float TRIGGER_DEADZONE = 0.2f;
    const float LINEAR_JERK_ACCEL = 0.4f;
    const float LINEAR_JERK_BRAKE = 2.5f;

    // 预留的换挡模拟参数；当前底盘算法没有使用它们。
    const float SHIFT_12_REAL_KMH = 15.0f;
    const float SHIFT_23_REAL_KMH = 30.0f;
    const float SHIFT_CUT_FACTOR = 0.15f;
    const float SHIFT_MIN_THROTTLE = 0.2f;
    const uint32_t SHIFT_CUT_TIME_MS = 100;

    // ---------- 转向动力学 ----------
    const float YAW_SENSITIVITY = 25.0f;
    const float SPEED_SENS_K = 0.08f;
    const float TURN_ACCEL_PIVOT = 3.0f;
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
    const float IMU_MAX_DT = 0.05f;
    const float IMU_GYRO_SANITY_DPS = 550.0f;
    const uint32_t CONTROLLER_TIMEOUT_MS = 300;
    const uint32_t YAW_SENSOR_STALE_US = 50000;
    const uint32_t YAW_SENSOR_CHECK_US = 10000;
    const uint32_t VBAT_SAMPLE_MS = 100;
}
