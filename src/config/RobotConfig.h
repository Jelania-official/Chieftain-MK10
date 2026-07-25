#pragma once

#include <Arduino.h>

// ==========================================
// 1. 全局配置参数 (酋长 MK10 1:32 仿真配置)
// ==========================================
namespace Config {
    // [通讯配置]
    const char* const XBOX_MAC = "28:ea:0b:d9:0b:9f"; // 手柄蓝牙MAC地址

    // [I2C 端口分配]
    const uint8_t I2C_FOC_SDA = 17, I2C_FOC_SCL = 16; // 磁编码器(AS5600)总线
    const uint8_t I2C_IMU_SDA = 22, I2C_IMU_SCL = 21; // 陀螺仪(MPU6050)总线

    // [电池电压检测] 使用 ADC1，避免和蓝牙/Wi-Fi 占用的 ADC2 冲突
    const uint8_t VBAT_ADC_PIN = 39;                // 仅输入，适合做电压采样
    const float VBAT_DIVIDER_R1 = 100000.0f;        // 上拉分压电阻，默认 100k
    const float VBAT_DIVIDER_R2 = 33000.0f;         // 下拉分压电阻，默认 33k
    const float VBAT_LPF = 0.1f;                    // 电压低通滤波系数
    const float VBAT_WARN = 10.8f;                  // 3S 低压预警阈值
    const float VBAT_CUTOFF = 10.2f;                // 3S 低压切断阈值
    const bool ENABLE_BATTERY_MONITOR = false;      // false: 使用外置低压报警器，禁用 VN 采样与软件低压切断

    // [底盘动力引脚]
    const uint8_t R_IN1 = 25, R_IN2 = 33, R_PWM = 32; // 右侧直流驱动
    const uint8_t L_IN1 = 14, L_IN2 = 27, L_PWM = 26; // 左侧直流驱动
    const uint8_t PWM_CH_R = 8, PWM_CH_L = 9;         // ESP32 硬件PWM通道
    const uint32_t PWM_FREQ = 10000;                  // 电机控制频率 10kHz
    const uint8_t PWM_RES = 8;                        // 8位分辨率 (0-255)
    const float MOTOR_PWM_DEADZONE = 1.0f;            // 输出小于该值时完全断开电机
    const float TRACK_STOP_DEADZONE_KMH = 0.10f;      // 履带速度死区；保留低速调试能力，避免 0.3km/h 被直接归零
    const float TRACK_FF_KS_START = 90.0f;            // 起步静摩擦前馈 PWM，用于破除履带静摩擦
    const float TRACK_FF_KS_RUN = 45.0f;              // 运行保持静摩擦前馈 PWM，转起来后避免持续猛冲
    const uint32_t TRACK_START_BOOST_MAX_MS = 220;    // 起步前馈最长持续时间，避免低速目标一直吃大 PWM
    const float TRACK_START_RELEASE_RATIO = 0.55f;    // 实测速度达到目标速度该比例后释放起步前馈
    const float TRACK_START_RELEASE_MIN_KMH = 0.18f;  // 释放阈值下限，避免编码器低速抖动导致过早释放
    const float TRACK_FF_KV = 4.4f;                   // 速度前馈 PWM/(km/h)
    const float TRACK_FF_KA = 0.8f;                   // 加速度前馈 PWM/(km/h/s)，调试初期偏保守，避免阶跃时猛冲
    const float TRACK_FF_MAX_ACCEL = 80.0f;           // 前馈使用的目标加速度限幅
    const float TRACK_FF_ACCEL_LPF = 0.25f;           // 目标加速度前馈低通，抑制死区边缘脉冲
    const float TRACK_PI_KP = 3.0f;                   // 履带速度 PI 比例项
    const float TRACK_PI_KI = 0.25f;                  // 履带速度 PI 积分项
    const float TRACK_PI_MAX_I = 80.0f;               // 履带速度 PI 积分限幅
    const float TRACK_PI_MAX_CORRECTION = 90.0f;      // PI 只做误差修正，主输出交给前馈
    const float TRACK_EXTERNAL_PWM_MAX = 70.0f;        // 外部惯量 PWM 总修正限幅
    const float TRACK_STALL_TARGET_MIN_KMH = 0.45f;   // 堵转检测最低目标速度；低速起步测试不触发
    const float TRACK_STALL_PWM_MIN = 95.0f;           // 堵转检测最低 PWM；小 PWM 推不动不算堵转
    const float TRACK_STALL_ACTUAL_MAX_KMH = 0.08f;   // 实际速度低于该值且持续高 PWM，认为可能堵转
    const uint32_t TRACK_STALL_GRACE_MS = 650;         // 堵转判定持续时间，给正常起步留出余量
    const float TRACK_STALL_CLEAR_TARGET_KMH = 0.12f; // 松开目标到该速度以下后清除堵转锁存
    const uint32_t ENCODER_SAMPLE_US = 5000;          // 编码器测速周期，对齐 200Hz 控制环
    const float ENCODER_SPEED_LPF = 0.35f;            // 编码器速度低通，降低低速量化抖动

    // [编码器引脚] - 34/35需外部上拉电阻(10K\0805)
    const uint8_t R_ENCA = 35, R_ENCB = 34;
    const uint8_t L_ENCA = 23, L_ENCB = 4; 

    // --- 酋长 MK10 物理数据映射 (1:32) ---
    const int SCALE = 32;                   // 比例尺
    const int ENCODER_PPR = 7;              // 编码器线数
    const int GEAR_RATIO = 59;              // 减速箱变比
    const float WHEEL_D = 0.017f;           // 主动轮直径 17mm (0.017m)

    // [转换系数]: RPM 映射为真车等效 km/h
    // 计算: (RPM * D*PI * 60分钟 / 1000米) * 32倍比例
    const float RPM_TO_REAL_KMH = (WHEEL_D * PI * 60.0f / 1000.0f) * SCALE; 

    const float REAL_V_MAX = 48.0f;         // 真车最大前进速度 (km/h)
    const float REAL_V_REV_MAX = 11.0f;     // 真车最大倒车速度 (km/h)
    
    // [惯性模拟]: 基于 13.3 hp/t 沉重感
    // 现实中酋长MK10加速较慢，设定加速度约为 2.5 km/h每秒
    const float REAL_ACCEL = 2.5f;          
    const float REAL_BRAKE = 8.0f;          // 刹车减速度 (km/h/s)
    const float TRIGGER_DEADZONE = 0.2f;    // 扳机触发阈值
    // [动力学精细调校 - 3阶导数 Jerk 限制]
    // 前进/后退推力爬升限制 (km/h/s²)
    const float LINEAR_JERK_ACCEL = 0.4f;  // 模拟 L60 引擎缓慢的扭矩堆积
    const float LINEAR_JERK_BRAKE = 2.5f;  // 刹车Jerk更大，保证制动响应同时防冲击
    const float SHIFT_12_REAL_KMH = 15.0f; // TN12 1->2 挡模拟速度点
    const float SHIFT_23_REAL_KMH = 30.0f; // TN12 2->3 挡模拟速度点
    const float SHIFT_CUT_FACTOR = 0.15f;  // 换挡时剩余动力比例
    const float SHIFT_MIN_THROTTLE = 0.2f; // 进入换挡模拟的最小油门
    const uint32_t SHIFT_CUT_TIME_MS = 100; // 换挡动力中断时间

    // [横向动力学与随速感应]
    const float YAW_SENSITIVITY = 25.0f;   // 最大差速分量
    const float SPEED_SENS_K = 0.08f;      // 随速衰减系数 (越高，高速时方向盘越“重”)
    const float TURN_ACCEL_PIVOT = 3.0f;   // 原地转向差速建立速度 (km/h/s)，像另一组油门慢慢建立
    const float TURN_BRAKE_PIVOT = 8.0f;   // 原地转向摩擦/反向刹车回正速度 (km/h/s)
    const float TURN_ACCEL_MOVING = 7.0f;  // 行进中转向差速建立速度 (km/h/s)
    const float TURN_BRAKE_MOVING = 12.0f; // 行进中转向松杆回正速度 (km/h/s)
    const float TURN_MOVING_BLEND_KMH = 8.0f; // 从原地转向仿真过渡到行进双流分配的速度尺度
    const float YAW_INERTIA_ALPHA_TAU = 0.04f;          // yaw 角加速度低通时间常数
    const float YAW_INERTIA_ALPHA_DEADZONE_DPS2 = 25.0f;// yaw 角加速度死区
    const float YAW_INERTIA_ALPHA_MAX_DPS2 = 500.0f;    // yaw 角加速度限幅
    const float YAW_INERTIA_PWM_GAIN = 0.05f;           // yaw 虚拟惯量增益：PWM/(deg/s^2)
    const float YAW_INERTIA_PWM_MAX = 28.0f;            // yaw 虚拟惯量最大差速 PWM
    const float YAW_INERTIA_PWM_SIGN = 1.0f;            // 实测方向反了就改成 -1.0f

    // [虚拟旋转惯量]
    // 用底盘俯仰角加速度生成反向电机力矩：T_virtual = -I_virtual * alpha。
    const float V_INERTIA_ALPHA_TAU = 0.035f;         // 角加速度低通时间常数，单位 s
    const float V_INERTIA_ALPHA_DEADZONE_DPS2 = 18.0f;// 角加速度死区，抑制 IMU 微分底噪
    const float V_INERTIA_ALPHA_MAX_DPS2 = 450.0f;    // 角加速度限幅
    const float V_INERTIA_PWM_GAIN = 0.08f;           // 虚拟惯量增益：PWM/(deg/s^2)
    const float V_INERTIA_PWM_MAX = 45.0f;            // 虚拟惯量最大 PWM 修正
    const float V_INERTIA_PWM_SIGN = 1.0f;            // 实测方向反了就改成 -1.0f

    // [环境阻力精细调校]
    // 假设在垂直90度时，重力带来的最大加速度。数值越大，爬坡越吃力，下坡溜得越快。
    const float SLOPE_GRAVITY_MAX = 12.0f; // 单位：km/h/s
    const float GRADE_PITCH_TAU = 0.35f;   // 坡度角低通时间常数，滤掉起步点头/颠簸高频

    // [炮塔与双稳]
    const uint8_t SERVO_PIN = 15;           // 俯仰舵机引脚
    const uint8_t FOC_PWM_A = 19, FOC_PWM_B = 18, FOC_PWM_C = 5; // 无刷驱动引脚
    const float REAL_TURRET_VEL = 22.5f;    // 真车转塔速度 (deg/s)
    const float YAW_OUTER_KP = 2.2f;
    const float YAW_OUTER_KD = 0.5f;
    const float YAW_INNER_KP = 0.18f;
    const float YAW_INNER_KI = 0.01f;
    const float YAW_INNER_KD = 0.002f;
    const float YAW_OUTER_RATE_MAX = 25.0f; // yaw 外环最大目标角速度，略高于真车满速避免追不上手柄目标
    const float YAW_VOLTAGE_MAX = 6.0f;     // 给 SimpleFOC torque/voltage 目标的总限幅
    const float YAW_CHASSIS_FF_GAIN = 0.6f; // 底盘 yaw 角速度前馈增益
    const int IMU_CALIB_SAMPLES = 2000;     // IMU 启动校准采样次数 (2000次约4秒)
    const float GUN_PITCH_MIN = -10.0f;     // 正常最低俯角 (deg)
    const float GUN_PITCH_MAX = 20.0f;      // 最高仰角 (deg)
    const float SERVO_CMD_MIN = 45.0f;      // 舵机安全命令下限
    const float SERVO_CMD_MAX = 135.0f;     // 舵机安全命令上限
    const float PITCH_ACC_TAU = 0.6f;       // 炮管 pitch 互补滤波中加速度计纠漂时间常数
    const float PITCH_STAB_KP = 70.0f;      // pitch 角度误差到目标角速度的比例增益
    const float PITCH_STAB_KD = 0.35f;      // 炮管自身 pitch 角速度阻尼
    const float PITCH_CHASSIS_FF = 1.0f;    // 底盘 pitch 角速度前馈补偿
    const float PITCH_SERVO_RATE_DEADZONE_DPS = 1.5f; // 小于该角速度命令时不刷新舵机，降低嗡嗡抖动
    const float REAR_DECK_CENTER_YAW = 180.0f; // 炮塔正后方相对角 (deg)
    const float REAR_DECK_AVOID_START = 15.0f; // 距正后方左右15度开始抬炮
    const float REAR_DECK_AVOID_FULL = 10.0f;  // 距正后方左右10度内完全抬到安全俯角
    const float REAR_DECK_SAFE_PITCH = 0.0f;   // 发动机舱上方允许的最低俯角 (deg)
    const float REAR_DECK_BLEND_EXP = 1.0f;    // 1.0为线性，>1更晚抬，<1更早抬
    const float TURRET_FRONT_SENSOR_OFFSET = 0.0f; // AS5600 读数对应车体正前的机械角 (deg)
    const float TURRET_SENSOR_SIGN = 1.0f;         // 方向修正: 1.0正常, -1.0反向
    const float IMU_MAX_DT = 0.05f;               // IMU 单次采样最大有效周期 (s)
    const float IMU_GYRO_SANITY_DPS = 550.0f;     // IMU 角速度合理上限 (deg/s)
    const float PITCH_RATE_CMD_MAX = 180.0f;      // 俯仰稳定最大指令角速度 (deg/s)
    const uint32_t CONTROLLER_TIMEOUT_MS = 300;   // 手柄失联超时
    const uint32_t YAW_SENSOR_STALE_US = 50000;   // AS5600 缓存有效期
    const uint32_t YAW_SENSOR_CHECK_US = 10000;   // AS5600 主动健康探测周期
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t AS5600_ANGLE_REG = 0x0C;
    const uint32_t VBAT_SAMPLE_MS = 100;          // 电池电压采样周期
}

