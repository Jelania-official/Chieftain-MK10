#include <Arduino.h>
#include <Wire.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <SimpleFOC.h>
#include <driver/pcnt.h>

// ==========================================
// 0. 调试控制中心 (Debug Control Center)
// ==========================================
#define GLOBAL_DEBUG 1 

enum DebugChannel { NONE, CHASSIS_ONLY, TURRET_ONLY, IMU_RAW, ALL };
DebugChannel currentChannel = CHASSIS_ONLY; // <-- [在此切换频道]

#if GLOBAL_DEBUG
  #define LOG(ch, fmt, ...) if(currentChannel == ch || currentChannel == ALL) Serial.printf(fmt, ##__VA_ARGS__)
  #define LOG_ALWAYS(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define LOG(ch, fmt, ...)
  #define LOG_ALWAYS(fmt, ...)
#endif

// ==========================================
// 1. 全局配置参数 (酋长 MK10 1:32 仿真配置)
// ==========================================
namespace Config {
    // [通讯配置]
    const char* XBOX_MAC = "28:ea:0b:d9:0b:9f"; // 手柄蓝牙MAC地址

    // [I2C 端口分配]
    const uint8_t I2C_FOC_SDA = 16, I2C_FOC_SCL = 17; // 磁编码器(AS5600)总线
    const uint8_t I2C_IMU_SDA = 21, I2C_IMU_SCL = 22; // 陀螺仪(MPU6050)总线

    // [电池电压检测] 使用 ADC1，避免和蓝牙/Wi-Fi 占用的 ADC2 冲突
    const uint8_t VBAT_ADC_PIN = 39;                // 仅输入，适合做电压采样
    const float VBAT_DIVIDER_R1 = 100000.0f;        // 上拉分压电阻，默认 100k
    const float VBAT_DIVIDER_R2 = 33000.0f;         // 下拉分压电阻，默认 33k
    const float VBAT_LPF = 0.1f;                    // 电压低通滤波系数
    const float VBAT_WARN = 10.8f;                  // 3S 低压预警阈值
    const float VBAT_CUTOFF = 10.2f;                // 3S 低压切断阈值

    // [底盘动力引脚]
    const uint8_t R_IN1 = 25, R_IN2 = 33, R_PWM = 32; // 右侧直流驱动
    const uint8_t L_IN1 = 26, L_IN2 = 27, L_PWM = 14; // 左侧直流驱动
    const uint8_t PWM_CH_R = 8, PWM_CH_L = 9;         // ESP32 硬件PWM通道
    const uint32_t PWM_FREQ = 10000;                  // 电机控制频率 10kHz
    const uint8_t PWM_RES = 8;                        // 8位分辨率 (0-255)
    const float MOTOR_PWM_DEADZONE = 1.0f;            // 输出小于该值时完全断开电机
    const float TRACK_STOP_DEADZONE_KMH = 0.5f;       // 履带速度死区
    const float TRACK_FF_KS = 32.0f;                  // 静摩擦前馈 PWM
    const float TRACK_FF_KV = 4.4f;                   // 速度前馈 PWM/(km/h)
    const float TRACK_FF_KA = 2.2f;                   // 加速度前馈 PWM/(km/h/s)
    const float TRACK_FF_KSLOPE = 18.0f;              // 坡度保持前馈 PWM
    const float TRACK_FF_MAX_ACCEL = 80.0f;           // 前馈使用的目标加速度限幅
    const float TRACK_FF_ACCEL_LPF = 0.25f;           // 目标加速度前馈低通，抑制死区边缘脉冲
    const float TRACK_PI_KP = 2.0f;                   // 履带速度 PI 比例项
    const float TRACK_PI_KI = 0.25f;                  // 履带速度 PI 积分项
    const float TRACK_PI_MAX_I = 80.0f;               // 履带速度 PI 积分限幅
    const float TRACK_PI_MAX_CORRECTION = 90.0f;      // PI 只做误差修正，主输出交给前馈
    const float TRACK_EXTERNAL_PWM_MAX = 70.0f;        // 外部惯量 PWM 总修正限幅
    const uint32_t ENCODER_SAMPLE_US = 5000;          // 编码器测速周期，对齐 200Hz 控制环
    const float ENCODER_SPEED_LPF = 0.35f;            // 编码器速度低通，降低低速量化抖动

    // [编码器引脚] - 34/35需外部上拉电阻(10K\0805)
    const uint8_t R_ENCA = 23, R_ENCB = 4;
    const uint8_t L_ENCA = 35, L_ENCB = 34; 

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
    const uint8_t FOC_PWM_A = 5, FOC_PWM_B = 19, FOC_PWM_C = 18; // 无刷驱动引脚
    const float REAL_TURRET_VEL = 22.5f;    // 真车转塔速度 (deg/s)
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

portMUX_TYPE turretStateMux = portMUX_INITIALIZER_UNLOCKED;

// ==========================================
// 2. 基础控制算法 (PID)
// ==========================================
// 最底层的通用 PID，给炮塔级联控制使用。
class CustomPID {
public:
    float kp, ki, kd, maxOut, maxI;
    float integral = 0, prevError = 0;
    CustomPID(float p, float i, float d, float mi, float mo) 
        : kp(p), ki(i), kd(d), maxI(mi), maxOut(mo) {}

    float calculate(float target, float actual, float dt) {
        if (dt <= 0.0f) dt = 0.001f;
        float error = target - actual;
        integral += error * dt;
        integral = constrain(integral, -maxI, maxI);
        float derivative = (error - prevError) / dt;
        prevError = error;
        return constrain(kp * error + ki * integral + kd * derivative, -maxOut, maxOut);
    }
    void reset() { integral = 0; prevError = 0; }
};

// 炮塔 yaw 用的是位置环套速度环的级联结构，外环给目标角速度，内环出最终驱动量。
class CascadePID {
public:
    CustomPID outer; CustomPID inner; float ff_gain;
    CascadePID(CustomPID out, CustomPID in, float ff = 0.0f) : outer(out), inner(in), ff_gain(ff) {}
    float calculate(float posRef, float posFdb, float velFdb, float chassisVel, float dt) {
        float targetVel = outer.calculate(posRef, posFdb, dt);
        float innerOut = inner.calculate(targetVel, velFdb, dt);
        return constrain(innerOut + (ff_gain * chassisVel),
                         -Config::YAW_VOLTAGE_MAX,
                         Config::YAW_VOLTAGE_MAX);
    }
    void reset() { outer.reset(); inner.reset(); }
};

// 履带速度控制：前馈承担主要 PWM，PI 只修正编码器反馈误差。
class TrackVelocityController {
private:
    float integral = 0.0f;
    float lastTarget = 0.0f;
    float filteredTargetAccel = 0.0f;
    bool wasTargetActive = false;

public:
    float calculate(float target, float actual, float dt, float pitchAngleDeg, float externalPwm) {
        if (dt <= 0.0f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;
        externalPwm = constrain(externalPwm,
                                -Config::TRACK_EXTERNAL_PWM_MAX,
                                Config::TRACK_EXTERNAL_PWM_MAX);

        if (abs(target) < Config::TRACK_STOP_DEADZONE_KMH &&
            abs(actual) < Config::TRACK_STOP_DEADZONE_KMH) {
            reset();
            return externalPwm;
        }

        bool targetActive = abs(target) >= Config::TRACK_STOP_DEADZONE_KMH;
        float targetAccel = 0.0f;
        if (targetActive && wasTargetActive) {
            targetAccel = constrain((target - lastTarget) / dt,
                                    -Config::TRACK_FF_MAX_ACCEL,
                                    Config::TRACK_FF_MAX_ACCEL);
        }
        filteredTargetAccel += Config::TRACK_FF_ACCEL_LPF * (targetAccel - filteredTargetAccel);
        lastTarget = target;
        wasTargetActive = targetActive;

        float ff = 0.0f;
        if (targetActive) {
            float dir = (target > 0.0f) ? 1.0f : -1.0f;
            ff = (Config::TRACK_FF_KS * dir) +
                 (Config::TRACK_FF_KV * target) +
                 (Config::TRACK_FF_KA * filteredTargetAccel) +
                 (Config::TRACK_FF_KSLOPE * sin(pitchAngleDeg * DEG_TO_RAD));
        }

        float error = target - actual;
        if (abs(target) < Config::TRACK_STOP_DEADZONE_KMH) {
            integral *= 0.9f;
        } else {
            integral += error * dt;
        }
        integral = constrain(integral, -Config::TRACK_PI_MAX_I, Config::TRACK_PI_MAX_I);

        float correction = constrain((Config::TRACK_PI_KP * error) +
                                     (Config::TRACK_PI_KI * integral),
                                     -Config::TRACK_PI_MAX_CORRECTION,
                                     Config::TRACK_PI_MAX_CORRECTION);

        return constrain(ff + correction + externalPwm, -255.0f, 255.0f);
    }

    void reset() {
        integral = 0.0f;
        lastTarget = 0.0f;
        filteredTargetAccel = 0.0f;
        wasTargetActive = false;
    }
};

// ==========================================
// 3. 硬件抽象 (电机与编码器)
// ==========================================
// TB6612 有刷电机抽象：输入范围约定为 -255~255，符号代表方向。
class DCMotor {
private:
    uint8_t in1, in2, pwmPin, pwmCh; bool isLeft;
public:
    DCMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, uint8_t ch, bool left = false)
        : in1(pin1), in2(pin2), pwmPin(pwmPin), pwmCh(ch), isLeft(left) {}
    void init() {
        pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
        ledcSetup(pwmCh, Config::PWM_FREQ, Config::PWM_RES);
        ledcAttachPin(pwmPin, pwmCh);
    }
    void drive(float output) {
        // 1. 限制范围并处理死区
        float absOut = constrain(abs(output), 0.0f, 255.0f);
        if (absOut < Config::MOTOR_PWM_DEADZONE) {
            // 完全停止：不仅 PWM 给 0，电机引脚也要拉低，防止发热
            digitalWrite(in1, LOW); 
            digitalWrite(in2, LOW); 
            ledcWrite(pwmCh, 0);
            return;
        }

        // 2. 物理驱动方向逻辑；静摩擦补偿由履带前馈控制器负责。
        if (output > 0) {
            digitalWrite(in1, isLeft ? HIGH : LOW); 
            digitalWrite(in2, isLeft ? LOW : HIGH);
            ledcWrite(pwmCh, (uint32_t)absOut);
        } else {
            digitalWrite(in1, isLeft ? LOW : HIGH); 
            digitalWrite(in2, isLeft ? HIGH : LOW);
            ledcWrite(pwmCh, (uint32_t)absOut);
        }
    }
};

// N20 编码器抽象：用 ESP32 的 PCNT 外设做 AB 相计数，再换算成真车等效速度。
class CustomEncoder {
private:
    uint8_t pinA, pinB; pcnt_unit_t unit; uint32_t lastSampleUs = 0; float lastSpeed = 0.0f;
public:
    CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit) : pinA(pinA), pinB(pinB), unit(p_unit) {}
    void init() {
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);
        pcnt_config_t cfg = {};
        cfg.pulse_gpio_num = pinA; cfg.ctrl_gpio_num = pinB;
        cfg.channel = PCNT_CHANNEL_0; cfg.unit = unit;
        cfg.pos_mode = PCNT_COUNT_INC; cfg.neg_mode = PCNT_COUNT_DEC;
        cfg.lctrl_mode = PCNT_MODE_KEEP; cfg.hctrl_mode = PCNT_MODE_REVERSE;
        cfg.counter_h_lim = 32767; cfg.counter_l_lim = -32768;
        esp_err_t err = pcnt_unit_config(&cfg);
        if (err != ESP_OK) {
            LOG_ALWAYS("!!! PCNT config failed: unit=%d err=%d\n", (int)unit, (int)err);
        }
        pcnt_set_filter_value(unit, 1000);
        pcnt_filter_enable(unit);
        pcnt_counter_pause(unit); pcnt_counter_clear(unit); pcnt_counter_resume(unit);
        lastSampleUs = micros();
    }
    float getRealSpeedKMH() {
        uint32_t now = micros();
        uint32_t dtUs = now - lastSampleUs;
        if (dtUs >= Config::ENCODER_SAMPLE_US) {
            int16_t count = 0;
            pcnt_counter_pause(unit);
            pcnt_get_counter_value(unit, &count);
            pcnt_counter_clear(unit);
            pcnt_counter_resume(unit);

            float rpm = (count / (float)Config::ENCODER_PPR / Config::GEAR_RATIO) * (60000000.0f / dtUs);
            float measuredSpeed = rpm * Config::RPM_TO_REAL_KMH; // 输出真车等效速度
            lastSpeed += Config::ENCODER_SPEED_LPF * (measuredSpeed - lastSpeed);
            if (count == 0 && abs(lastSpeed) < Config::TRACK_STOP_DEADZONE_KMH) lastSpeed = 0.0f;
            lastSampleUs = now;
        }
        return lastSpeed;
    }
};

// ==========================================
// 4. 底盘系统 (真车物理模拟)
// ==========================================
// 单侧履带 = 电机 + 编码器 + 前馈/PI 速度控制器。
class TankTrack {
public:
    DCMotor motor; CustomEncoder encoder; TrackVelocityController controller;
    float currentSpeed = 0, targetSpeed = 0;
    TankTrack(DCMotor m, CustomEncoder e) : motor(m), encoder(e) {}
    void init() { motor.init(); encoder.init(); }
    void update(float target, float dt, float pitchAngleDeg, float externalPwm) {
        targetSpeed = target;
        currentSpeed = encoder.getRealSpeedKMH();
        if (abs(targetSpeed) < Config::TRACK_STOP_DEADZONE_KMH) targetSpeed = 0;
        motor.drive(controller.calculate(targetSpeed, currentSpeed, dt, pitchAngleDeg, externalPwm));
    }
    void stop() { targetSpeed = 0; controller.reset(); motor.drive(0); }
};

// 专为重型内燃机设计的油门平滑器：踩油门迟滞(模拟涡轮/转速爬升)，松油门瞬间切断
class ThrottleSmoother {
private:
    float current_val = 0.0f;
    float rise_rate; // 踩下时的爬升速度 (对应引擎迟滞)
    float fall_rate; // 松开时的下降速度 (几乎瞬间)

public:
    ThrottleSmoother(float rise, float fall) : rise_rate(rise), fall_rate(fall) {}

    float update(float target, float dt) {
        if (target > current_val) {
            // 踩油门：缓慢建立扭矩
            current_val = min(current_val + rise_rate * dt, target);
        } else {
            // 松油门/刹车：极其迅速地卸载扭矩
            current_val = max(current_val - fall_rate * dt, target);
        }
        return current_val;
    }
    void reset() { current_val = 0.0f; }
};

// 底盘总控：把手柄输入解释成“发动机推力/刹车/阻力/坡度”的合力，再映射到双履带。
class TankChassis {
private:
    TankTrack rightTrack, leftTrack;
    float v_real = 0, spinV = 0; 

    // 引擎油门建立缓慢 (0.8)，但松油门切断极快 (10.0)
    ThrottleSmoother engineSmoother; 
    
    // 刹车液压建立较快 (3.0)，松开也快 (10.0)
    ThrottleSmoother brakeSmoother;  

    float longitudinalAccel = 0.0f;
    float gradePitchDeg = 0.0f;
    bool gradePitchReady = false;
    float lastPitchRateDeg = 0.0f;
    float filteredPitchAlpha = 0.0f;
    bool pitchRateReady = false;
    float lastYawRateDeg = 0.0f;
    float filteredYawAlpha = 0.0f;
    bool yawRateReady = false;

    float moveToward(float current, float target, float maxDelta) {
        float delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }

    float calculateVirtualInertiaPwm(float pitchRateDeg, float dt) {
        if (dt <= 0.0f) return 0.0f;
        if (!pitchRateReady) {
            lastPitchRateDeg = pitchRateDeg;
            pitchRateReady = true;
            return 0.0f;
        }

        float rawAlpha = (pitchRateDeg - lastPitchRateDeg) / dt;
        lastPitchRateDeg = pitchRateDeg;
        rawAlpha = constrain(rawAlpha,
                             -Config::V_INERTIA_ALPHA_MAX_DPS2,
                             Config::V_INERTIA_ALPHA_MAX_DPS2);

        float alphaBlend = dt / (Config::V_INERTIA_ALPHA_TAU + dt);
        filteredPitchAlpha += alphaBlend * (rawAlpha - filteredPitchAlpha);

        float effectiveAlpha = filteredPitchAlpha;
        if (abs(effectiveAlpha) < Config::V_INERTIA_ALPHA_DEADZONE_DPS2) {
            effectiveAlpha = 0.0f;
        } else {
            effectiveAlpha = copysign(abs(effectiveAlpha) - Config::V_INERTIA_ALPHA_DEADZONE_DPS2,
                                      effectiveAlpha);
        }

        return constrain(-Config::V_INERTIA_PWM_SIGN *
                         Config::V_INERTIA_PWM_GAIN *
                         effectiveAlpha,
                         -Config::V_INERTIA_PWM_MAX,
                         Config::V_INERTIA_PWM_MAX);
    }

    float calculateYawInertiaPwm(float yawRateDeg, float dt) {
        if (dt <= 0.0f) return 0.0f;
        if (!yawRateReady) {
            lastYawRateDeg = yawRateDeg;
            yawRateReady = true;
            return 0.0f;
        }

        float rawAlpha = (yawRateDeg - lastYawRateDeg) / dt;
        lastYawRateDeg = yawRateDeg;
        rawAlpha = constrain(rawAlpha,
                             -Config::YAW_INERTIA_ALPHA_MAX_DPS2,
                             Config::YAW_INERTIA_ALPHA_MAX_DPS2);

        float alphaBlend = dt / (Config::YAW_INERTIA_ALPHA_TAU + dt);
        filteredYawAlpha += alphaBlend * (rawAlpha - filteredYawAlpha);

        float effectiveAlpha = filteredYawAlpha;
        if (abs(effectiveAlpha) < Config::YAW_INERTIA_ALPHA_DEADZONE_DPS2) {
            effectiveAlpha = 0.0f;
        } else {
            effectiveAlpha = copysign(abs(effectiveAlpha) - Config::YAW_INERTIA_ALPHA_DEADZONE_DPS2,
                                      effectiveAlpha);
        }

        return constrain(-Config::YAW_INERTIA_PWM_SIGN *
                         Config::YAW_INERTIA_PWM_GAIN *
                         effectiveAlpha,
                         -Config::YAW_INERTIA_PWM_MAX,
                         Config::YAW_INERTIA_PWM_MAX);
    }

    float updateGradePitch(float pitchAngleDeg, float dt) {
        if (!gradePitchReady) {
            gradePitchDeg = pitchAngleDeg;
            gradePitchReady = true;
            return gradePitchDeg;
        }

        float gradeBlend = dt / (Config::GRADE_PITCH_TAU + dt);
        gradePitchDeg += gradeBlend * (pitchAngleDeg - gradePitchDeg);
        return gradePitchDeg;
    }

public:
    TankChassis() : 
        rightTrack(DCMotor(Config::R_IN1, Config::R_IN2, Config::R_PWM, Config::PWM_CH_R, false),
                   CustomEncoder(Config::R_ENCA, Config::R_ENCB, PCNT_UNIT_0)),
        leftTrack (DCMotor(Config::L_IN1, Config::L_IN2, Config::L_PWM, Config::PWM_CH_L, true),
                   CustomEncoder(Config::L_ENCA, Config::L_ENCB, PCNT_UNIT_1)),
        engineSmoother(0.8f, 10.0f),  // 参数可调：0.8表示油门踩到底需1秒多建立全扭矩
        brakeSmoother(4.0f, 10.0f)    // 参数可调：刹车建立很快
    {}

    void init() { rightTrack.init(); leftTrack.init(); }

    // 这里的速度单位统一用“真车等效 km/h”，这样比例映射和参数调校更直观。
    void processKinematics(float triggerL, float triggerR, float joyX, float dt, float currentPitchRate, float currentYawRate, float pitchAngle) {
        float gradePitch = updateGradePitch(pitchAngle, dt);

        // ==========================================
        // 纵向动力学 (游戏式 RT 前进 / LT 倒车)
        // ==========================================
        
        // 1. 获取平滑后的油门与刹车输入 (0.0 ~ 1.0)
        // trigger 做了平方处理，模拟摇杆的指数曲线，增加微操手感
        float forwardInput = (triggerR > Config::TRIGGER_DEADZONE) ? (triggerR * triggerR) : 0.0f;
        float reverseInput = (triggerL > Config::TRIGGER_DEADZONE) ? (triggerL * triggerL) : 0.0f;
        float driveInput = forwardInput - reverseInput;
        float desiredDir = (driveInput > 0.001f) ? 1.0f : ((driveInput < -0.001f) ? -1.0f : 0.0f);

        float raw_throttle = abs(driveInput);
        float raw_brake = 0.0f;

        // 当请求方向与当前运动方向相反时，先把该输入当成刹车；接近停稳后再自动换向。
        bool brakingToReverse = (v_real > 0.3f && desiredDir < 0.0f) || (v_real < -0.3f && desiredDir > 0.0f);
        if (brakingToReverse) {
            raw_brake = raw_throttle;
            raw_throttle = 0.0f;
            engineSmoother.reset();
        }

        float eff_throttle = engineSmoother.update(raw_throttle, dt);
        float eff_brake    = brakeSmoother.update(raw_brake, dt);

        // 2. 计算各独立作用力（换算为加速度，单位 km/h/s）
        float force_engine = eff_throttle * Config::REAL_ACCEL * desiredDir;

        float force_brake = eff_brake * Config::REAL_BRAKE;

        // 阻力：始终与当前运动方向相反
        float resDir = (v_real > 0.1f) ? 1.0f : ((v_real < -0.1f) ? -1.0f : (v_real / 0.1f));
        float airResist = 0.001f * v_real * v_real;
        float rollResist = 0.8f * constrain(abs(v_real)/1.0f, 0.0f, 1.0f); // 滚动阻力
        float force_resist = -(airResist + rollResist) * resDir;

        // 坡度重力分量
        float force_slope = -Config::SLOPE_GRAVITY_MAX * sin(gradePitch * DEG_TO_RAD);

        // 3. 施加刹车力的方向判定
        // 刹车力是没有主动方向的，它只能去“抵消”当前的速度。
        if (v_real > 0.1f) {
            force_brake = -force_brake; // 车往前走，刹车向后拉
        } else if (v_real < -0.1f) {
            force_brake = force_brake;  // 车往后走，刹车向前拉
        } else {
            // 速度极小时，重力可能导致溜车。如果刹车踩得够死，静摩擦力接管，抵消所有外力。
            if (eff_brake > 0.1f && abs(force_engine + force_slope) < (force_brake)) {
                force_engine = 0; force_slope = 0; force_resist = 0; force_brake = 0; 
                v_real = 0; // 死死刹停
            } else {
                force_brake = 0;
            }
        }

        // 4. 净力求和
        float a_net = force_engine + force_brake + force_resist + force_slope;

        // 5. jerk 限制后的实际纵向加速度。持续加速时车身持续抬头/低头，而不是只在起步瞬间响应。
        bool accelerationBuilds = (a_net * longitudinalAccel >= 0.0f) && (abs(a_net) > abs(longitudinalAccel));
        float jerkLimit = accelerationBuilds ? Config::LINEAR_JERK_ACCEL : Config::LINEAR_JERK_BRAKE;
        longitudinalAccel = moveToward(longitudinalAccel, a_net, jerkLimit * dt);
        if (abs(a_net) < 0.02f && abs(longitudinalAccel) < 0.02f) longitudinalAccel = 0.0f;

        // 6. 积分计算最终纵向速度
        v_real += longitudinalAccel * dt;

        // 极限速度钳制
        v_real = constrain(v_real, -Config::REAL_V_REV_MAX, Config::REAL_V_MAX);
        if ((v_real >= Config::REAL_V_MAX && longitudinalAccel > 0.0f) ||
            (v_real <= -Config::REAL_V_REV_MAX && longitudinalAccel < 0.0f)) {
            longitudinalAccel = 0.0f;
        }

        // ==========================================
        // 横向动力学：摇杆直接给目标差速，真实车体的转动惯量由 yaw IMU 反馈补偿。
        // ==========================================
        
        float joyX_adj = (abs(joyX) < 0.12f) ? 0 : joyX; // 死区
        float joyX_squared = copysign(joyX_adj * joyX_adj, joyX_adj); 
        
        // 随速感应灵敏度
        float dynamic_sens = Config::YAW_SENSITIVITY / (1.0f + abs(v_real) * Config::SPEED_SENS_K);
        
        // 目标自转速度
        float target_spin_v = joyX_squared * dynamic_sens;
        spinV = target_spin_v;

        // ==========================================
        // 双流耦合输出 (保持不变)
        // ==========================================
        float Lv_tgt = v_real + spinV;
        float Rv_tgt = v_real - spinV;
        float pitchInertiaPwm = calculateVirtualInertiaPwm(currentPitchRate, dt);
        float yawInertiaPwm = calculateYawInertiaPwm(currentYawRate, dt);

        float max_val = max(abs(Lv_tgt), abs(Rv_tgt));
        if (max_val > Config::REAL_V_MAX) {
            float ratio = Config::REAL_V_MAX / max_val;
            Lv_tgt *= ratio; Rv_tgt *= ratio;
        }

        leftTrack.update(Lv_tgt, dt, gradePitch, pitchInertiaPwm + yawInertiaPwm);
        rightTrack.update(Rv_tgt, dt, gradePitch, pitchInertiaPwm - yawInertiaPwm);
    }

    void stop() { 
        v_real = 0; spinV = 0; 
        longitudinalAccel = 0.0f;
        gradePitchDeg = 0.0f;
        gradePitchReady = false;
        lastPitchRateDeg = 0.0f;
        filteredPitchAlpha = 0.0f;
        pitchRateReady = false;
        lastYawRateDeg = 0.0f;
        filteredYawAlpha = 0.0f;
        yawRateReady = false;
        engineSmoother.reset(); brakeSmoother.reset();
        leftTrack.stop(); rightTrack.stop(); 
    }
};

// ==========================================
// 5. 炮塔双稳系统 (22.5°/s 现实限速)
// ==========================================
// 炮塔部分分成三层：
// 1. 读取 IMU/编码器，得到底盘姿态、炮管姿态和炮塔方位；
// 2. 处理玩家输入，维护“世界系目标朝向”和“炮管目标俯仰”；
// 3. 在稳定器开启时输出舵机俯仰指令和 yaw FOC 目标电压。
class TankTurret {
private:
    Adafruit_MPU6050 &mpuC, &mpuT; 
    Servo pitchServo; 
    BLDCMotor yawMotor;
    BLDCDriver3PWM yawDriver;
    MagneticSensorI2C yawSensor;
    CascadePID yawPID;
    sensors_event_t aC, gC, tC, aT, gT, tT;
    
    float currentPitchAngle = 90.0f;
    float savedPitch = 0, savedYawCont = 0;
    float yawContDeg = 0;
    volatile bool switchState = false;
    float pitchFiltered = 0;
    float t_gyroZ_offset = 0, t_gyroX_offset = 0; // 炮塔零偏
    float c_gyroZ_offset = 0, c_gyroX_offset = 0; // 底盘零偏
    float t_gx_cal_deg = 0.0f; // 存放处理后的炮管 pitch 角速度 (deg/s)
    float t_gz_cal_deg = 0.0f; // 存放处理后的炮塔 yaw 角速度 (deg/s)
    float c_gx_cal_deg = 0.0f; // 存放处理后的底盘俯仰角速度 (deg/s)
    float c_gz_cal_deg = 0.0f; // 存放处理后的底盘 yaw 角速度 (deg/s)
    float chassisPitchFiltered = 0.0f; // [新增] 用于存放底盘的坡度角
    bool ready = false;
    volatile bool imuHealthy = false;
    volatile bool yawSensorHealthy = false;
    volatile float cachedTurretMechYawDeg = 0.0f;
    volatile uint32_t lastYawSensorUpdateUs = 0;
    volatile float pendingYawTarget = 0.0f;
    uint32_t lastYawSensorCheckUs = 0;

    // 角度统一折算到 [-180, 180]，方便做“是不是在车体后方”的几何判断。
    float wrapAngle180(float angleDeg) {
        angleDeg = fmod(angleDeg, 360.0f);
        if (angleDeg > 180.0f) angleDeg -= 360.0f;
        if (angleDeg < -180.0f) angleDeg += 360.0f;
        return angleDeg;
    }

    // 根据“相对车体后方”的夹角，计算当前允许的最低俯角。
    // 正常时允许到 GUN_PITCH_MIN，接近发动机舱时逐渐抬到 REAR_DECK_SAFE_PITCH。
    float getRearDeckMinPitch(float yawDeg) {
        float yawWrapped = wrapAngle180(yawDeg);
        float rearOffset = abs(abs(yawWrapped) - Config::REAR_DECK_CENTER_YAW);

        if (rearOffset >= Config::REAR_DECK_AVOID_START) return Config::GUN_PITCH_MIN;
        if (rearOffset <= Config::REAR_DECK_AVOID_FULL) return Config::REAR_DECK_SAFE_PITCH;

        float span = Config::REAR_DECK_AVOID_START - Config::REAR_DECK_AVOID_FULL;
        float blend = (Config::REAR_DECK_AVOID_START - rearOffset) / span;
        blend = pow(constrain(blend, 0.0f, 1.0f), Config::REAR_DECK_BLEND_EXP);
        return Config::GUN_PITCH_MIN + (Config::REAR_DECK_SAFE_PITCH - Config::GUN_PITCH_MIN) * blend;
    }

    float protectPitchForRearDeck(float pitchDeg, float yawDeg) {
        return constrain(pitchDeg, getRearDeckMinPitch(yawDeg), Config::GUN_PITCH_MAX);
    }

    bool readAS5600MechanicalDeg(float& angleDeg) {
        Wire.beginTransmission(Config::AS5600_ADDR);
        Wire.write(Config::AS5600_ANGLE_REG);
        if (Wire.endTransmission(false) != 0) return false;

        uint8_t received = Wire.requestFrom(Config::AS5600_ADDR, (uint8_t)2);
        if (received != 2 || Wire.available() < 2) return false;

        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        uint16_t raw = ((uint16_t)(msb & 0x0F) << 8) | lsb;
        angleDeg = (raw * 360.0f) / 4096.0f;
        return isfinite(angleDeg);
    }

    void setImuHealthy(bool healthy) {
        portENTER_CRITICAL(&turretStateMux);
        imuHealthy = healthy;
        if (!healthy) {
            switchState = false;
            pendingYawTarget = 0.0f;
        }
        portEXIT_CRITICAL(&turretStateMux);
    }

    void setStabilizationEnabled(bool enabled) {
        portENTER_CRITICAL(&turretStateMux);
        switchState = enabled;
        if (!enabled) pendingYawTarget = 0.0f;
        portEXIT_CRITICAL(&turretStateMux);
    }

    bool isStabilizationEnabled() const {
        bool enabled;
        portENTER_CRITICAL(&turretStateMux);
        enabled = switchState;
        portEXIT_CRITICAL(&turretStateMux);
        return enabled;
    }

    void updateYawSensorCache(bool healthy, float sensorDeg = 0.0f) {
        portENTER_CRITICAL(&turretStateMux);
        yawSensorHealthy = healthy;
        if (healthy) {
            cachedTurretMechYawDeg = sensorDeg;
            lastYawSensorUpdateUs = micros();
        } else {
            switchState = false;
            pendingYawTarget = 0.0f;
        }
        portEXIT_CRITICAL(&turretStateMux);
    }

    bool readFreshYawSensorDeg(float& sensorDeg) const {
        bool healthy;
        uint32_t updatedUs;
        float cachedDeg;

        portENTER_CRITICAL(&turretStateMux);
        healthy = yawSensorHealthy;
        updatedUs = lastYawSensorUpdateUs;
        cachedDeg = cachedTurretMechYawDeg;
        portEXIT_CRITICAL(&turretStateMux);

        if (!healthy || ((uint32_t)(micros() - updatedUs) > Config::YAW_SENSOR_STALE_US)) return false;
        sensorDeg = cachedDeg;
        return true;
    }

    bool controlSensorsHealthy() const {
        bool imuOk, yawOk;
        uint32_t updatedUs;

        portENTER_CRITICAL(&turretStateMux);
        imuOk = imuHealthy;
        yawOk = yawSensorHealthy;
        updatedUs = lastYawSensorUpdateUs;
        portEXIT_CRITICAL(&turretStateMux);

        return imuOk && yawOk &&
               ((uint32_t)(micros() - updatedUs) <= Config::YAW_SENSOR_STALE_US);
    }

    void publishYawTarget(float targetVoltage) {
        portENTER_CRITICAL(&turretStateMux);
        pendingYawTarget = targetVoltage;
        portEXIT_CRITICAL(&turretStateMux);
    }

    bool readFocCommand(float& targetVoltage) const {
        bool enabled, imuOk, yawOk;
        uint32_t updatedUs;

        portENTER_CRITICAL(&turretStateMux);
        enabled = switchState;
        imuOk = imuHealthy;
        yawOk = yawSensorHealthy;
        updatedUs = lastYawSensorUpdateUs;
        targetVoltage = pendingYawTarget;
        portEXIT_CRITICAL(&turretStateMux);

        if (!enabled || !imuOk || !yawOk ||
            ((uint32_t)(micros() - updatedUs) > Config::YAW_SENSOR_STALE_US)) {
            targetVoltage = 0.0f;
            return false;
        }
        return true;
    }

    // FOC 核心持续刷新 AS5600 机械角缓存，控制核心只读缓存，避免跨核争用 I2C。
    bool isYawSensorFresh() const {
        float unused;
        return readFreshYawSensorDeg(unused);
    }

    float getTurretRelativeYawDegFromSensor() {
        float sensorDeg;
        if (!readFreshYawSensorDeg(sensorDeg)) return Config::REAR_DECK_CENTER_YAW;
        return wrapAngle180((sensorDeg - Config::TURRET_FRONT_SENSOR_OFFSET) * Config::TURRET_SENSOR_SIGN);
    }

    bool validateImuEvent(float valueDegPerSec) {
        return isfinite(valueDegPerSec) && abs(valueDegPerSec) <= Config::IMU_GYRO_SANITY_DPS;
    }

public:
    TankTurret(Adafruit_MPU6050& c, Adafruit_MPU6050& t) 
        : mpuC(c), mpuT(t), yawMotor(7), 
          yawDriver(Config::FOC_PWM_A, Config::FOC_PWM_B, Config::FOC_PWM_C),
          yawSensor(AS5600_I2C),
          yawPID(CustomPID(2.2, 0.0, 0.5, 0.0, Config::YAW_OUTER_RATE_MAX),
                 CustomPID(0.18, 0.01, 0.002, 5.0, Config::YAW_VOLTAGE_MAX),
                 Config::YAW_CHASSIS_FF_GAIN) {}

    bool init() {
        ESP32PWM::allocateTimer(2);
        pitchServo.setPeriodHertz(333);
        pitchServo.attach(Config::SERVO_PIN, 500, 2500);
        pitchServo.write(90);

        bool chassisOk = mpuC.begin(0x68, &Wire1);
        bool turretOk = mpuT.begin(0x69, &Wire1);
        if (!chassisOk || !turretOk) {
            LOG_ALWAYS("!!! MPU6050 init failed: chassis=%d turret=%d\n", chassisOk, turretOk);
            ready = false;
            return false;
        }
        mpuC.setGyroRange(MPU6050_RANGE_500_DEG); mpuT.setGyroRange(MPU6050_RANGE_500_DEG);

        yawSensor.init();
        float initialSensorDeg = 0.0f;
        if (readAS5600MechanicalDeg(initialSensorDeg)) {
            updateYawSensorCache(true, initialSensorDeg);
        } else {
            updateYawSensorCache(false);
            ready = false;
            LOG_ALWAYS("!!! AS5600 init check failed.\n");
            return false;
        }
        yawDriver.voltage_power_supply = 12.0; yawDriver.init();
        yawMotor.linkSensor(&yawSensor); yawMotor.linkDriver(&yawDriver);
        yawMotor.controller = MotionControlType::torque;
        yawMotor.init();
        ready = (yawMotor.initFOC() == 1);
        if (!ready) {
            LOG_ALWAYS("!!! Yaw motor FOC init failed.\n");
        }
        return ready;
    }

    // 上电静态标定：仅估零偏，不假设炮塔当前必须朝向正前。
    void calibrate() {
        if (!ready) return;
        LOG_ALWAYS(">>> Calibrating IMUs (%d samples), Keep Static...\n", Config::IMU_CALIB_SAMPLES);
        float sumT_Z = 0, sumT_X = 0, sumC_Z = 0, sumC_X = 0;
        
        for (int i = 0; i < Config::IMU_CALIB_SAMPLES; i++) { 
            mpuT.getEvent(&aT, &gT, &tT); 
            mpuC.getEvent(&aC, &gC, &tC); 
            
            sumT_Z += gT.gyro.z; sumT_X += gT.gyro.x; 
            sumC_Z += gC.gyro.z; sumC_X += gC.gyro.x; 
            delay(2); 
        }
        
        t_gyroZ_offset = sumT_Z / (float)Config::IMU_CALIB_SAMPLES; 
        t_gyroX_offset = sumT_X / (float)Config::IMU_CALIB_SAMPLES;
        c_gyroZ_offset = sumC_Z / (float)Config::IMU_CALIB_SAMPLES; 
        c_gyroX_offset = sumC_X / (float)Config::IMU_CALIB_SAMPLES;
        setImuHealthy(true);
        yawPID.reset();
        setStabilizationEnabled(false);
        savedPitch = pitchFiltered;
        savedYawCont = yawContDeg;

        // 校准完成后，炮管上下“点头”一下
        pitchServo.write(105);
        delay(200);
        pitchServo.write(90);
        LOG_ALWAYS(">>> Calib Done!\n");
    }

    void updateIMU(float dt) {
        if (!ready) return;
        if (dt <= 0.0f || dt > Config::IMU_MAX_DT) {
            setImuHealthy(false);
            return;
        }
        mpuC.getEvent(&aC, &gC, &tC); 
        mpuT.getEvent(&aT, &gT, &tT);

        // 保存底盘校准后的角速度，减去零偏并转换为度/秒
        c_gx_cal_deg = (gC.gyro.x - c_gyroX_offset) * RAD_TO_DEG;
        c_gz_cal_deg = (gC.gyro.z - c_gyroZ_offset) * RAD_TO_DEG;

        // 底盘俯仰角计算 (坡度)
        float c_pitchAcc = atan2(aC.acceleration.y, aC.acceleration.z) * RAD_TO_DEG;
        // 互补滤波计算底盘的绝对姿态
        chassisPitchFiltered = 0.96f * (chassisPitchFiltered + c_gx_cal_deg * dt) + 0.04f * c_pitchAcc;

        // 1. 减去零偏，得到真实角速度。这里先做原始值健壮性检查，异常就直接退出稳定。
        float gz_cal = gT.gyro.z - t_gyroZ_offset;
        if (abs(gz_cal) < 0.005f) gz_cal = 0.0f; // 消除静止底噪带来的缓慢漂移
        t_gx_cal_deg = (gT.gyro.x - t_gyroX_offset) * RAD_TO_DEG;
        t_gz_cal_deg = gz_cal * RAD_TO_DEG;
        float pitchAcc = atan2(aT.acceleration.y, aT.acceleration.z) * RAD_TO_DEG;

        if (!isfinite(c_pitchAcc) || !isfinite(pitchAcc) ||
            !validateImuEvent(c_gx_cal_deg) ||
            !validateImuEvent(t_gx_cal_deg) ||
            !validateImuEvent(t_gz_cal_deg) ||
            !validateImuEvent(c_gz_cal_deg)) {
            setImuHealthy(false);
            return;
        }
        setImuHealthy(true);

        // 2. 俯仰角 (Pitch) 互补滤波：陀螺仪管快速稳定，加速度计只慢速纠漂。
        float pitchAccelBlend = dt / (Config::PITCH_ACC_TAU + dt);
        float pitchGyroPrediction = pitchFiltered + t_gx_cal_deg * dt;
        pitchFiltered = (1.0f - pitchAccelBlend) * pitchGyroPrediction + pitchAccelBlend * pitchAcc;

        // 3. yaw 继续用积分维持“世界系目标”，而不是相对车体角；
        // 相对车体几何关系已经由 AS5600 单独负责。
        yawContDeg += t_gz_cal_deg * dt;
    }

    // 获取已经算好的底盘俯仰速率
    float getLatestChassisPitchRate() {
        return c_gx_cal_deg;
    }
    float getLatestChassisYawRate() {
        return c_gz_cal_deg;
    }
    // 获取当前坡度角
    float getChassisPitchAngle() {
        return chassisPitchFiltered;
    }

    void runFOC() {
        if (!ready) return;
        yawMotor.loopFOC();

        uint32_t nowUs = micros();
        if ((uint32_t)(nowUs - lastYawSensorCheckUs) >= Config::YAW_SENSOR_CHECK_US) {
            lastYawSensorCheckUs = nowUs;
            float sensorDeg = 0.0f;
            updateYawSensorCache(readAS5600MechanicalDeg(sensorDeg), sensorDeg);
        }

        float targetVoltage = 0.0f;
        if (readFocCommand(targetVoltage)) {
            yawMotor.target = targetVoltage;
            yawMotor.move();
        } else {
            yawMotor.target = 0.0f;
            yawMotor.move(0);
        }
    }

    void enterSafeState() {
        setImuHealthy(false);
        yawPID.reset();
        publishYawTarget(0.0f);
    }

    // 断连和故障分开处理：断连不等于传感器坏了，只是立即停止执行目标。
    void enterDisconnectedState() {
        setStabilizationEnabled(false);
        yawPID.reset();
        publishYawTarget(0.0f);
    }

    // A 键作为稳定器总开关；只有 IMU 和 AS5600 都健康时才允许进入稳定模式。
    void handleUI(bool aPressed, float joyX, float joyY, float dt) {
        if (!ready) return;
        static bool lastA = false;
        if (aPressed && !lastA) {
            if (isStabilizationEnabled()) {
                setStabilizationEnabled(false);
                yawPID.reset();
                publishYawTarget(0.0f);
            } else if (controlSensorsHealthy()) {
                setStabilizationEnabled(true);
                savedPitch = pitchFiltered;
                savedYawCont = yawContDeg;
                yawPID.reset();
            }
        }
        lastA = aPressed;
        if (isStabilizationEnabled()) {
            // 真车 22.5 deg/s 映射
            if (abs(joyX) > 0.15f) {
                // 计算有效推力比例：0.15时为0，1.0时为1.0
                float effectiveX = copysign((abs(joyX) - 0.15f) / 0.85f, joyX);
                savedYawCont += effectiveX * Config::REAL_TURRET_VEL * dt;
            }
            if (abs(joyY) > 0.15f) {
                // 使用同样的线性映射：消除 0.15 处的突变跳变
                float effectiveY = copysign((abs(joyY) - 0.15f) / 0.85f, joyY);
                // 注意：joyY 通常向上推是负值，向下推是正值，请根据你的操作习惯确认符号
                savedPitch = constrain(savedPitch - effectiveY * Config::REAL_TURRET_VEL * dt, Config::GUN_PITCH_MIN, Config::GUN_PITCH_MAX);
            }
        }
    }

    void updateStabilization(float dt) {
        if (!ready || !isStabilizationEnabled()) return;
        if (!controlSensorsHealthy()) {
            enterSafeState();
            return;
        }
        
        // 俯仰双稳：底盘抬头会立刻通过前馈向下补，位置误差再由 P 环慢慢拉回。
        float chassisPitchRateDeg = c_gx_cal_deg;
        float chassisYawRateDeg = c_gz_cal_deg;

        float protectedPitch = protectPitchForRearDeck(savedPitch, getTurretRelativeYawDegFromSensor());
        float pErr = protectedPitch - pitchFiltered;
        // P 负责回到目标，底盘前馈负责抵消车体点头，炮管自身角速度阻尼负责压过冲和抖动。
        float pitchRateCmd = (pErr * Config::PITCH_STAB_KP) -
                             (chassisPitchRateDeg * Config::PITCH_CHASSIS_FF) -
                             (t_gx_cal_deg * Config::PITCH_STAB_KD);
        pitchRateCmd = constrain(pitchRateCmd, -Config::PITCH_RATE_CMD_MAX, Config::PITCH_RATE_CMD_MAX);
        if (abs(pitchRateCmd) < Config::PITCH_SERVO_RATE_DEADZONE_DPS) {
            pitchRateCmd = 0.0f;
        }

        // 舵机命令限位用机构角，不直接等于物理俯仰角。
        if (pitchRateCmd != 0.0f) {
            currentPitchAngle = constrain(currentPitchAngle + (pitchRateCmd * dt), Config::SERVO_CMD_MIN, Config::SERVO_CMD_MAX);
            pitchServo.write(currentPitchAngle);
        }
        
        // yaw 双稳继续工作在“世界系目标”上，底盘转动时通过底盘 yaw 角速度前馈抵消。
        float yVoltage = yawPID.calculate(savedYawCont, yawContDeg, t_gz_cal_deg, -chassisYawRateDeg, dt);
        publishYawTarget(yVoltage);
        
        LOG(TURRET_ONLY, "Y_Tgt:%.2f, Y_Real:%.2f, Y_RelSens:%.2f\n", savedYawCont, yawContDeg, getTurretRelativeYawDegFromSensor());
    }

    bool isReady() const { return ready; }
    bool isHealthy() const { return ready && controlSensorsHealthy(); }
};

// ==========================================
// 6. 顶层统筹与任务调度
// ==========================================
class TankRobot {
private:
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
    Adafruit_MPU6050 mpuChassis, mpuTurret;
    TankChassis chassis; TankTurret turret;
    uint32_t lastIMU = 0, lastUI = 0, lastCtrl = 0;
    uint32_t lastControllerPacketMs = 0;
    uint32_t lastBatterySampleMs = 0;
    uint32_t lastBatteryCutoffLogMs = 0;
    bool chassisReady = false;
    bool turretReady = false;
    float batteryVoltage = 0.0f;
    bool batteryValid = false;

    float readBatteryVoltage() {
        uint32_t adcMilliVolts = analogReadMilliVolts(Config::VBAT_ADC_PIN);
        float adcVolts = adcMilliVolts * 0.001f;
        return adcVolts * ((Config::VBAT_DIVIDER_R1 + Config::VBAT_DIVIDER_R2) / Config::VBAT_DIVIDER_R2);
    }

    void updateBatteryMonitor() {
        uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastBatterySampleMs) < Config::VBAT_SAMPLE_MS) return;
        lastBatterySampleMs = nowMs;

        float sample = readBatteryVoltage();
        if (!isfinite(sample) || sample <= 0.0f) {
            batteryValid = false;
            return;
        }

        if (!batteryValid) {
            batteryVoltage = sample;
            batteryValid = true;
        } else {
            batteryVoltage += Config::VBAT_LPF * (sample - batteryVoltage);
        }
    }

    bool batteryCritical() const {
        return batteryValid && batteryVoltage <= Config::VBAT_CUTOFF;
    }

    bool batteryWarning() const {
        return batteryValid && batteryVoltage <= Config::VBAT_WARN;
    }

    void updateControllerPacketClock() {
        unsigned long receivedAt = xboxController.getReceiveNotificationAt();
        if (receivedAt != 0) {
            lastControllerPacketMs = receivedAt;
        }
    }

    // 手柄库的 isConnected() 不是 const 成员，所以这里不能把方法声明成 const。
    bool controllerHealthy() {
        return xboxController.isConnected() &&
               lastControllerPacketMs != 0 &&
               ((uint32_t)(millis() - lastControllerPacketMs) <= Config::CONTROLLER_TIMEOUT_MS);
    }

public:
    TankRobot() : xboxController(Config::XBOX_MAC), turret(mpuChassis, mpuTurret) {}


    void setup() {
        Serial.begin(921600);
        Wire.begin(Config::I2C_FOC_SDA, Config::I2C_FOC_SCL); Wire.setClock(400000); 
        Wire1.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL); Wire1.setClock(400000); 
        analogReadResolution(12);
        analogSetPinAttenuation(Config::VBAT_ADC_PIN, ADC_11db);
        pinMode(Config::VBAT_ADC_PIN, INPUT);
        updateBatteryMonitor();
        chassis.init();
        chassisReady = true;
        turretReady = turret.init();
        if (turretReady) {
            delay(200);
            turret.calibrate();
        } else {
            LOG_ALWAYS("!!! Turret unavailable; chassis control remains enabled.\n");
        }
        xboxController.begin(); 
        lastControllerPacketMs = 0;

        uint32_t now = micros();
        lastIMU = now; lastUI = now; lastCtrl = now;
    }

    
    void runFOC_Only() {
        if (!turretReady) return;
        turret.runFOC(); // 内部调用 yawMotor.loopFOC() 和 move()
    }

    // Core 1 主循环：低频 UI、中频控制、高频 IMU，和 Core 0 的 FOC 任务解耦。
    void loop_without_FOC() {
        // 蓝牙、IMU、底盘动力学都在 Core 1 执行
        uint32_t nowMicros = micros();
        updateBatteryMonitor();

        if (turretReady && nowMicros - lastIMU >= 2000) { // 500Hz IMU
            float dtIMU = (nowMicros - lastIMU) * 1e-6f; 
            lastIMU = nowMicros;
            turret.updateIMU(dtIMU);
        }
        if (nowMicros - lastUI >= 20000) { // 50Hz UI
            float dtUI = (nowMicros - lastUI) * 1e-6f;
            lastUI = nowMicros; xboxController.onLoop();
            updateControllerPacketClock();
            if (turretReady && controllerHealthy()) {
                float jX = (xboxController.xboxNotif.joyRHori - 32767.5f) / 32767.5f;
                float jY = (xboxController.xboxNotif.joyRVert - 32767.5f) / 32767.5f;
                turret.handleUI(xboxController.xboxNotif.btnA, jX, jY, dtUI);
            }
        }
        if (nowMicros - lastCtrl >= 5000) { // 200Hz 控制
            float dtCtrl = (nowMicros - lastCtrl) * 1e-6f;
            lastCtrl = nowMicros;
            if (batteryCritical()) {
                if (chassisReady) chassis.stop();
                if (turretReady) turret.enterSafeState();
                if (millis() - lastBatteryCutoffLogMs >= 1000) {
                    lastBatteryCutoffLogMs = millis();
                    LOG_ALWAYS("!!! Battery cutoff active: %.2fV\n", batteryVoltage);
                }
            } else if (chassisReady && controllerHealthy()) {
                float tL = xboxController.xboxNotif.trigLT / 1023.0f;
                float tR = xboxController.xboxNotif.trigRT / 1023.0f;
                float jLX = (xboxController.xboxNotif.joyLHori - 32767.5f) / 32767.5f;
                // 坡度角用于坡度前馈，pitch/yaw rate 用于生成虚拟旋转惯量。
                float pRate = turretReady ? turret.getLatestChassisPitchRate() : 0.0f;
                float yRate = turretReady ? turret.getLatestChassisYawRate() : 0.0f;
                float pAngle = turretReady ? turret.getChassisPitchAngle() : 0.0f;
                chassis.processKinematics(tL, tR, jLX, dtCtrl, pRate, yRate, pAngle);
                if (turretReady) turret.updateStabilization(dtCtrl);

                static uint32_t lastBatteryWarnLogMs = 0;
                if (batteryWarning() && (millis() - lastBatteryWarnLogMs >= 1000)) {
                    lastBatteryWarnLogMs = millis();
                    LOG_ALWAYS("*** Battery low warning: %.2fV\n", batteryVoltage);
                }
            } else {
                if (chassisReady) chassis.stop();
                if (turretReady) turret.enterDisconnectedState();
            }
        }
    }
};

// ==========================================
// 全局实例与 FreeRTOS 双核入口
// ==========================================
TankRobot robot;
TaskHandle_t FOC_TaskHandle;
// Core 0 的任务函数：只负责无刷电机的 FOC 算法
void FocTask(void *pvParameters) {
    for (;;) {
        robot.runFOC_Only(); 
        vTaskDelay(0); // 必须保留，让出微秒级CPU时间给系统底层（如看门狗），防止崩溃
    }
}

// 默认在 Core 1 上运行的 setup
void setup() { 
    robot.setup(); 
    
    // 将 FOC 任务绑定到 Core 0
    xTaskCreatePinnedToCore(
        FocTask,       // 任务函数
        "FOC_Task",    // 任务名称
        8192,          // 堆栈大小 (分配8K给FOC防溢出)
        NULL,          // 任务参数
        5,             // 优先级 (数字越大优先级越高，设为5确保FOC优先执行)
        &FOC_TaskHandle, // 任务句柄
        0              // 核心编号：Core 0
    );
}

// 默认在 Core 1 上运行的 loop
void loop() { 
    // 处理底盘、IMU、蓝牙、PID和舵机
    robot.loop_without_FOC(); 
}
