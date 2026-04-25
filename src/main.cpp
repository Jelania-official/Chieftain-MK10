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

    // [底盘动力引脚]
    const uint8_t R_IN1 = 25, R_IN2 = 33, R_PWM = 32; // 右侧直流驱动
    const uint8_t L_IN1 = 26, L_IN2 = 27, L_PWM = 14; // 左侧直流驱动
    const uint8_t PWM_CH_R = 8, PWM_CH_L = 9;         // ESP32 硬件PWM通道
    const uint32_t PWM_FREQ = 10000;                  // 电机控制频率 10kHz
    const uint8_t PWM_RES = 8;                        // 8位分辨率 (0-255)

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

    // [横向动力学与随速感应 (新增)]
    const float YAW_JERK_ACCEL = 2.0f;     // 转向液压建立速度 (比前进快)
    const float YAW_JERK_BRAKE = 4.0f;     // 转向停止时的惯性缓冲
    const float YAW_SENSITIVITY = 25.0f;   // 转向加速度增益 (配合阻尼决定最大转向速度)
    const float YAW_DAMPING = 3.5f;        // 转向物理阻尼系数 (每秒衰减比例，模拟侧向摩擦)
    const float SPEED_SENS_K = 0.08f;      // 随速衰减系数 (越高，高速时方向盘越“重”)

    // [惯性补偿参数]
    // 增益：决定了补偿的力度
    const float V_INERTIA_GAIN = 0.15f; 
    // 滤波系数：0.0 到 1.0 之间。
    // 越小越平稳（如 0.05），越大响应越快（如 0.3）。
    const float V_INERTIA_LPF = 0.1f;

    // [环境阻力精细调校]
    // 假设在垂直90度时，重力带来的最大加速度。数值越大，爬坡越吃力，下坡溜得越快。
    const float SLOPE_GRAVITY_MAX = 12.0f; // 单位：km/h/s

    // [炮塔与双稳]
    const uint8_t SERVO_PIN = 15;           // 俯仰舵机引脚
    const uint8_t FOC_PWM_A = 5, FOC_PWM_B = 19, FOC_PWM_C = 18; // 无刷驱动引脚
    const float REAL_TURRET_VEL = 22.5f;    // 真车转塔速度 (deg/s)
    const int IMU_CALIB_SAMPLES = 2000;     // IMU 启动校准采样次数 (2000次约4秒)
    const float GUN_PITCH_MIN = -10.0f;     // 正常最低俯角 (deg)
    const float GUN_PITCH_MAX = 20.0f;      // 最高仰角 (deg)
    const float REAR_DECK_CENTER_YAW = 180.0f; // 炮塔正后方相对角 (deg)
    const float REAR_DECK_AVOID_START = 15.0f; // 距正后方左右15度开始抬炮
    const float REAR_DECK_AVOID_FULL = 10.0f;  // 距正后方左右10度内完全抬到安全俯角
    const float REAR_DECK_SAFE_PITCH = 0.0f;   // 发动机舱上方允许的最低俯角 (deg)
    const float REAR_DECK_BLEND_EXP = 1.0f;    // 1.0为线性，>1更晚抬，<1更早抬
}

// ==========================================
// 2. 基础控制算法 (PID)
// ==========================================
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

class CascadePID {
public:
    CustomPID outer; CustomPID inner; float ff_gain;
    CascadePID(CustomPID out, CustomPID in, float ff = 0.0f) : outer(out), inner(in), ff_gain(ff) {}
    float calculate(float posRef, float posFdb, float velFdb, float chassisVel, float dt) {
        float targetVel = outer.calculate(posRef, posFdb, dt);
        float innerOut = inner.calculate(targetVel, velFdb, dt);
        return innerOut + (ff_gain * chassisVel);
    }
    void reset() { outer.reset(); inner.reset(); }
};

// ==========================================
// 3. 硬件抽象 (电机与编码器)
// ==========================================
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
        float absOut = abs(output);
        if (absOut < 0.1f) { 
            // 完全停止：不仅 PWM 给 0，电机引脚也要拉低，防止发热
            digitalWrite(in1, LOW); 
            digitalWrite(in2, LOW); 
            ledcWrite(pwmCh, 0);
            return;
        }
        // 2. 关键优化：线性重映射 (Deadzone Compensation)
        // 把 0~255 的输入映射到 40~255 的输出
        float mappedPWM = (absOut > 0) ? 40.0f + (absOut / 255.0f) * 215.0f : 0;
        mappedPWM = constrain(mappedPWM, 40, 255);

        // 3. 物理驱动方向逻辑
        if (output > 0) {
            digitalWrite(in1, isLeft ? HIGH : LOW); 
            digitalWrite(in2, isLeft ? LOW : HIGH);
            ledcWrite(pwmCh, (uint32_t)mappedPWM);
        } else {
            digitalWrite(in1, isLeft ? LOW : HIGH); 
            digitalWrite(in2, isLeft ? HIGH : LOW);
            ledcWrite(pwmCh, (uint32_t)mappedPWM);
        }
    }
};

class CustomEncoder {
private:
    pcnt_unit_t unit; int16_t lastCount = 0; uint32_t lastTime = 0; float lastSpeed = 0.0f;
public:
    CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit) : unit(p_unit) {
        pcnt_config_t cfg;
        cfg.pulse_gpio_num = pinA; cfg.ctrl_gpio_num = pinB;
        cfg.channel = PCNT_CHANNEL_0; cfg.unit = unit;
        cfg.pos_mode = PCNT_COUNT_INC; cfg.neg_mode = PCNT_COUNT_DEC;
        cfg.lctrl_mode = PCNT_MODE_KEEP; cfg.hctrl_mode = PCNT_MODE_REVERSE;
        cfg.counter_h_lim = 32767; cfg.counter_l_lim = -32768;
        pcnt_unit_config(&cfg);
    }
    void init() {
        pcnt_counter_pause(unit); pcnt_counter_clear(unit); pcnt_counter_resume(unit);
        pcnt_get_counter_value(unit, &lastCount);
        lastTime = millis();
    }
    float getRealSpeedKMH() {
        uint32_t now = millis(); uint32_t dt = now - lastTime;
        if (dt >= 10) { // 10ms 测速周期
            int16_t currentCount; pcnt_get_counter_value(unit, &currentCount);
            int32_t delta = (int32_t)currentCount - (int32_t)lastCount;
            lastCount = currentCount;
            float rpm = (delta / (float)Config::ENCODER_PPR / Config::GEAR_RATIO) * (60000.0f / dt);
            lastSpeed = rpm * Config::RPM_TO_REAL_KMH; // 输出真车等效速度
            lastTime = now;
        }
        return lastSpeed;
    }
};

// ==========================================
// 4. 底盘系统 (真车物理模拟)
// ==========================================
class TankTrack {
public:
    DCMotor motor; CustomEncoder encoder; CustomPID pid;
    float currentSpeed = 0, targetSpeed = 0;
    TankTrack(DCMotor m, CustomEncoder e, CustomPID p) : motor(m), encoder(e), pid(p) {}
    void init() { motor.init(); encoder.init(); }
    void update(float target, float dt) {
        targetSpeed = target;
        currentSpeed = encoder.getRealSpeedKMH();
        if (abs(targetSpeed) < 0.5f) targetSpeed = 0; // 0.5km/h 死区
        motor.drive(pid.calculate(targetSpeed, currentSpeed, dt));
    }
    void stop() { targetSpeed = 0; pid.reset(); motor.drive(0); }
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

class TankChassis {
private:
    TankTrack rightTrack, leftTrack;
    float v_real = 0, spinV = 0; 

    // 引擎油门建立缓慢 (0.8)，但松油门切断极快 (10.0)
    ThrottleSmoother engineSmoother; 
    
    // 刹车液压建立较快 (3.0)，松开也快 (10.0)
    ThrottleSmoother brakeSmoother;  

    float lastPitchRate = 0.0f;
    float filteredAlpha = 0.0f;

public:
    TankChassis() : 
        rightTrack(DCMotor(Config::R_IN1, Config::R_IN2, Config::R_PWM, Config::PWM_CH_R, false),
                   CustomEncoder(Config::R_ENCA, Config::R_ENCB, PCNT_UNIT_0),
                   CustomPID(1.2, 0.05, 0.3, 100.0, 255.0)),
        leftTrack (DCMotor(Config::L_IN1, Config::L_IN2, Config::L_PWM, Config::PWM_CH_L, true),
                   CustomEncoder(Config::L_ENCA, Config::L_ENCB, PCNT_UNIT_1),
                   CustomPID(1.2, 0.05, 0.3, 100.0, 255.0)),
        engineSmoother(0.8f, 10.0f),  // 参数可调：0.8表示油门踩到底需1秒多建立全扭矩
        brakeSmoother(4.0f, 10.0f)    // 参数可调：刹车建立很快
    {}

    void init() { rightTrack.init(); leftTrack.init(); }

    void processKinematics(float triggerL, float triggerR, float joyX, float dt, float currentPitchRate, float pitchAngle) {
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
        float force_slope = -Config::SLOPE_GRAVITY_MAX * sin(pitchAngle * DEG_TO_RAD);

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

        // 5. 积分计算最终纵向速度
        v_real += a_net * dt;

        // 极限速度钳制
        v_real = constrain(v_real, -Config::REAL_V_REV_MAX, Config::REAL_V_MAX);


        // ==========================================
        // 横向动力学 (重构为：目标速度追踪 + 强阻尼停转)
        // ==========================================
        
        float joyX_adj = (abs(joyX) < 0.12f) ? 0 : joyX; // 死区
        float joyX_squared = copysign(joyX_adj * joyX_adj, joyX_adj); 
        
        // 随速感应灵敏度
        float dynamic_sens = Config::YAW_SENSITIVITY / (1.0f + abs(v_real) * Config::SPEED_SENS_K);
        
        // 目标自转速度 (不再是目标加速度！)
        float target_spin_v = joyX_squared * dynamic_sens;

        // 履带转向的物理限制：加速建立有一个限度，但减速极快（因为没有滑行）
        float max_yaw_accel = Config::YAW_JERK_ACCEL; 
        
        // 以限制好的加速度，向目标速度逼近
        if (spinV < target_spin_v) {
            spinV = min(spinV + max_yaw_accel * dt, target_spin_v);
        } else if (spinV > target_spin_v) {
            spinV = max(spinV - max_yaw_accel * dt, target_spin_v);
        }

        // 当松开摇杆时，施加极强的地面摩擦力瞬间刹停
        if (joyX_adj == 0) {
            if (spinV > 0) spinV = max(spinV - Config::YAW_JERK_BRAKE * dt, 0.0f);
            else spinV = min(spinV + Config::YAW_JERK_BRAKE * dt, 0.0f);
        }

        // ==========================================
        // 双流耦合输出 (保持不变)
        // ==========================================
        float Lv_tgt = v_real + spinV;
        float Rv_tgt = v_real - spinV;

        float rawAlpha = (currentPitchRate - lastPitchRate) / dt;
        lastPitchRate = currentPitchRate;
        filteredAlpha = filteredAlpha + Config::V_INERTIA_LPF * (rawAlpha - filteredAlpha);
        
        float v_comp = filteredAlpha * Config::V_INERTIA_GAIN;
        Lv_tgt -= v_comp;
        Rv_tgt -= v_comp;

        float max_val = max(abs(Lv_tgt), abs(Rv_tgt));
        if (max_val > Config::REAL_V_MAX) {
            float ratio = Config::REAL_V_MAX / max_val;
            Lv_tgt *= ratio; Rv_tgt *= ratio;
        }

        leftTrack.update(Lv_tgt, dt); 
        rightTrack.update(Rv_tgt, dt);
    }

    void stop() { 
        v_real = 0; spinV = 0; 
        lastPitchRate = 0.0f; filteredAlpha = 0.0f;
        engineSmoother.reset(); brakeSmoother.reset();
        leftTrack.stop(); rightTrack.stop(); 
    }
};

// ==========================================
// 5. 炮塔双稳系统 (22.5°/s 现实限速)
// ==========================================
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
    bool yawInitDone = false, switchState = false;
    float pitchFiltered = 0, gyroZ_offset = 0, gyroX_offset = 0;
    float t_gyroZ_offset = 0, t_gyroX_offset = 0; // 炮塔零偏
    float c_gyroZ_offset = 0, c_gyroX_offset = 0; // 底盘零偏
    float c_gx_cal_deg = 0.0f; // 存放处理后的底盘俯仰角速度 (deg/s)
    float chassisPitchFiltered = 0.0f; // [新增] 用于存放底盘的坡度角
    bool ready = false;

    float wrapAngle180(float angleDeg) {
        angleDeg = fmod(angleDeg, 360.0f);
        if (angleDeg > 180.0f) angleDeg -= 360.0f;
        if (angleDeg < -180.0f) angleDeg += 360.0f;
        return angleDeg;
    }

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

public:
    TankTurret(Adafruit_MPU6050& c, Adafruit_MPU6050& t) 
        : mpuC(c), mpuT(t), yawMotor(7), 
          yawDriver(Config::FOC_PWM_A, Config::FOC_PWM_B, Config::FOC_PWM_C),
          yawSensor(AS5600_I2C),
          yawPID(CustomPID(2.2, 0.0, 0.5, 0.0, 15.0), CustomPID(0.18, 0.01, 0.002, 5.0, 8.0), 0.6f) {}

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
        
        // 校准完成后，炮管上下“点头”一下
        pitchServo.write(105);
        delay(200);
        pitchServo.write(90);
        LOG_ALWAYS(">>> Calib Done!\n");
    }

    void updateIMU(float dt) {
        if (!ready) return;
        mpuC.getEvent(&aC, &gC, &tC); 
        mpuT.getEvent(&aT, &gT, &tT);

        // 保存底盘校准后的 X 轴角速度，减去零偏并转换为度/秒
        c_gx_cal_deg = (gC.gyro.x - c_gyroX_offset) * RAD_TO_DEG;

        // 底盘俯仰角计算 (坡度)
        float c_pitchAcc = atan2(aC.acceleration.y, aC.acceleration.z) * RAD_TO_DEG;
        // 互补滤波计算底盘的绝对姿态
        chassisPitchFiltered = 0.96f * (chassisPitchFiltered + c_gx_cal_deg * dt) + 0.04f * c_pitchAcc;

        // 1. 减去零偏，得到真实的角速度
        float gz_cal = gT.gyro.z - t_gyroZ_offset;
        if (abs(gz_cal) < 0.005f) gz_cal = 0.0f; // 消除静止底噪带来的缓慢漂移
        float gx_cal = gT.gyro.x - t_gyroX_offset;

        // 2. 俯仰角 (Pitch) 互补滤波
        float pitchAcc = atan2(aT.acceleration.y, aT.acceleration.z) * RAD_TO_DEG;
        pitchFiltered = 0.96f * (pitchFiltered + gx_cal * RAD_TO_DEG * dt) + 0.04f * pitchAcc;

        // 3. 偏航角 (Yaw) 直接积分 (删除了多余的 accYaw 和 unwrapYaw)
        // yawContDeg 现在就是一个纯净的、无限连续的相对角度
        yawContDeg += gz_cal * RAD_TO_DEG * dt; 
    }

    // 获取已经算好的底盘俯仰速率
    float getLatestChassisPitchRate() {
        return c_gx_cal_deg;
    }
    // 获取当前坡度角
    float getChassisPitchAngle() {
        return chassisPitchFiltered;
    }

    void runFOC() { if (!ready) return; yawMotor.loopFOC(); if (switchState) yawMotor.move(); else yawMotor.move(0); }

    void handleUI(bool aPressed, float joyX, float joyY, float dt) {
        if (!ready) return;
        static bool lastA = false;
        if (aPressed && !lastA) {
            switchState = !switchState;
            if (switchState) { savedPitch = pitchFiltered; savedYawCont = yawContDeg; yawPID.reset(); }
        }
        lastA = aPressed;
        if (switchState) {
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
        if (!ready || !switchState) return;
        
        // 修正：补偿计算时，减去底盘 IMU 的零偏
        float chassisPitchRateDeg = c_gx_cal_deg;
        float chassisYawRateDeg = (gC.gyro.z - c_gyroZ_offset) * RAD_TO_DEG;

        // 假设期望的 P 增益为 150 (每1度误差，要求 150度/秒 的回正速度)
        // 前馈增益设为 1.0 (底盘抬起 10度/秒，舵机就低头 10度/秒 完全抵消)
        float kP_pitch = 100.0f; 
        float kFF_pitch = 1.0f;  

        float protectedPitch = protectPitchForRearDeck(savedPitch, savedYawCont);
        float pErr = protectedPitch - pitchFiltered;
        // 算出期望的补偿角速度 (deg/s)
        float pitchRateCmd = (pErr * kP_pitch) - (chassisPitchRateDeg * kFF_pitch);

        // 乘以 dt，得到本帧需要改变的具体角度
        currentPitchAngle = constrain(currentPitchAngle + (pitchRateCmd * dt), 45.0f, 135.0f);
        pitchServo.write(currentPitchAngle);
        
        float yVoltage = yawPID.calculate(savedYawCont, yawContDeg, (gT.gyro.z - t_gyroZ_offset) * RAD_TO_DEG, -chassisYawRateDeg, dt);
        yawMotor.target = yVoltage;
        
        LOG(TURRET_ONLY, "Y_Tgt:%.2f, Y_Real:%.2f\n", savedYawCont, yawContDeg);
    }

    bool isReady() const { return ready; }
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
    bool systemReady = false;

public:
    TankRobot() : xboxController(Config::XBOX_MAC), turret(mpuChassis, mpuTurret) {}


    void setup() {
        Serial.begin(921600);
        Wire.begin(Config::I2C_FOC_SDA, Config::I2C_FOC_SCL); Wire.setClock(400000); 
        Wire1.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL); Wire1.setClock(400000); 
        chassis.init();
        systemReady = turret.init();
        if (systemReady) {
            delay(200);
            turret.calibrate();
        } else {
            LOG_ALWAYS("!!! System entered safe mode because turret init failed.\n");
        }
        xboxController.begin(); 

        uint32_t now = micros();
        lastIMU = now; lastUI = now; lastCtrl = now;
    }

    
    void runFOC_Only() {
        if (!systemReady) return;
        turret.runFOC(); // 内部调用 yawMotor.loopFOC() 和 move()
    }

    void loop_without_FOC() {
        // 蓝牙、IMU、底盘动力学都在 Core 1 执行
        uint32_t nowMicros = micros();

        if (systemReady && nowMicros - lastIMU >= 2000) { // 500Hz IMU
            float dtIMU = (nowMicros - lastIMU) * 1e-6f; 
            lastIMU = nowMicros;
            turret.updateIMU(dtIMU);
        }
        if (nowMicros - lastUI >= 20000) { // 50Hz UI
            float dtUI = (nowMicros - lastUI) * 1e-6f;
            lastUI = nowMicros; xboxController.onLoop();
            if (systemReady && xboxController.isConnected()) {
                float jX = (xboxController.xboxNotif.joyRHori - 32767.5f) / 32767.5f;
                float jY = (xboxController.xboxNotif.joyRVert - 32767.5f) / 32767.5f;
                turret.handleUI(xboxController.xboxNotif.btnA, jX, jY, dtUI);
            }
        }
        if (nowMicros - lastCtrl >= 5000) { // 200Hz 控制
            float dtCtrl = (nowMicros - lastCtrl) * 1e-6f;
            lastCtrl = nowMicros;
            if (systemReady && xboxController.isConnected()) {
                float tL = xboxController.xboxNotif.trigLT / 1023.0f;
                float tR = xboxController.xboxNotif.trigRT / 1023.0f;
                float jLX = (xboxController.xboxNotif.joyLHori - 32767.5f) / 32767.5f;
                // 从底盘 IMU 获取 pitch 速度和坡度角
                float pRate = turret.getLatestChassisPitchRate();
                float pAngle = turret.getChassisPitchAngle();
                chassis.processKinematics(tL, tR, jLX, dtCtrl, pRate, pAngle);
                turret.updateStabilization(dtCtrl);
            } else { chassis.stop(); }
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
