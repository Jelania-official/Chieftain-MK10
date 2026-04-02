#include <Arduino.h>
#include <Wire.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <SimpleFOC.h>
#include <driver/pcnt.h>

#define PI 3.1415926

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
    const uint8_t PWM_CH_R = 0, PWM_CH_L = 1;         // ESP32 硬件PWM通道
    const uint32_t PWM_FREQ = 10000;                  // 电机控制频率 10kHz
    const uint8_t PWM_RES = 8;                        // 8位分辨率 (0-255)

    // [编码器引脚] - 34/35需外部上拉电阻
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
    const float RT_THRESHOLD = 0.2;         // 扳机触发阈值
    const uint32_t RT_HOLD_TIME = 2000;     // 挂档长按时间(ms)
    // [动力学精细调校 - 3阶导数 Jerk 限制]
    // 前进/后退推力爬升限制 (km/h/s²)
    const float LINEAR_JERK_ACCEL = 0.4f;  // 模拟 L60 引擎缓慢的扭矩堆积
    const float LINEAR_JERK_BRAKE = 2.5f;  // 模拟液压制动系统快速建立压力

    // 转向/自转动力爬升限制
    const float YAW_JERK_ACCEL = 0.3f;     // 模拟侧向铲土阻力导致的启动迟滞
    const float YAW_JERK_BRAKE = 1.2f;     // 转向停止时的惯性缓冲

    // [炮塔与双稳]
    const uint8_t SERVO_PIN = 15;           // 俯仰舵机引脚
    const uint8_t FOC_PWM_A = 5, FOC_PWM_B = 19, FOC_PWM_C = 18; // 无刷驱动引脚
    const float REAL_TURRET_VEL = 22.5f;    // 真车转塔速度 (deg/s)
    const int IMU_CALIB_SAMPLES = 2000;     // IMU 启动校准采样次数 (2000次约4秒)
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
        // 公式：实际PWM = 最小起步PWM + (输入 / 255) * (最大255 - 最小40)
        float mappedPWM = 40.0f + (absOut / 255.0f) * (255.0f - 40.0f);
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
    pcnt_unit_t unit; int16_t lastCount = 0; uint32_t lastTime = 0;
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
        static float lastSpeed = 0;
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

class AccelRateLimiter {
private:
    float a_actual = 0.0f;
    float jerk_accel, jerk_brake;

public:
    // 构造函数：初始化不同的 Jerk 限制
    AccelRateLimiter(float j_accel, float j_brake) 
        : jerk_accel(j_accel), jerk_brake(j_brake) {}

    float update(float target_a, float dt) {
        float a_error = target_a - a_actual;

        // 根据加/减速状态选择限制值
        float current_jerk_limit = (abs(target_a) > abs(a_actual)) 
                                   ? jerk_accel 
                                   : jerk_brake;

        float max_delta_a = current_jerk_limit * dt;
        a_actual += constrain(a_error, -max_delta_a, max_delta_a);

        return a_actual;
    }

    void reset() { a_actual = 0.0f; }
};

class TankChassis {
private:
    TankTrack rightTrack, leftTrack;
    float v_real = 0, spinV = 0; 
    bool reverseMode = false;
    uint32_t rtPressedStartTime = 0;

    // --- 实例化两个独立的动力限幅器 ---
    AccelRateLimiter linearLimiter; 
    AccelRateLimiter yawLimiter;

public:
    TankChassis() : 
        rightTrack(DCMotor(Config::R_IN1, Config::R_IN2, Config::R_PWM, Config::PWM_CH_R, false),
                   CustomEncoder(Config::R_ENCA, Config::R_ENCB, PCNT_UNIT_0),
                   CustomPID(1.2, 0.05, 0.3, 100.0, 255.0)),
        leftTrack (DCMotor(Config::L_IN1, Config::L_IN2, Config::L_PWM, Config::PWM_CH_L, true),
                   CustomEncoder(Config::L_ENCA, Config::L_ENCB, PCNT_UNIT_1),
                   CustomPID(1.2, 0.05, 0.3, 100.0, 255.0)),
        // 在初始化列表中注入 Config 参数
        linearLimiter(Config::LINEAR_JERK_ACCEL, Config::LINEAR_JERK_BRAKE),
        yawLimiter(Config::YAW_JERK_ACCEL, Config::YAW_JERK_BRAKE) 
    {}

    void init() { rightTrack.init(); leftTrack.init(); }

    void processKinematics(float triggerL, float triggerR, float joyX, float dt) {
        uint32_t now = millis();
        bool isStopped = (abs(v_real) < 0.5f);

        // 1. 倒车档位切换逻辑
        if (triggerR > Config::RT_THRESHOLD) {
            if (rtPressedStartTime == 0) rtPressedStartTime = now;
            else if (isStopped && (now - rtPressedStartTime >= Config::RT_HOLD_TIME)) {
                if (!reverseMode) { reverseMode = true; linearLimiter.reset(); }
            }
        }
        else {
            rtPressedStartTime = 0;
            if (isStopped && reverseMode && triggerL > Config::RT_THRESHOLD) {
                reverseMode = false;
                linearLimiter.reset();
            }
        }

        // 2. 前进/后退动力学模拟
        float airResist = 0.001f * v_real * v_real;
        // 速度在 0~0.5km/h 之间时，阻力从 0 线性增加到 0.6，彻底解决起步跳变
        float rollResistFactor = constrain(abs(v_real) / 0.5f, 0.0f, 1.0f);
        float rollResist = 0.6f * rollResistFactor;        
        float target_a_engine = 0.0f;
        float smooth_a_engine = 0.0f;
        float brake_force = 0.0f;
        float a_final = 0.0f;

        if (!reverseMode) {
            target_a_engine = Config::REAL_ACCEL * (triggerL * triggerL);
            smooth_a_engine = linearLimiter.update(target_a_engine, dt);
            brake_force = Config::REAL_BRAKE * (triggerR * triggerR); 
            a_final = smooth_a_engine - airResist - rollResist - brake_force;
            v_real = constrain(v_real + a_final * dt, 0, Config::REAL_V_MAX);
        } else {
            target_a_engine = -Config::REAL_ACCEL * (triggerR * triggerR);
            smooth_a_engine = linearLimiter.update(target_a_engine, dt);
            brake_force = Config::REAL_BRAKE * (triggerL * triggerL);
            a_final = smooth_a_engine + airResist + rollResist + brake_force;
            v_real = constrain(v_real + a_final * dt, -Config::REAL_V_REV_MAX, 0);
        }

        // 3. 转向逻辑 (包含转向限幅)
        float joyX_adj = (abs(joyX) < 0.12f) ? 0 : joyX;
        bool isSpinMode = (abs(v_real) < 1.0f && abs(joyX_adj) > 0.2f);
        float Lv_tgt = 0, Rv_tgt = 0;

        if (isSpinMode) {
            // 原地自转：将摇杆映射为目标转向加速度
            float target_a_spin = (2.0f * abs(joyX_adj) - 0.5f) * 5.0f; 
            float smooth_a_spin = yawLimiter.update(target_a_spin, dt);
            
            spinV = constrain(spinV + smooth_a_spin * dt, 0, 8.0f); 
            Lv_tgt = -spinV * copysign(1.0f, joyX_adj); 
            Rv_tgt =  spinV * copysign(1.0f, joyX_adj);
            
            linearLimiter.reset(); // 自转时重置前进限幅器，防止干扰
        } else {
            spinV = (spinV - 5.0f * dt > 0.0f) ? (spinV - 5.0f * dt) : 0.0f;
            yawLimiter.reset();    // 正常行驶模式重置转向限幅器
            
            // 差速模型
            Lv_tgt = v_real * (1.0f + 0.4f * joyX_adj);
            Rv_tgt = v_real * (1.0f - 0.4f * joyX_adj);
        }

        leftTrack.update(Lv_tgt, dt); 
        rightTrack.update(Rv_tgt, dt);

        LOG(CHASSIS_ONLY, "REAL_KMH:%.2f, L:%.2f, R:%.2f, Mode:%s\n", 
            v_real, leftTrack.currentSpeed, rightTrack.currentSpeed, reverseMode ? "REV" : "FWD");
    }

    void stop() { 
        v_real = 0; spinV = 0; 
        linearLimiter.reset(); yawLimiter.reset();
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
    float pitchFiltered = 0;
    float t_gyroZ_offset = 0, t_gyroX_offset = 0; // 炮塔零偏
    float c_gyroZ_offset = 0, c_gyroX_offset = 0; // 底盘零偏

public:
    TankTurret(Adafruit_MPU6050& c, Adafruit_MPU6050& t) 
        : mpuC(c), mpuT(t), yawMotor(7), 
          yawDriver(Config::FOC_PWM_A, Config::FOC_PWM_B, Config::FOC_PWM_C),
          yawSensor(AS5600_I2C),
          yawPID(CustomPID(2.2, 0.0, 0.5, 0.0, 15.0), CustomPID(0.18, 0.01, 0.002, 5.0, 8.0), 0.6f) {}

    void init() {
        ESP32PWM::allocateTimer(2);
        pitchServo.setPeriodHertz(50);
        pitchServo.attach(Config::SERVO_PIN, 500, 2500);
        pitchServo.write(90);

        mpuC.begin(0x68, &Wire1); mpuT.begin(0x69, &Wire1);
        mpuC.setGyroRange(MPU6050_RANGE_500_DEG); mpuT.setGyroRange(MPU6050_RANGE_500_DEG);

        yawSensor.init();
        yawDriver.voltage_power_supply = 12.0; yawDriver.init();
        yawMotor.linkSensor(&yawSensor); yawMotor.linkDriver(&yawDriver);
        yawMotor.controller = MotionControlType::torque;
        yawMotor.init(); yawMotor.initFOC();
    }

    void calibrate() {
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
        mpuC.getEvent(&aC, &gC, &tC); 
        mpuT.getEvent(&aT, &gT, &tT);

        // 1. 减去零偏，得到真实的角速度
        float gz_cal = gT.gyro.z - t_gyroZ_offset; 
        float gx_cal = gT.gyro.x - t_gyroX_offset;

        // 2. 俯仰角 (Pitch) 互补滤波
        float pitchAcc = atan2(aT.acceleration.y, aT.acceleration.z) * RAD_TO_DEG;
        pitchFiltered = 0.96f * (pitchFiltered + gx_cal * RAD_TO_DEG * dt) + 0.04f * pitchAcc;

        // 3. 偏航角 (Yaw) 直接积分 (删除了多余的 accYaw 和 unwrapYaw)
        // yawContDeg 现在就是一个纯净的、无限连续的相对角度
        yawContDeg += gz_cal * RAD_TO_DEG * dt; 
    }

    void runFOC() { yawMotor.loopFOC(); if (switchState) yawMotor.move(); else yawMotor.move(0); }

    void handleUI(bool aPressed, float joyX, float joyY, float dt) {
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
                savedPitch = constrain(savedPitch - effectiveY * Config::REAL_TURRET_VEL * dt, -15.0f, 30.0f);
            }
        }
    }

    void updateStabilization(float dt) {
        if (!switchState) return;
        
        // 修正：补偿计算时，减去底盘 IMU 的零偏
        float c_gx_cal = gC.gyro.x - c_gyroX_offset;
        float c_gz_cal = gC.gyro.z - c_gyroZ_offset;

        float pErr = savedPitch - pitchFiltered;
        float pComp = (pErr * 0.85f) - (c_gx_cal * RAD_TO_DEG * 0.06f);
        currentPitchAngle = constrain(currentPitchAngle + pComp, 45.0f, 135.0f);
        pitchServo.write(currentPitchAngle);
        
        float yVoltage = yawPID.calculate(savedYawCont, yawContDeg, (gT.gyro.z - t_gyroZ_offset) * RAD_TO_DEG, -c_gz_cal * RAD_TO_DEG, dt);
        yawMotor.target = yVoltage;
        
        LOG(TURRET_ONLY, "Y_Tgt:%.2f, Y_Real:%.2f\n", savedYawCont, yawContDeg);
    }
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

public:
    TankRobot() : xboxController(Config::XBOX_MAC), turret(mpuChassis, mpuTurret) {}

    void setup() {
        Serial.begin(921600);
        Wire.begin(Config::I2C_FOC_SDA, Config::I2C_FOC_SCL); Wire.setClock(400000); 
        Wire1.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL); Wire1.setClock(400000); 
        chassis.init(); turret.init(); 
        delay(200); turret.calibrate(); 
        xboxController.begin(); 
    }

    void loop() {
        uint32_t nowMicros = micros();
        turret.runFOC(); 

        if (nowMicros - lastIMU >= 2000) { // 500Hz IMU
            float dtIMU = (nowMicros - lastIMU) * 1e-6f; 
            lastIMU = nowMicros;
            turret.updateIMU(dtIMU);
        }
        if (nowMicros - lastUI >= 20000) { // 50Hz UI
            float dtUI = (nowMicros - lastUI) * 1e-6f;
            lastUI = nowMicros; xboxController.onLoop();
            if (xboxController.isConnected()) {
                float jX = (xboxController.xboxNotif.joyRHori - 32767.5f) / 32767.5f;
                float jY = (xboxController.xboxNotif.joyRVert - 32767.5f) / 32767.5f;
                turret.handleUI(xboxController.xboxNotif.btnA, jX, jY, dtUI);
            }
        }
        if (nowMicros - lastCtrl >= 5000) { // 200Hz 控制
            float dtCtrl = (nowMicros - lastCtrl) * 1e-6f;
            lastCtrl = nowMicros;
            if (xboxController.isConnected()) {
                float tL = xboxController.xboxNotif.trigLT / 1023.0f;
                float tR = xboxController.xboxNotif.trigRT / 1023.0f;
                float jLX = (xboxController.xboxNotif.joyLHori - 32767.5f) / 32767.5f;
                chassis.processKinematics(tL, tR, jLX, dtCtrl);
                turret.updateStabilization(dtCtrl);
            } else { chassis.stop(); }
        }
    }
};

TankRobot robot;
void setup() { robot.setup(); }
void loop() { robot.loop(); }