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
/* * [调试的四个关键维度]
 * 1. 传感器自检 (IMU_RAW): 
 * 观察点：坦克静止时，GZ 是否接近 0。用途：确认漂移校准和 I2C 稳定性。
 * 2. 云台闭环 (TURRET_ONLY): 
 * 观察点：Y_Tgt 与 Y_Real 的跟随曲线。用途：精调云台 PID。
 * 3. 底盘动力学 (CHASSIS_ONLY): 
 * 观察点：V_Tgt 与 L/R_Spd。用途：确认编码器极性和底盘 PID。
 * 4. 一键静默 (GLOBAL_DEBUG 0): 
 * 用途：正式运行，关闭所有打印，释放 CPU 给 FOC 运算。
 */

#define GLOBAL_DEBUG 1  // 1: 开启调试, 0: 彻底关闭输出

enum DebugChannel {
    NONE,
    CHASSIS_ONLY,  // 监控：V_Tgt, L_Spd, R_Spd
    TURRET_ONLY,   // 监控：Y_Tgt, Y_Real, P_Real
    IMU_RAW,       // 监控：GZ 原始偏移 (用于确认校准)
    ALL            // 全部输出
};

DebugChannel currentChannel = TURRET_ONLY; // <-- [在此切换频道]

#if GLOBAL_DEBUG
  // FireWater 协议要求格式为 "名字:数值\n"
  // 这里使用空格分隔多个键值对，VOFA+ 可以在一行内解析多个通道
  #define LOG(ch, fmt, ...) if(currentChannel == ch || currentChannel == ALL) Serial.printf(fmt, ##__VA_ARGS__)
  #define LOG_ALWAYS(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define LOG(ch, fmt, ...)
  #define LOG_ALWAYS(fmt, ...)
#endif

// ==========================================
// 1. 全局配置参数 (保持不变)
// ==========================================
namespace Config {
    const char* XBOX_MAC = "28:ea:0b:d9:0b:9f";
    const uint8_t I2C_FOC_SDA = 16, I2C_FOC_SCL = 17; 
    const uint8_t I2C_IMU_SDA = 21, I2C_IMU_SCL = 22; 
    const uint8_t R_IN1 = 25, R_IN2 = 33, R_PWM = 32;
    const uint8_t L_IN1 = 26, L_IN2 = 27, L_PWM = 14;
    const uint8_t PWM_CH_R = 0, PWM_CH_L = 1;
    const uint32_t PWM_FREQ = 10000;
    const uint8_t PWM_RES = 8;
    const uint8_t R_ENCA = 23, R_ENCB = 4;
    const uint8_t L_ENCA = 35, L_ENCB = 34; 
    const int ENCODER_PPR = 7;
    const int GEAR_RATIO = 59;
    const float RPM_TO_MS = 13.3 / 540.0;
    const float V_MAX = 14.5;
    const float V_REVERSE_MAX = 6.0;
    const float BRAKE_COEFF = 1.5;
    const float RT_THRESHOLD = 0.2;
    const uint32_t RT_HOLD_TIME = 2000;
    const uint8_t SERVO_PIN = 15; 
    const uint8_t FOC_PWM_A = 5, FOC_PWM_B = 19, FOC_PWM_C = 18;
}

// ==========================================
// 2. 基础控制算法 (保持不变)
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
    CustomPID outer;
    CustomPID inner;
    float ff_gain;

    CascadePID(CustomPID out, CustomPID in, float ff = 0.0f) 
        : outer(out), inner(in), ff_gain(ff) {}

    float calculate(float posRef, float posFdb, float velFdb, float chassisVel, float dt) {
        float targetVel = outer.calculate(posRef, posFdb, dt);
        float innerOut = inner.calculate(targetVel, velFdb, dt);
        return innerOut + (ff_gain * chassisVel);
    }
    void reset() { outer.reset(); inner.reset(); }
};

// ==========================================
// 3. 硬件抽象
// ==========================================
class DCMotor {
private:
    uint8_t in1, in2, pwmPin, pwmCh;
    bool isLeft;
public:
    DCMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, uint8_t ch, bool left = false)
        : in1(pin1), in2(pin2), pwmPin(pwmPin), pwmCh(ch), isLeft(left) {}
    void init() {
        pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
        ledcSetup(pwmCh, Config::PWM_FREQ, Config::PWM_RES);
        ledcAttachPin(pwmPin, pwmCh);
    }
    void drive(float output) {
        output = constrain(output, -255, 255);
        if (abs(output) < 35 && output != 0) output = copysign(35, output);
        if (output > 0) {
            digitalWrite(in1, isLeft ? HIGH : LOW); digitalWrite(in2, isLeft ? LOW : HIGH);
            ledcWrite(pwmCh, output);
        } else if (output < 0) {
            digitalWrite(in1, isLeft ? LOW : HIGH); digitalWrite(in2, isLeft ? HIGH : LOW);
            ledcWrite(pwmCh, -output);
        } else { stop(); }
    }
    void stop() { digitalWrite(in1, LOW); digitalWrite(in2, LOW); ledcWrite(pwmCh, 0); }
};

class CustomEncoder {
private:
    pcnt_unit_t unit;
    int16_t lastCount = 0;
    uint32_t lastTime = 0;
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
    float getSpeedMS() {
        uint32_t now = millis();
        uint32_t dt = now - lastTime;
        static float lastSpeed = 0;
        if (dt >= 5) {
            int16_t currentCount;
            pcnt_get_counter_value(unit, &currentCount);
            int32_t delta = (int32_t)currentCount - (int32_t)lastCount;
            lastCount = currentCount;
            float rpm = (delta / (float)Config::ENCODER_PPR / Config::GEAR_RATIO) * (60000.0f / dt);
            lastSpeed = rpm * Config::RPM_TO_MS;
            lastTime = now;
        }
        return lastSpeed;
    }
};

// ==========================================
// 4. 子系统封装 (注入 CHASSIS_ONLY 日志)
// ==========================================
class TankTrack {
public:
    DCMotor motor; CustomEncoder encoder; CustomPID pid;
    float currentSpeed = 0, targetSpeed = 0;
    TankTrack(DCMotor m, CustomEncoder e, CustomPID p) : motor(m), encoder(e), pid(p) {}
    void init() { motor.init(); encoder.init(); }
    void update(float target, float dt) {
        targetSpeed = target;
        currentSpeed = encoder.getSpeedMS();
        if (abs(targetSpeed) < 0.015f) targetSpeed = 0;
        motor.drive(pid.calculate(targetSpeed, currentSpeed, dt));
    }
    void stop() { targetSpeed = 0; pid.reset(); motor.stop(); }
};

class TankChassis {
private:
    TankTrack rightTrack, leftTrack;
    float v = 0, spinV = 0;
    bool reverseMode = false;
    uint32_t rtPressedStartTime = 0;
public:
    TankChassis() : 
        rightTrack(DCMotor(Config::R_IN1, Config::R_IN2, Config::R_PWM, Config::PWM_CH_R, false),
                   CustomEncoder(Config::R_ENCA, Config::R_ENCB, PCNT_UNIT_0),
                   CustomPID(2.0, 0.1, 0.5, 1000.0, 255.0)),
        leftTrack (DCMotor(Config::L_IN1, Config::L_IN2, Config::L_PWM, Config::PWM_CH_L, true),
                   CustomEncoder(Config::L_ENCA, Config::L_ENCB, PCNT_UNIT_1),
                   CustomPID(2.0, 0.1, 0.5, 1000.0, 255.0)) {}

    void init() { rightTrack.init(); leftTrack.init(); }
    void processKinematics(float triggerL, float triggerR, float joyX, float dt) {
        uint32_t now = millis();
        bool isStopped = (abs(v) < 0.1f);
        if (triggerR > Config::RT_THRESHOLD) {
            if (rtPressedStartTime == 0) rtPressedStartTime = now;
            else if (isStopped && (now - rtPressedStartTime >= Config::RT_HOLD_TIME)) reverseMode = true;
        } else {
            rtPressedStartTime = 0;
            if (v > -0.1f && reverseMode && triggerL > Config::RT_THRESHOLD) reverseMode = false;
        }
        float a = 0.0f;
        float rollResist = (abs(v) < 3.0f) ? 0.5f : 0.338f;
        if (!reverseMode) {
            a = 1.4f * (triggerL * triggerL) - 0.006f * v * v - rollResist - Config::BRAKE_COEFF * triggerR;
            v = constrain(v + a * dt, 0, Config::V_MAX);
        } else {
            a = -1.4f * (triggerR * triggerR) + 0.006f * v * v + rollResist + Config::BRAKE_COEFF * triggerL;
            v = constrain(v + a * dt, -Config::V_REVERSE_MAX, 0);
        }
        float joyX_adj = (abs(joyX) < 0.12f) ? 0 : joyX;
        bool isSpinMode = (abs(v) < 0.1f && abs(joyX_adj) > 0.2f);
        float Lv = 0, Rv = 0;
        if (isSpinMode) {
            spinV = constrain(spinV + (1.4f * abs(joyX_adj) - 0.5f) * dt, 0, 1.22f);
            Lv = -spinV * copysign(1.0f, joyX_adj); Rv = spinV * copysign(1.0f, joyX_adj);
        } else {
            spinV = max(0.0f, spinV - 1.0f * dt);
            Lv = constrain(v * (1.0f + 0.5f * joyX_adj), -Config::V_MAX, Config::V_MAX);
            Rv = constrain(v * (1.0f - 0.5f * joyX_adj), -Config::V_MAX, Config::V_MAX);
        }
        leftTrack.update(Lv, dt); rightTrack.update(Rv, dt);

        // [注入日志]：底盘维度
        LOG(CHASSIS_ONLY, "V_Tgt:%.2f, L_Spd:%.2f, R_Spd:%.2f, Rev:%d\n", v, leftTrack.currentSpeed, rightTrack.currentSpeed, reverseMode);
    }
    void stop() { v = 0; spinV = 0; leftTrack.stop(); rightTrack.stop(); }
};

// ==========================================
// 5. 炮塔双稳系统 (注入 TURRET_ONLY & IMU_RAW 日志)
// ==========================================
class TankTurret {
private:
    Adafruit_MPU6050& mpuC; 
    Adafruit_MPU6050& mpuT; 
    Servo pitchServo; 
    BLDCMotor yawMotor;
    BLDCDriver3PWM yawDriver;
    MagneticSensorI2C yawSensor;
    CascadePID yawPID;

    sensors_event_t aC, gC, tC;
    sensors_event_t aT, gT, tT;
    
    float currentPitchAngle = 90.0f;
    float savedPitch = 0, savedYawCont = 0;
    float yawContDeg = 0, yawRawLastDeg = 0;
    bool yawInitDone = false;
    float pitchFiltered = 0;

    bool switchState = false;
    float sensitivity = 1.0f;
    float gyroZ_offset = 0; 
    float gyroX_offset = 0; 

    float unwrapYaw(float currentYawDeg) {
        if (!yawInitDone) {
            yawRawLastDeg = currentYawDeg; yawContDeg = currentYawDeg;
            yawInitDone = true; return currentYawDeg;
        }
        float dyaw = currentYawDeg - yawRawLastDeg;
        if (dyaw > 180.0f) dyaw -= 360.0f;
        else if (dyaw < -180.0f) dyaw += 360.0f;
        yawContDeg += dyaw;
        yawRawLastDeg = currentYawDeg;
        return yawContDeg;
    }

public:
    TankTurret(Adafruit_MPU6050& c, Adafruit_MPU6050& t) 
        : mpuC(c), mpuT(t), yawMotor(7), 
          yawDriver(Config::FOC_PWM_A, Config::FOC_PWM_B, Config::FOC_PWM_C),
          yawSensor(AS5600_I2C),
          yawPID(CustomPID(2.2, 0.0, 0.5, 0.0, 15.0), 
                 CustomPID(0.18, 0.01, 0.002, 5.0, 8.0), 
                 0.6f) {}

    void init() {
        ESP32PWM::allocateTimer(2);
        pitchServo.setPeriodHertz(50);
        pitchServo.attach(Config::SERVO_PIN, 500, 2500);
        pitchServo.write(90);

        if (!mpuC.begin(0x68, &Wire1)) LOG_ALWAYS("Chassis MPU Fail\n");
        if (!mpuT.begin(0x69, &Wire1)) LOG_ALWAYS("Turret MPU Fail\n");
        
        mpuC.setGyroRange(MPU6050_RANGE_500_DEG);
        mpuC.setFilterBandwidth(MPU6050_BAND_21_HZ); 
        mpuT.setGyroRange(MPU6050_RANGE_500_DEG);
        mpuT.setFilterBandwidth(MPU6050_BAND_21_HZ);

        yawSensor.init();
        yawDriver.voltage_power_supply = 12.0;
        yawDriver.init();
        yawMotor.linkSensor(&yawSensor);
        yawMotor.linkDriver(&yawDriver);
        yawMotor.controller = MotionControlType::torque;
        yawMotor.init();
        yawMotor.initFOC();
    }

    void calibrate() {
        LOG_ALWAYS(">>> Calibrating IMU, Keep Static...\n");
        float sumZ = 0, sumX = 0;
        const int sampleCount = 500;
        for (int i = 0; i < sampleCount; i++) {
            mpuT.getEvent(&aT, &gT, &tT);
            sumZ += gT.gyro.z;
            sumX += gT.gyro.x;
            delay(2); 
        }
        gyroZ_offset = sumZ / (float)sampleCount;
        gyroX_offset = sumX / (float)sampleCount;
        LOG_ALWAYS("Done! Offsets - Z: %.4f, X: %.4f\n", gyroZ_offset, gyroX_offset);
    }

    void updateIMU() {
        mpuC.getEvent(&aC, &gC, &tC);
        mpuT.getEvent(&aT, &gT, &tT);

        // [注入日志]：传感器维度
        LOG(IMU_RAW, "GZ_C:%.3f, GZ_T_Raw:%.3f, GZ_T_Cal:%.3f\n", gC.gyro.z, gT.gyro.z, gT.gyro.z - gyroZ_offset);

        float gz_calibrated = gT.gyro.z - gyroZ_offset; 
        float gx_calibrated = gT.gyro.x - gyroX_offset; 

        float pitchAcc = atan2(aT.acceleration.y, aT.acceleration.z) * RAD_TO_DEG;
        pitchFiltered = 0.96f * (pitchFiltered + gx_calibrated * RAD_TO_DEG * 0.002f) + 0.04f * pitchAcc;
        
        static float accumulatedYaw = 0;
        accumulatedYaw += gz_calibrated * RAD_TO_DEG * 0.002f; 
        unwrapYaw(accumulatedYaw); 
    }

    void runFOC() {
        yawMotor.loopFOC();
        if (switchState) yawMotor.move(); 
        else { yawMotor.target = 0; yawMotor.move(0); }
    }

    void resetYawIntegral() {
        yawContDeg = 0; 
        yawRawLastDeg = 0;
        yawInitDone = false; 
    }

    void handleUI(bool aPressed, bool upPressed, bool downPressed, float joyX, float joyY, float dt) {
        static bool lastA = false;
        if (aPressed && !lastA) {
            switchState = !switchState;
            LOG_ALWAYS("Turret Stabilizer: %s\n", switchState ? "ON" : "OFF");
            if (switchState) {
                savedPitch = pitchFiltered; 
                savedYawCont = 0; 
                resetYawIntegral(); 
                yawPID.reset();
            }
        }
        lastA = aPressed;
        if (switchState) {
            if (abs(joyX) > 0.15f) savedYawCont += joyX * sensitivity * 85.0f * dt;
            if (abs(joyY) > 0.15f) savedPitch = constrain(savedPitch - joyY * sensitivity * 60.0f * dt, -15.0f, 30.0f);
        }
    }

    void updateStabilization(float dt) {
        if (!switchState) return;

        float pitchError = savedPitch - pitchFiltered;
        float pitchComp = (pitchError * 0.85f) - (gC.gyro.x * RAD_TO_DEG * 0.06f);
        currentPitchAngle = constrain(currentPitchAngle + pitchComp, 45.0f, 135.0f);
        pitchServo.write(currentPitchAngle);

        float yawGyroRad = gT.gyro.z; 
        float chassisTurnRateDeg = gC.gyro.z * RAD_TO_DEG;
        
        float yawVoltage = yawPID.calculate(savedYawCont, yawContDeg, yawGyroRad * RAD_TO_DEG, -chassisTurnRateDeg, dt);
        yawMotor.target = yawVoltage;

        // 这样在 VOFA+ 侧边栏会自动出现三个波形：Tgt, Real, Volt
        LOG(TURRET_ONLY, "Tgt:%.2f Real:%.2f Volt:%.2f\n", savedYawCont, yawContDeg, yawVoltage);
    }

    float getCurrentYaw() { return yawContDeg; }
    float getCurrentPitch() { return pitchFiltered; } 
};

// ==========================================
// 6. 顶层统筹
// ==========================================
class TankRobot {
private:
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
    Adafruit_MPU6050 mpuChassis; 
    Adafruit_MPU6050 mpuTurret;
    TankChassis chassis;
    TankTurret turret;
    uint32_t lastIMU = 0, lastUI = 0, lastCtrl = 0;

public:
    TankRobot() : 
        xboxController(Config::XBOX_MAC),
        turret(mpuChassis, mpuTurret) {}

    void setup() {
        #if GLOBAL_DEBUG
        Serial.begin(921600);
        #endif
        
        Wire.begin(Config::I2C_FOC_SDA, Config::I2C_FOC_SCL);
        Wire.setClock(400000); 
        Wire1.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL);
        Wire1.setClock(400000); 

        chassis.init();
        turret.init(); 
        delay(100);     
        turret.calibrate(); 

        xboxController.begin(); 
        LOG_ALWAYS(">>> System Ready. Current Debug Channel: %d\n", currentChannel);
    }

    void loop() {
        uint32_t nowMicros = micros();
        turret.runFOC(); 

        if (nowMicros - lastIMU >= 2000) { 
            lastIMU = nowMicros;
            turret.updateIMU();
        }

        if (nowMicros - lastUI >= 20000) { 
            float dtUI = (nowMicros - lastUI) * 1e-6f;
            lastUI = nowMicros;
            xboxController.onLoop();
            if (xboxController.isConnected()) {
                float jX = (xboxController.xboxNotif.joyRHori - 32767.5f) / 32767.5f;
                float jY = (xboxController.xboxNotif.joyRVert - 32767.5f) / 32767.5f;
                turret.handleUI(xboxController.xboxNotif.btnA, false, false, jX, jY, dtUI);
            }
        }

        if (nowMicros - lastCtrl >= 5000) { 
            float dtCtrl = (nowMicros - lastCtrl) * 1e-6f;
            lastCtrl = nowMicros;
            if (xboxController.isConnected()) {
                float trigL = xboxController.xboxNotif.trigLT / 1023.0f;
                float trigR = xboxController.xboxNotif.trigRT / 1023.0f;
                float joyLX = (xboxController.xboxNotif.joyLHori - 32767.5f) / 32767.5f;
                chassis.processKinematics(trigL, trigR, joyLX, dtCtrl);
                turret.updateStabilization(dtCtrl);
            } else { chassis.stop(); }
        }
    }
};

TankRobot robot;
void setup() { robot.setup(); }
void loop() { robot.loop(); }