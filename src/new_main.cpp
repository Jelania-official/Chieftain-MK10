#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#define PI 3.1415926
#include <AS201.h>
#include <Servo.h>
#include <SimpleFOC.h>
#include <driver/pcnt.h>

// ==========================================
// 1. 全局配置参数 (Config Namespace)
// ==========================================
namespace Config {
    // 手柄
    const char* XBOX_MAC = "28:ea:0b:d9:0b:9f";

    // 履带电机引脚
    const uint8_t R_IN1 = 25, R_IN2 = 33, R_PWM = 32;
    const uint8_t L_IN1 = 26, L_IN2 = 27, L_PWM = 14;
    const uint8_t PWM_CH_R = 0, PWM_CH_L = 1;
    const uint32_t PWM_FREQ = 10000;
    const uint8_t PWM_RES = 8;

    // 编码器引脚及参数
    const uint8_t R_ENCA = 23, R_ENCB = 22;
    const uint8_t L_ENCA = 35, L_ENCB = 34;
    const int ENCODER_PPR = 7;
    const int GEAR_RATIO = 59;
    const float RPM_TO_MS = 13.3 / 540.0;

    // 底盘运动参数
    const float V_MAX = 14.5;
    const float V_REVERSE_MAX = 6.0;
    const float BRAKE_COEFF = 1.5;
    const float RT_THRESHOLD = 0.2;
    const uint32_t RT_HOLD_TIME = 2000;

    // 炮塔双稳参数
    const uint8_t SERVO_PIN = 15;
    const uint8_t FOC_PWM_A = 21, FOC_PWM_B = 19, FOC_PWM_C = 18;
}

// ==========================================
// 2. 基础控制算法类
// ==========================================
class CustomPID {
public:
    float kp, ki, kd, maxOut, maxI;
    float integral = 0, prevError = 0;

    CustomPID(float p, float i, float d, float maxI, float maxOut) 
        : kp(p), ki(i), kd(d), maxOut(maxOut), maxI(maxI) {}

    float calculate(float target, float actual, float dt) {
        if (dt <= 0.0f) dt = 0.001f;
        float error = target - actual;
        
        integral += error * dt;
        integral = constrain(integral, -maxI, maxI);
        
        float derivative = (error - prevError) / dt;
        prevError = error;
        
        float output = kp * error + ki * integral + kd * derivative;
        return constrain(output, -maxOut, maxOut);
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
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

    void reset() {
        outer.reset();
        inner.reset();
    }
};

// ==========================================
// 3. 硬件抽象类
// ==========================================
class DCMotor {
private:
    uint8_t in1, in2, pwmPin, pwmCh;
    bool isLeft;

public:
    DCMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, uint8_t ch, bool left = false)
        : in1(pin1), in2(pin2), pwmPin(pwmPin), pwmCh(ch), isLeft(left) {}

    void init() {
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        ledcSetup(pwmCh, Config::PWM_FREQ, Config::PWM_RES);
        ledcAttachPin(pwmPin, pwmCh);
        stop();
    }

    void drive(float output) {
        output = constrain(output, -255, 255);
        
        // 如果是最小启动速度保护，可以加在这里
        if (abs(output) < 35 && output != 0) {
            output = copysign(35, output);
        }

        // 左右轮对称性处理
        float speed = output;

        if (speed > 0) {
            digitalWrite(in1, isLeft ? HIGH : LOW);
            digitalWrite(in2, isLeft ? LOW : HIGH);
            ledcWrite(pwmCh, speed);
        } else if (speed < 0) {
            digitalWrite(in1, isLeft ? LOW : HIGH);
            digitalWrite(in2, isLeft ? HIGH : LOW);
            ledcWrite(pwmCh, -speed);
        } else {
            stop();
        }
    }

    void stop() {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(pwmCh, 0);
    }
};

class CustomEncoder {
private:
    pcnt_unit_t unit;
    int16_t lastCount = 0;
    uint32_t lastTime = 0;

public:
    CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit) : unit(p_unit) {
        pcnt_config_t cfg;
        cfg.pulse_gpio_num = pinA;
        cfg.ctrl_gpio_num = pinB;
        cfg.channel = PCNT_CHANNEL_0;
        cfg.unit = unit;
        cfg.pos_mode = PCNT_COUNT_INC;
        cfg.neg_mode = PCNT_COUNT_DEC;
        cfg.lctrl_mode = PCNT_MODE_KEEP;
        cfg.hctrl_mode = PCNT_MODE_REVERSE;
        cfg.counter_h_lim = 32767;
        cfg.counter_l_lim = -32768;
        
        pcnt_unit_config(&cfg);
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
    }

    void init() {
        pcnt_counter_pause(unit);
        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);
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
// 4. 子系统封装类
// ==========================================

// --- 履带总成 ---
class TankTrack {
public:
    DCMotor motor;
    CustomEncoder encoder;
    CustomPID pid;
    float currentSpeed = 0;
    float targetSpeed = 0;

    TankTrack(DCMotor m, CustomEncoder e, CustomPID p) 
        : motor(m), encoder(e), pid(p) {}

    void init() {
        motor.init();
        encoder.init();
    }

    void update(float target, float dt) {
        targetSpeed = target;
        currentSpeed = encoder.getSpeedMS();
        
        if (abs(targetSpeed) < 0.015f) targetSpeed = 0;
        
        float output = pid.calculate(targetSpeed, currentSpeed, dt);
        motor.drive(output);
    }

    void stop() {
        targetSpeed = 0;
        pid.reset();
        motor.stop();
    }
};

// --- 底盘运动控制 ---
class TankChassis {
private:
    TankTrack rightTrack;
    TankTrack leftTrack;
    
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

    void init() {
        rightTrack.init();
        leftTrack.init();
    }

    void processKinematics(float triggerL, float triggerR, float joyX, float dt) {
        uint32_t now = millis();
        bool isStopped = (abs(v) < 0.1f);

        // 1. 倒车模式状态机
        if (triggerR > Config::RT_THRESHOLD) {
            if (rtPressedStartTime == 0) rtPressedStartTime = now;
            else if (isStopped && (now - rtPressedStartTime >= Config::RT_HOLD_TIME)) {
                reverseMode = true;
                v = 0;
            }
        } else {
            rtPressedStartTime = 0;
            if (v > -0.1f && reverseMode && triggerL > Config::RT_THRESHOLD) {
                reverseMode = false;
            }
        }

        // 2. 纵向油门解算
        float a = 0.0f;
        float rollResist = (abs(v) < 3.0f) ? 0.5f : 0.338f;
        
        if (!reverseMode) {
            a = 1.4f * (triggerL * triggerL) - 0.006f * v * v - rollResist - Config::BRAKE_COEFF * triggerR;
            if (v < 0.5f) a *= 0.5f;
            v = constrain(v + a * dt, 0, Config::V_MAX);
        } else {
            a = -1.4f * (triggerR * triggerR) + 0.006f * v * v + rollResist + Config::BRAKE_COEFF * triggerL;
            v = constrain(v + a * dt, -Config::V_REVERSE_MAX, 0);
        }

        // 3. 转向与自转解算
        float deadZone = 4000.0f / 32767.5f;
        if (abs(joyX) < deadZone) joyX = 0;

        bool isSpinMode = (abs(v) < 0.1f && abs(joyX) > 0.2f);
        float Lv = 0, Rv = 0;

        if (isSpinMode) {
            float spinA = 1.4f * abs(joyX) - 0.5f;
            spinV = constrain(spinV + spinA * dt, 0, 1.22f);
            float dir = copysign(1.0f, joyX);
            Lv = -spinV * dir;
            Rv =  spinV * dir;
        } else {
            if (spinV > 0) {
                spinV -= 1.0f * dt; // 摩擦力衰减
                if (spinV < 0) spinV = 0;
            }
            float turnFactor = 0.5f;
            float turnRatio = constrain(joyX, -1.0f, 1.0f);
            Lv = v * (1.0f + turnFactor * turnRatio);
            Rv = v * (1.0f - turnFactor * turnRatio);
            
            float maxV = reverseMode ? Config::V_REVERSE_MAX : Config::V_MAX;
            Lv = constrain(Lv, -maxV, maxV);
            Rv = constrain(Rv, -maxV, maxV);
        }

        // 4. 发送给底层
        leftTrack.update(Lv, dt);
        rightTrack.update(Rv, dt);
    }

    void stop() {
        v = 0; spinV = 0;
        leftTrack.stop();
        rightTrack.stop();
    }
    
    float getLv() { return leftTrack.currentSpeed; }
    float getRv() { return rightTrack.currentSpeed; }
};

// --- 炮塔双稳控制 ---
class TankTurret {
private:
    AS201& imuChassis;
    AS201& imuTurret;
    
    Servo pitchServo;
    BLDCMotor yawMotor;
    BLDCDriver3PWM yawDriver;
    MagneticSensorI2C yawSensor;

    CascadePID yawPID;
    
    // 俯仰PID状态
    float servoPosInt = 0, servoVelInt = 0;
    float servoPosErrLast = 0, servoVelErrLast = 0;
    float controlAngle = 0;

    float radYawGyroLpf = 0;
    float dPosLpf = 0, dVelLpf = 0;

    bool switchState = false;
    bool yawInitDone = false;
    float yawRawLastRad = 0, yawContRad = 0;
    
    float savedPitch = 0, savedYawCont = 0;
    float sensitivity = 1.0f;

    float unwrapYaw(float currentYawDeg) {
        float radNow = currentYawDeg * DEG_TO_RAD;
        if (!yawInitDone) {
            yawRawLastRad = radNow;
            yawContRad = radNow;
            yawInitDone = true;
            return currentYawDeg;
        }
        float dyaw = radNow - yawRawLastRad;
        if (dyaw > PI) dyaw -= 2.0f * PI;
        else if (dyaw < -PI) dyaw += 2.0f * PI;
        
        yawContRad += dyaw;
        yawRawLastRad = radNow;
        return yawContRad * RAD_TO_DEG;
    }

public:
    TankTurret(AS201& chassis, AS201& turret) 
        : imuChassis(chassis), imuTurret(turret),
          yawMotor(7), 
          yawDriver(Config::FOC_PWM_A, Config::FOC_PWM_B, Config::FOC_PWM_C),
          yawSensor(AS5600_I2C),
          yawPID(CustomPID(1.0, 0.0, 1.0, 0.0, 12.0),   // Outer
                 CustomPID(0.1, 0.01, 0.0, 1000.0, 6.0)) // Inner
    {}

    void init() {
        pitchServo.attach(Config::SERVO_PIN);
        
        yawDriver.voltage_power_supply = 12.0;
        yawDriver.init();
        
        yawSensor.init();
        yawMotor.linkSensor(&yawSensor);
        yawMotor.linkDriver(&yawDriver);
        yawMotor.voltage_limit = 8.0;
        yawMotor.controller = MotionControlType::torque;
        
        yawMotor.PID_velocity.P = 0;
        yawMotor.PID_velocity.I = 0;
        yawMotor.PID_velocity.D = 0;
        yawMotor.LPF_velocity.Tf = 0.01;
        
        yawMotor.init();
        yawMotor.initFOC();
    }

    void updateIMU() {
        imuChassis.update();
        imuTurret.update();
    }

    // FOC 极高频调用
    void runFOC() {
        yawMotor.loopFOC();
        yawMotor.move();
    }

    void handleUI(bool aPressed, bool upPressed, bool downPressed, float joyX, float joyY, float dt) {
        static bool lastA = false;
        
        // 双稳开关切换
        if (aPressed && !lastA) {
            switchState = !switchState;
            if (switchState) {
                savedPitch = imuTurret.getData().roll;
                savedYawCont = unwrapYaw(imuTurret.getData().yaw);
            }
        }
        lastA = aPressed;

        if (switchState) {
            // 灵敏度调整
            if (upPressed) sensitivity = min(sensitivity + 0.1f, 2.0f);
            if (downPressed) sensitivity = max(sensitivity - 0.1f, 0.3f);

            // 右摇杆姿态积分
            float deadZone = 4000.0f / 32767.5f;
            if (abs(joyX) < deadZone) joyX = 0;
            if (abs(joyY) < deadZone) joyY = 0;

            float yawVel = joyX * sensitivity * 60.0f;     // °/s
            float pitchVel = -joyY * sensitivity * 40.0f;  // °/s

            savedYawCont += yawVel * dt;
            savedPitch = constrain(savedPitch + pitchVel * dt, -10.0f, 35.0f);
        }
    }

    void updateStabilization(float dt) {
        if (!switchState) {
            pitchServo.writeMicroseconds(1500); // 居中
            controlAngle = 0;
            servoPosInt = servoVelInt = 0;
            // 停用时最好把滤波器也清零，防止再次开启时产生大跳变
            dPosLpf = 0; 
            dVelLpf = 0; 
            radYawGyroLpf = 0;
            
            yawPID.reset();
            yawMotor.target = 0;
            return;
        }

        // ==========================================
        // 1. 俯仰控制 (Pitch Servo - 双环PID)
        // ==========================================
        float pitchTurret = imuTurret.getData().roll;
        float gyroChassisPitch = imuChassis.getData().gx;
        float gyroTurretPitch = imuTurret.getData().gx;

        // [位置环]
        float errPos = savedPitch - pitchTurret;
        servoPosInt = constrain(servoPosInt + errPos * dt, -50.0f, 50.0f);
        float dPos = (errPos - servoPosErrLast) / dt;
        dPosLpf += (dPos - dPosLpf) * dt / (0.02f + dt); // 计算位置环 LPF
        servoPosErrLast = errPos;
        float uPos = 2.0f * errPos + 0.0f * servoPosInt + 0.2f * dPosLpf; // ✅ 使用 dPosLpf

        // [速度环]
        float targetVel = -gyroChassisPitch + uPos;
        float errVel = targetVel - gyroTurretPitch;
        servoVelInt = constrain(servoVelInt + errVel * dt, -50.0f, 50.0f);
        float dVel = (errVel - servoVelErrLast) / dt;
        dVelLpf += (dVel - dVelLpf) * dt / (0.02f + dt); // ✅ 补上速度环 LPF
        servoVelErrLast = errVel;
        float uVel = 1.2f * errVel + 0.0f * servoVelInt + 0.05f * dVelLpf; // ✅ 使用 dVelLpf

        // [映射舵机 PWM]
        controlAngle = constrain(controlAngle + uVel * dt, -10.0f, 30.0f);
        float pwm_float = 500.0f + ((controlAngle + 90.0f) / 180.0f) * 2000.0f;
        pitchServo.writeMicroseconds((int)pwm_float);


        // ==========================================
        // 2. 偏航控制 (Yaw BLDC - 串级PID + 前馈)
        // ==========================================
        float yawNowCont = unwrapYaw(imuTurret.getData().yaw);
        float radYawNow = yawNowCont * DEG_TO_RAD;
        float radYawRef = savedYawCont * DEG_TO_RAD;
        
        // 读取陀螺仪并滤波
        float radYawGyro = imuTurret.getData().gz * DEG_TO_RAD; 
        radYawGyroLpf += (radYawGyro - radYawGyroLpf) * dt / (0.01f + dt); // ✅ 偏航陀螺仪 LPF
        
        float radYawChassisGyro = -imuChassis.getData().gz * DEG_TO_RAD;

        // 传入滤波后的 radYawGyroLpf 给串级 PID 计算
        float outputYaw = yawPID.calculate(radYawRef, radYawNow, radYawGyroLpf, radYawChassisGyro, dt);
        yawMotor.target = constrain(outputYaw, -yawMotor.voltage_limit, yawMotor.voltage_limit);
    }
    
    float getCurrentYaw() { return unwrapYaw(imuTurret.getData().yaw); }
};

// ==========================================
// 5. 顶层统筹主类
// ==========================================
class TankRobot {
private:
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
    AS201 imuChassis;
    AS201 imuTurret;
    
    TankChassis chassis;
    TankTurret turret;

    uint32_t lastIMU = 0, lastUI = 0, lastCtrl = 0, lastPrint = 0;

public:
    TankRobot() : 
        xboxController(Config::XBOX_MAC),
        imuChassis(16, 17, Serial2),
        imuTurret(2, 4, Serial1),
        turret(imuChassis, imuTurret) {}

    void setup() {
        Serial.begin(921600);
        Serial1.begin(115200, SERIAL_8N1, 2, 4);
        Serial2.begin(115200, SERIAL_8N1, 16, 17);

        xboxController.begin();
        chassis.init();
        turret.init();
    }

    void loop() {
        uint32_t nowMicros = micros();

        // 1. 最高优先级：FOC更新 (不阻塞)
        turret.runFOC();

        // 2. IMU更新 (500Hz)
        if (nowMicros - lastIMU >= 2000) {
            lastIMU = nowMicros;
            turret.updateIMU();
        }

        // 3. 用户输入与状态机 (50Hz)
        if (nowMicros - lastUI >= 20000) {
            float dtUI = (nowMicros - lastUI) * 1e-6f;
            lastUI = nowMicros;

            xboxController.onLoop();
            
            if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification()) {
                float joyX = (xboxController.xboxNotif.joyRHori - 32767.5f) / 32767.5f;
                float joyY = (xboxController.xboxNotif.joyRVert - 32767.5f) / 32767.5f;
                turret.handleUI(
                    xboxController.xboxNotif.btnA,
                    xboxController.xboxNotif.btnDirUp,
                    xboxController.xboxNotif.btnDirDown,
                    joyX, joyY, dtUI
                );
            } else if (xboxController.getCountFailedConnection() > 2) {
                ESP.restart(); // 掉线重启保护
            }
        }

        // 4. 控制主循环 (200Hz)
        if (nowMicros - lastCtrl >= 5000) {
            float dtCtrl = (nowMicros - lastCtrl) * 1e-6f;
            lastCtrl = nowMicros;

            if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification()) {
                float trigL = xboxController.xboxNotif.trigLT / 1023.0f;
                float trigR = xboxController.xboxNotif.trigRT / 1023.0f;
                float joyLX = (xboxController.xboxNotif.joyLHori - 32767.5f) / 32767.5f;
                
                chassis.processKinematics(trigL, trigR, joyLX, dtCtrl);
            } else {
                chassis.stop();
            }

            turret.updateStabilization(dtCtrl);
        }

        // 5. 调试输出 (5Hz)
        uint32_t nowMillis = nowMicros / 1000;
        if (nowMillis - lastPrint >= 200) {
            lastPrint = nowMillis;
            Serial.printf("L: %.2f | R: %.2f | Yaw: %.2f\n", 
                          chassis.getLv(), chassis.getRv(), turret.getCurrentYaw());
        }
    }
};

// ==========================================
// 6. Arduino 标准入口
// ==========================================
TankRobot robot;

void setup() {
    robot.setup();
}

void loop() {
    robot.loop();
}