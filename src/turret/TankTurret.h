#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <SimpleFOC.h>
#include "../config/DebugLog.h"
#include "../config/RobotConfig.h"
#include "../control/Controllers.h"

extern portMUX_TYPE turretStateMux;

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
          yawPID(CustomPID(Config::YAW_OUTER_KP, 0.0f, Config::YAW_OUTER_KD, 0.0f, Config::YAW_OUTER_RATE_MAX),
                 CustomPID(Config::YAW_INNER_KP, Config::YAW_INNER_KI, Config::YAW_INNER_KD, 5.0f, Config::YAW_VOLTAGE_MAX),
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

