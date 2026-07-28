#include "TankTurret.h"

portMUX_TYPE turretStateMux = portMUX_INITIALIZER_UNLOCKED;

namespace {
constexpr uint8_t MPU6050_CHASSIS_ADDR = 0x68;
constexpr uint8_t MPU6050_TURRET_ADDR = 0x69;
constexpr uint8_t MPU6050_ACCEL_OUT_REG = 0x3B;
constexpr size_t MPU6050_FRAME_BYTES = 14;
constexpr float MPU6050_ACCEL_LSB_PER_G_4G = 8192.0f;
constexpr float MPU6050_GYRO_LSB_PER_DPS_500 = 65.5f;

int16_t decodeInt16(const uint8_t* bytes) {
        return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    }
}

// 下面这些工具函数围绕炮塔角度安全展开：角度折算、后甲板避让、AS5600 原始角读取。

// 角度统一折算到 [-180, 180]，方便做“是不是在车体后方”的几何判断。
float TankTurret::wrapAngle180(float angleDeg) const {
        angleDeg = fmod(angleDeg, 360.0f);
        if (angleDeg > 180.0f) angleDeg -= 360.0f;
        if (angleDeg < -180.0f) angleDeg += 360.0f;
        return angleDeg;
    }

// 根据“相对车体后方”的夹角，计算当前允许的最低俯角。
    // 正常时允许到 GUN_PITCH_MIN，接近发动机舱时逐渐抬到 REAR_DECK_SAFE_PITCH。
float TankTurret::getRearDeckMinPitch(float yawDeg) {
        float yawWrapped = wrapAngle180(yawDeg);
        float rearOffset = abs(abs(yawWrapped) - Config::REAR_DECK_CENTER_YAW);

        if (rearOffset >= Config::REAR_DECK_AVOID_START) return Config::GUN_PITCH_MIN;
        if (rearOffset <= Config::REAR_DECK_AVOID_FULL) return Config::REAR_DECK_SAFE_PITCH;

        float span = Config::REAR_DECK_AVOID_START - Config::REAR_DECK_AVOID_FULL;
        float blend = (Config::REAR_DECK_AVOID_START - rearOffset) / span;
        blend = pow(constrain(blend, 0.0f, 1.0f), Config::REAR_DECK_BLEND_EXP);
        return Config::GUN_PITCH_MIN + (Config::REAR_DECK_SAFE_PITCH - Config::GUN_PITCH_MIN) * blend;
    }

float TankTurret::protectPitchForRearDeck(float pitchDeg, float yawDeg) {
        return constrain(pitchDeg, getRearDeckMinPitch(yawDeg), Config::GUN_PITCH_MAX);
    }

// 直接从 AS5600 读取 12-bit 机械角，换算成 0~360 度。
bool TankTurret::readAS5600MechanicalDeg(float& angleDeg) {
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

bool TankTurret::readMpu6050Event(uint8_t address, sensors_event_t& accel,
                                 sensors_event_t& gyro, sensors_event_t& temp) {
        Wire1.beginTransmission(address);
        Wire1.write(MPU6050_ACCEL_OUT_REG);
        if (Wire1.endTransmission(false) != 0) return false;

        size_t received = Wire1.requestFrom(address, (uint8_t)MPU6050_FRAME_BYTES, (uint8_t)true);
        if (received != MPU6050_FRAME_BYTES || Wire1.available() < (int)MPU6050_FRAME_BYTES) {
            while (Wire1.available() > 0) Wire1.read();
            return false;
        }

        uint8_t frame[MPU6050_FRAME_BYTES] = {};
        for (size_t i = 0; i < MPU6050_FRAME_BYTES; ++i) {
            frame[i] = (uint8_t)Wire1.read();
        }

        int16_t rawAx = decodeInt16(&frame[0]);
        int16_t rawAy = decodeInt16(&frame[2]);
        int16_t rawAz = decodeInt16(&frame[4]);
        int16_t rawTemp = decodeInt16(&frame[6]);
        int16_t rawGx = decodeInt16(&frame[8]);
        int16_t rawGy = decodeInt16(&frame[10]);
        int16_t rawGz = decodeInt16(&frame[12]);

        const float accelScale = SENSORS_GRAVITY_STANDARD / MPU6050_ACCEL_LSB_PER_G_4G;
        const float gyroScale = DEG_TO_RAD / MPU6050_GYRO_LSB_PER_DPS_500;
        accel.acceleration.x = rawAx * accelScale;
        accel.acceleration.y = rawAy * accelScale;
        accel.acceleration.z = rawAz * accelScale;
        gyro.gyro.x = rawGx * gyroScale;
        gyro.gyro.y = rawGy * gyroScale;
        gyro.gyro.z = rawGz * gyroScale;
        temp.temperature = (rawTemp / 340.0f) + 36.53f;
        uint32_t timestampMs = millis();
        accel.timestamp = timestampMs;
        gyro.timestamp = timestampMs;
        temp.timestamp = timestampMs;

        return isfinite(accel.acceleration.x) &&
               isfinite(accel.acceleration.y) &&
               isfinite(accel.acceleration.z) &&
               isfinite(gyro.gyro.x) &&
               isfinite(gyro.gyro.y) &&
               isfinite(gyro.gyro.z) &&
               isfinite(temp.temperature);
    }

bool TankTurret::validateImuSample(const sensors_event_t& accel, float gxDeg,
                                   float gzDeg, float& accelNorm,
                                   float& accelPitchDeg) const {
        accelNorm = sqrtf(
            accel.acceleration.x * accel.acceleration.x +
            accel.acceleration.y * accel.acceleration.y +
            accel.acceleration.z * accel.acceleration.z);
        accelPitchDeg = atan2f(accel.acceleration.y, accel.acceleration.z) * RAD_TO_DEG;

        return isfinite(accelNorm) &&
               accelNorm >= Config::IMU_ACCEL_NORM_MIN_MPS2 &&
               accelNorm <= Config::IMU_ACCEL_NORM_MAX_MPS2 &&
               isfinite(accelPitchDeg) &&
               validateImuEvent(gxDeg) &&
               validateImuEvent(gzDeg);
    }

float TankTurret::calculateAccelTrust(float accelNorm, float accelPitchDeg,
                                      float gyroPredictionDeg) const {
        float gravityDeviationG = abs(accelNorm - SENSORS_GRAVITY_STANDARD) /
                                  SENSORS_GRAVITY_STANDARD;
        float normSpan = Config::IMU_ACCEL_TRUST_ZERO_DEVIATION_G -
                         Config::IMU_ACCEL_TRUST_FULL_DEVIATION_G;
        float normTrust = 1.0f - constrain(
            (gravityDeviationG - Config::IMU_ACCEL_TRUST_FULL_DEVIATION_G) / normSpan,
            0.0f, 1.0f);

        float innovationDeg = abs(wrapAngle180(accelPitchDeg - gyroPredictionDeg));
        float innovationSpan = Config::IMU_ACCEL_INNOVATION_ZERO_TRUST_DEG -
                               Config::IMU_ACCEL_INNOVATION_FULL_TRUST_DEG;
        float innovationTrust = 1.0f - constrain(
            (innovationDeg - Config::IMU_ACCEL_INNOVATION_FULL_TRUST_DEG) / innovationSpan,
            0.0f, 1.0f);

        return min(normTrust, innovationTrust);
    }

void TankTurret::setImuSensorHealthy(bool chassisSensor, bool healthy) {
        portENTER_CRITICAL(&turretStateMux);
        if (chassisSensor) {
            chassisImuHealthy = healthy;
        } else {
            turretImuHealthy = healthy;
        }
        // 炮塔 IMU 是 pitch/yaw 稳定闭环的主反馈；底盘 IMU 故障只关闭其前馈和坡度补偿。
        if (!chassisSensor && !healthy) {
            switchState = false;
            pendingYawTarget = 0.0f;
        }
        portEXIT_CRITICAL(&turretStateMux);
    }

void TankTurret::initializeImuHealth(ImuHealthState& state, bool chassisSensor,
                                     bool healthy, uint32_t nowUs) {
        state.healthy = healthy;
        state.consecutiveFailures = 0;
        state.consecutiveSuccesses = healthy ? Config::IMU_RECOVERY_CONFIRM_SAMPLES : 0;
        state.lastValidSampleUs = healthy ? nowUs : 0;
        setImuSensorHealthy(chassisSensor, healthy);
    }

void TankTurret::updateImuHealth(ImuHealthState& state, bool chassisSensor,
                                 bool sampleValid, uint32_t nowUs) {
        if (sampleValid) {
            state.lastValidSampleUs = nowUs;
            state.consecutiveFailures = 0;
            if (state.consecutiveSuccesses < Config::IMU_RECOVERY_CONFIRM_SAMPLES) {
                ++state.consecutiveSuccesses;
            }
            if (!state.healthy &&
                state.consecutiveSuccesses >= Config::IMU_RECOVERY_CONFIRM_SAMPLES) {
                state.healthy = true;
                setImuSensorHealthy(chassisSensor, true);
                LOG_ALWAYS(">>> %s IMU recovered.\n", chassisSensor ? "Chassis" : "Turret");
            }
            return;
        }

        state.consecutiveSuccesses = 0;
        if (state.consecutiveFailures < Config::IMU_FAILURE_CONFIRM_SAMPLES) {
            ++state.consecutiveFailures;
        }
        bool stale = state.lastValidSampleUs == 0 ||
                     ((uint32_t)(nowUs - state.lastValidSampleUs) > Config::IMU_STALE_US);
        if (state.healthy &&
            (state.consecutiveFailures >= Config::IMU_FAILURE_CONFIRM_SAMPLES || stale)) {
            state.healthy = false;
            setImuSensorHealthy(chassisSensor, false);
            LOG_ALWAYS("!!! %s IMU unhealthy: failures=%u stale=%d.\n",
                       chassisSensor ? "Chassis" : "Turret",
                       state.consecutiveFailures, stale ? 1 : 0);
        }
    }

void TankTurret::setStabilizationEnabled(bool enabled) {
        portENTER_CRITICAL(&turretStateMux);
        // 堵转锁存不能被普通的“开启稳定器”路径绕过；必须先走 A 键安全复位。
        switchState = enabled && !yawStallLatched;
        if (!enabled) pendingYawTarget = 0.0f;
        portEXIT_CRITICAL(&turretStateMux);
    }

bool TankTurret::isStabilizationEnabled() const {
        bool enabled;
        portENTER_CRITICAL(&turretStateMux);
        enabled = switchState;
        portEXIT_CRITICAL(&turretStateMux);
        return enabled;
    }

void TankTurret::updateYawSensorCache(bool healthy, float sensorDeg) {
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

bool TankTurret::readFreshYawSensorDeg(float& sensorDeg) const {
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

bool TankTurret::controlSensorsHealthy() const {
        bool turretImuOk, yawOk;
        uint32_t updatedUs;

        portENTER_CRITICAL(&turretStateMux);
        turretImuOk = turretImuHealthy;
        yawOk = yawSensorHealthy;
        updatedUs = lastYawSensorUpdateUs;
        portEXIT_CRITICAL(&turretStateMux);

        return turretImuOk && yawOk &&
               ((uint32_t)(micros() - updatedUs) <= Config::YAW_SENSOR_STALE_US);
    }

void TankTurret::publishYawTarget(float targetVoltage) {
        portENTER_CRITICAL(&turretStateMux);
        pendingYawTarget = targetVoltage;
        portEXIT_CRITICAL(&turretStateMux);
    }

bool TankTurret::readFocCommand(float& targetVoltage) const {
        bool enabled, turretImuOk, yawOk;
        uint32_t updatedUs;

        portENTER_CRITICAL(&turretStateMux);
        enabled = switchState;
        turretImuOk = turretImuHealthy;
        yawOk = yawSensorHealthy;
        updatedUs = lastYawSensorUpdateUs;
        targetVoltage = pendingYawTarget;
        portEXIT_CRITICAL(&turretStateMux);

        if (!enabled || !turretImuOk || !yawOk ||
            ((uint32_t)(micros() - updatedUs) > Config::YAW_SENSOR_STALE_US)) {
            targetVoltage = 0.0f;
            return false;
        }
        return true;
    }

// FOC 核心持续刷新 AS5600 机械角缓存，控制核心只读缓存，避免跨核争用 I2C。
bool TankTurret::isYawSensorFresh() const {
        float unused;
        return readFreshYawSensorDeg(unused);
    }

void TankTurret::resetYawStallCandidate() {
        yawStallCandidateActive = false;
        yawStallCandidateSinceMs = 0;
        yawStallCandidateStartDeg = 0.0f;
    }

void TankTurret::clearManualRateCommands() {
        yawManualRateCmdDps = 0.0f;
        pitchManualRateCmdDps = 0.0f;
    }

void TankTurret::latchYawStall(float targetVoltage, float positionErrorDeg) {
        portENTER_CRITICAL(&turretStateMux);
        yawStallLatched = true;
        switchState = false;
        pendingYawTarget = 0.0f;
        portEXIT_CRITICAL(&turretStateMux);

        yawPID.reset();
        clearManualRateCommands();
        resetYawStallCandidate();
        LOG_ALWAYS("!!! Yaw stall latched: voltage=%.2fV error=%.2fdeg. Remove obstruction, center yaw stick, then press A to re-arm.\n",
                   targetVoltage, positionErrorDeg);
    }

bool TankTurret::updateYawStallProtection(float targetVoltage, float positionErrorDeg) {
        float sensorDeg = 0.0f;
        if (abs(targetVoltage) < Config::YAW_STALL_VOLTAGE_MIN ||
            abs(positionErrorDeg) < Config::YAW_STALL_ERROR_MIN_DEG ||
            !readFreshYawSensorDeg(sensorDeg)) {
            resetYawStallCandidate();
            return false;
        }

        uint32_t nowMs = millis();
        if (!yawStallCandidateActive) {
            yawStallCandidateActive = true;
            yawStallCandidateSinceMs = nowMs;
            yawStallCandidateStartDeg = sensorDeg;
            return false;
        }

        // 只要候选窗口内确实移动超过阈值，就从当前位置重新观察，不把慢速运动误判成堵转。
        float travelDeg = abs(wrapAngle180(sensorDeg - yawStallCandidateStartDeg));
        if (travelDeg > Config::YAW_STALL_MAX_TRAVEL_DEG) {
            yawStallCandidateSinceMs = nowMs;
            yawStallCandidateStartDeg = sensorDeg;
            return false;
        }

        if ((uint32_t)(nowMs - yawStallCandidateSinceMs) < Config::YAW_STALL_CONFIRM_MS) {
            return false;
        }

        latchYawStall(targetVoltage, positionErrorDeg);
        return true;
    }

float TankTurret::getTurretRelativeYawDegFromSensor() {
        float sensorDeg;
        if (!readFreshYawSensorDeg(sensorDeg)) return Config::REAR_DECK_CENTER_YAW;
        return wrapAngle180((sensorDeg - Config::TURRET_FRONT_SENSOR_OFFSET) * Config::TURRET_SENSOR_SIGN);
    }

bool TankTurret::validateImuEvent(float valueDegPerSec) const {
        return isfinite(valueDegPerSec) && abs(valueDegPerSec) <= Config::IMU_GYRO_SANITY_DPS;
    }

// 构造函数只绑定对象和配置参数，真正访问硬件的动作放在 init()。
TankTurret::TankTurret(Adafruit_MPU6050& c, Adafruit_MPU6050& t)
        : mpuC(c), mpuT(t), yawMotor(7),
          yawDriver(Config::FOC_PWM_A, Config::FOC_PWM_B, Config::FOC_PWM_C),
          yawSensor(AS5600_I2C),
          yawPID(CustomPID(Config::YAW_OUTER_KP, 0.0f, Config::YAW_OUTER_KD, 0.0f, Config::YAW_OUTER_RATE_MAX),
                 CustomPID(Config::YAW_INNER_KP, Config::YAW_INNER_KI, Config::YAW_INNER_KD, 5.0f, Config::YAW_VOLTAGE_MAX),
                 Config::YAW_CHASSIS_FF_GAIN) {}

// 初始化顺序很重要：舵机先到中位，再检查传感器，最后启动 yaw FOC。
bool TankTurret::init() {
        ESP32PWM::allocateTimer(2);
        pitchServo.setPeriodHertz(333);
        pitchServo.attach(Config::SERVO_PIN, 500, 2500);
        pitchServo.write(90);

        bool chassisOk = mpuC.begin(MPU6050_CHASSIS_ADDR, &Wire1);
        bool turretOk = mpuT.begin(MPU6050_TURRET_ADDR, &Wire1);
        if (!chassisOk || !turretOk) {
            LOG_ALWAYS("!!! MPU6050 init failed: chassis=%d turret=%d\n", chassisOk, turretOk);
            ready = false;
            return false;
        }
        // 两颗 IMU 使用同一套量程/滤波配置，避免底盘和炮塔姿态数据尺度不一致。
        // 陀螺仪保持 ±500deg/s：覆盖炮塔/车体快速转动，同时比 ±1000/2000 保留更好分辨率。
        mpuC.setGyroRange(MPU6050_RANGE_500_DEG);
        mpuT.setGyroRange(MPU6050_RANGE_500_DEG);

        // 加速度计用 ±4g：比默认 ±2g 更能承受履带震动和碰撞冲击，不容易饱和。
        mpuC.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpuT.setAccelerometerRange(MPU6050_RANGE_4_G);

        // DLPF 44Hz：先把电机/履带的高频噪声滤掉，再交给互补滤波和后续姿态算法。
        mpuC.setFilterBandwidth(MPU6050_BAND_44_HZ);
        mpuT.setFilterBandwidth(MPU6050_BAND_44_HZ);

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
void TankTurret::calibrate() {
        if (!ready) return;
        LOG_ALWAYS(">>> Calibrating IMUs (%d samples), Keep Static...\n", Config::IMU_CALIB_SAMPLES);
        float sumT_Z = 0, sumT_X = 0, sumC_Z = 0, sumC_X = 0;
        int validTurretSamples = 0;
        int validChassisSamples = 0;
        const int maxAttempts = Config::IMU_CALIB_SAMPLES * 2;

        for (int attempt = 0;
             attempt < maxAttempts &&
             (validTurretSamples < Config::IMU_CALIB_SAMPLES ||
              validChassisSamples < Config::IMU_CALIB_SAMPLES);
             ++attempt) {
            bool turretReadOk = readMpu6050Event(MPU6050_TURRET_ADDR, aT, gT, tT);
            bool chassisReadOk = readMpu6050Event(MPU6050_CHASSIS_ADDR, aC, gC, tC);

            if (turretReadOk && validTurretSamples < Config::IMU_CALIB_SAMPLES) {
                sumT_Z += gT.gyro.z;
                sumT_X += gT.gyro.x;
                ++validTurretSamples;
            }
            if (chassisReadOk && validChassisSamples < Config::IMU_CALIB_SAMPLES) {
                sumC_Z += gC.gyro.z;
                sumC_X += gC.gyro.x;
                ++validChassisSamples;
            }
            delay(2);
        }

        bool turretCalibrated = validTurretSamples == Config::IMU_CALIB_SAMPLES;
        bool chassisCalibrated = validChassisSamples == Config::IMU_CALIB_SAMPLES;
        turretImuCalibrated = turretCalibrated;
        chassisImuCalibrated = chassisCalibrated;

        if (turretCalibrated) {
            t_gyroZ_offset = sumT_Z / (float)validTurretSamples;
            t_gyroX_offset = sumT_X / (float)validTurretSamples;
            pitchFiltered = atan2f(aT.acceleration.y, aT.acceleration.z) * RAD_TO_DEG;
            yawContDeg = 0.0f;
        }
        if (chassisCalibrated) {
            c_gyroZ_offset = sumC_Z / (float)validChassisSamples;
            c_gyroX_offset = sumC_X / (float)validChassisSamples;
            chassisPitchFiltered = atan2f(aC.acceleration.y, aC.acceleration.z) * RAD_TO_DEG;
        }

        uint32_t nowUs = micros();
        initializeImuHealth(turretImuState, false, turretCalibrated, nowUs);
        initializeImuHealth(chassisImuState, true, chassisCalibrated, nowUs);

        if (!turretCalibrated || !chassisCalibrated) {
            LOG_ALWAYS("!!! IMU calibration failed: chassis=%d/%d turret=%d/%d.\n",
                       validChassisSamples, Config::IMU_CALIB_SAMPLES,
                       validTurretSamples, Config::IMU_CALIB_SAMPLES);
            setStabilizationEnabled(false);
            publishYawTarget(0.0f);
            return;
        }
        yawPID.reset();
        setStabilizationEnabled(false);
        clearManualRateCommands();
        savedPitch = pitchFiltered;
        savedYawCont = yawContDeg;

        // 校准完成后，炮管上下“点头”一下
        pitchServo.write(105);
        delay(200);
        pitchServo.write(90);
        LOG_ALWAYS(">>> Calib Done!\n");
    }

// 同时更新底盘 IMU 和炮塔 IMU。底盘姿态给底盘动力学和炮塔前馈使用；
// 炮管/炮塔姿态给稳定控制闭环使用。
void TankTurret::updateIMU(float dt) {
        if (!ready) return;
        uint32_t nowUs = micros();
        if (dt <= 0.0f || dt > Config::IMU_MAX_DT) {
            updateImuHealth(chassisImuState, true, false, nowUs);
            updateImuHealth(turretImuState, false, false, nowUs);
            return;
        }
        bool chassisReadOk = readMpu6050Event(MPU6050_CHASSIS_ADDR, aC, gC, tC);
        bool turretReadOk = readMpu6050Event(MPU6050_TURRET_ADDR, aT, gT, tT);

        float nextCGxDeg = 0.0f, nextCGzDeg = 0.0f;
        float chassisAccelNorm = 0.0f, chassisPitchAcc = 0.0f;
        float nextChassisPitch = chassisPitchFiltered;
        bool chassisSampleValid = false;
        if (chassisReadOk && chassisImuCalibrated) {
            nextCGxDeg = (gC.gyro.x - c_gyroX_offset) * RAD_TO_DEG;
            nextCGzDeg = (gC.gyro.z - c_gyroZ_offset) * RAD_TO_DEG;
            chassisSampleValid = validateImuSample(
                aC, nextCGxDeg, nextCGzDeg, chassisAccelNorm, chassisPitchAcc);
            if (chassisSampleValid) {
                float gyroPrediction = chassisPitchFiltered + nextCGxDeg * dt;
                float accelTrust = calculateAccelTrust(
                    chassisAccelNorm, chassisPitchAcc, gyroPrediction);
                float baseBlend = dt / (Config::CHASSIS_PITCH_ACC_TAU_S + dt);
                nextChassisPitch = gyroPrediction +
                    (baseBlend * accelTrust * wrapAngle180(chassisPitchAcc - gyroPrediction));
                chassisSampleValid = isfinite(nextChassisPitch);
            }
        }
        updateImuHealth(chassisImuState, true, chassisSampleValid, nowUs);
        if (chassisSampleValid) {
            c_gx_cal_deg = nextCGxDeg;
            c_gz_cal_deg = nextCGzDeg;
            chassisPitchFiltered = nextChassisPitch;
        }

        float nextTGxDeg = 0.0f, nextTGzDeg = 0.0f;
        float turretAccelNorm = 0.0f, turretPitchAcc = 0.0f;
        float nextPitch = pitchFiltered;
        float nextYawCont = yawContDeg;
        bool turretSampleValid = false;
        if (turretReadOk && turretImuCalibrated) {
            nextTGxDeg = (gT.gyro.x - t_gyroX_offset) * RAD_TO_DEG;
            float turretGzCal = gT.gyro.z - t_gyroZ_offset;
            if (abs(turretGzCal) < 0.005f) turretGzCal = 0.0f;
            nextTGzDeg = turretGzCal * RAD_TO_DEG;
            turretSampleValid = validateImuSample(
                aT, nextTGxDeg, nextTGzDeg, turretAccelNorm, turretPitchAcc);
            if (turretSampleValid) {
                float gyroPrediction = pitchFiltered + nextTGxDeg * dt;
                float accelTrust = calculateAccelTrust(
                    turretAccelNorm, turretPitchAcc, gyroPrediction);
                float baseBlend = dt / (Config::PITCH_ACC_TAU + dt);
                nextPitch = gyroPrediction +
                    (baseBlend * accelTrust * wrapAngle180(turretPitchAcc - gyroPrediction));
                nextYawCont = yawContDeg + nextTGzDeg * dt;
                turretSampleValid = isfinite(nextPitch) && isfinite(nextYawCont);
            }
        }
        updateImuHealth(turretImuState, false, turretSampleValid, nowUs);
        if (turretSampleValid) {
            t_gx_cal_deg = nextTGxDeg;
            t_gz_cal_deg = nextTGzDeg;
            pitchFiltered = nextPitch;
            yawContDeg = nextYawCont;
        }
    }

// 获取已经算好的底盘俯仰速率
float TankTurret::getLatestChassisPitchRate() {
        return chassisImuIsHealthy() && isfinite(c_gx_cal_deg) ? c_gx_cal_deg : 0.0f;
    }

float TankTurret::getLatestChassisYawRate() {
        return chassisImuIsHealthy() && isfinite(c_gz_cal_deg) ? c_gz_cal_deg : 0.0f;
    }

// 获取当前坡度角
float TankTurret::getChassisPitchAngle() {
        return chassisImuIsHealthy() && isfinite(chassisPitchFiltered) ? chassisPitchFiltered : 0.0f;
    }

float TankTurret::getPitchTargetDeg() const {
        return savedPitch;
    }

float TankTurret::getPitchActualDeg() const {
        return pitchFiltered;
    }

float TankTurret::getPitchServoDeg() const {
        return currentPitchAngle;
    }

float TankTurret::getYawTargetDeg() const {
        return savedYawCont;
    }

float TankTurret::getYawActualDeg() const {
        return yawContDeg;
    }

float TankTurret::getYawRelativeDeg() {
        return getTurretRelativeYawDegFromSensor();
    }

float TankTurret::getYawVoltageTarget() const {
        float targetVoltage;
        portENTER_CRITICAL(&turretStateMux);
        targetVoltage = pendingYawTarget;
        portEXIT_CRITICAL(&turretStateMux);
        return targetVoltage;
    }

bool TankTurret::stabilizationActive() const {
        return isStabilizationEnabled();
    }

bool TankTurret::chassisImuIsHealthy() const {
        bool healthy;
        portENTER_CRITICAL(&turretStateMux);
        healthy = chassisImuHealthy;
        portEXIT_CRITICAL(&turretStateMux);
        return healthy;
    }

bool TankTurret::turretImuIsHealthy() const {
        bool healthy;
        portENTER_CRITICAL(&turretStateMux);
        healthy = turretImuHealthy;
        portEXIT_CRITICAL(&turretStateMux);
        return healthy;
    }

bool TankTurret::imuIsHealthy() const {
        bool chassisHealthy, turretHealthy;
        portENTER_CRITICAL(&turretStateMux);
        chassisHealthy = chassisImuHealthy;
        turretHealthy = turretImuHealthy;
        portEXIT_CRITICAL(&turretStateMux);
        return chassisHealthy && turretHealthy;
    }

bool TankTurret::yawSensorIsHealthy() const {
        bool healthy;
        portENTER_CRITICAL(&turretStateMux);
        healthy = yawSensorHealthy;
        portEXIT_CRITICAL(&turretStateMux);
        return healthy;
    }

bool TankTurret::yawStallIsLatched() const {
        bool latched;
        portENTER_CRITICAL(&turretStateMux);
        latched = yawStallLatched;
        portEXIT_CRITICAL(&turretStateMux);
        return latched;
    }

// Core 0 高频入口：持续运行 SimpleFOC，并把控制线程发布的电压目标送给 yaw 电机。
void TankTurret::runFOC() {
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

void TankTurret::enterSafeState() {
        // 低压或上层安全状态不等于传感器故障，不覆盖真实 IMU 健康状态。
        setStabilizationEnabled(false);
        yawPID.reset();
        clearManualRateCommands();
        resetYawStallCandidate();
        publishYawTarget(0.0f);
    }

// 断连和故障分开处理：断连不等于传感器坏了，只是立即停止执行目标。
void TankTurret::enterDisconnectedState() {
        setStabilizationEnabled(false);
        yawPID.reset();
        clearManualRateCommands();
        resetYawStallCandidate();
        publishYawTarget(0.0f);
    }

// A 键作为稳定器总开关；炮塔 IMU 和 AS5600 健康即可进入稳定模式。
// 底盘 IMU 不健康时仍可用炮塔自身反馈闭环，只是暂时没有车体角速度前馈。
void TankTurret::handleUI(bool aPressed, float joyX, float joyY) {
        if (!ready) return;
        static bool lastA = false;
        if (aPressed && !lastA) {
            if (yawStallIsLatched()) {
                // 操作者必须先让 yaw 摇杆回中；恢复时以当前位置为新目标，避免追赶堵转前的旧目标。
                if (abs(joyX) <= Config::YAW_STALL_REARM_JOY_MAX && controlSensorsHealthy()) {
                    portENTER_CRITICAL(&turretStateMux);
                    yawStallLatched = false;
                    portEXIT_CRITICAL(&turretStateMux);
                    savedPitch = pitchFiltered;
                    savedYawCont = yawContDeg;
                    yawPID.reset();
                    clearManualRateCommands();
                    resetYawStallCandidate();
                    publishYawTarget(0.0f);
                    setStabilizationEnabled(true);
                    LOG_ALWAYS(">>> Yaw stall cleared: current pose captured, stabilization re-enabled.\n");
                } else {
                    LOG_ALWAYS("!!! Yaw stall clear refused: center yaw stick and confirm turret IMU/AS5600 are healthy.\n");
                }
            } else if (isStabilizationEnabled()) {
                setStabilizationEnabled(false);
                yawPID.reset();
                clearManualRateCommands();
                resetYawStallCandidate();
                publishYawTarget(0.0f);
            } else if (controlSensorsHealthy()) {
                setStabilizationEnabled(true);
                savedPitch = pitchFiltered;
                savedYawCont = yawContDeg;
                yawPID.reset();
                clearManualRateCommands();
                resetYawStallCandidate();
            }
        }
        lastA = aPressed;
        if (isStabilizationEnabled()) {
            // 50Hz UI 只保存速度请求；目标角由 200Hz 控制环连续积分，避免每 20ms 一次的位置台阶。
            yawManualRateCmdDps = 0.0f;
            pitchManualRateCmdDps = 0.0f;
            if (abs(joyX) > 0.15f) {
                // 计算有效推力比例：0.15时为0，1.0时为1.0
                float effectiveX = copysign((abs(joyX) - 0.15f) / 0.85f, joyX);
                yawManualRateCmdDps = effectiveX * Config::REAL_TURRET_VEL;
            }
            if (abs(joyY) > 0.15f) {
                // 使用同样的线性映射：消除 0.15 处的突变跳变
                float effectiveY = copysign((abs(joyY) - 0.15f) / 0.85f, joyY);
                // 注意：joyY 通常向上推是负值，向下推是正值，请根据你的操作习惯确认符号
                pitchManualRateCmdDps = -effectiveY * Config::REAL_TURRET_VEL;
            }
        } else {
            clearManualRateCommands();
        }
    }

// Core 1 稳定控制入口：根据保存的世界系目标，更新 pitch 舵机和 yaw 电压目标。
void TankTurret::updateStabilization(float dt) {
        if (!ready || !isStabilizationEnabled()) return;
        if (!controlSensorsHealthy()) {
            enterSafeState();
            return;
        }

        // 俯仰双稳：底盘抬头会立刻通过前馈向下补，位置误差再由 P 环慢慢拉回。
        bool chassisFeedbackAvailable = chassisImuIsHealthy();
        float chassisPitchRateDeg = chassisFeedbackAvailable ? c_gx_cal_deg : 0.0f;
        float chassisYawRateDeg = chassisFeedbackAvailable ? c_gz_cal_deg : 0.0f;

        // 目标生成和闭环现在同为 200Hz。满摇杆时每次只推进约 0.1125°，而不是 50Hz 的 0.45°台阶。
        savedYawCont += yawManualRateCmdDps * dt;
        savedPitch = constrain(savedPitch + pitchManualRateCmdDps * dt,
                               Config::GUN_PITCH_MIN, Config::GUN_PITCH_MAX);

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
        float yVoltage = yawPID.calculate(savedYawCont, yawContDeg, yawManualRateCmdDps,
                                          t_gz_cal_deg, -chassisYawRateDeg, dt);
        if (updateYawStallProtection(yVoltage, savedYawCont - yawContDeg)) {
            return;
        }
        publishYawTarget(yVoltage);

        LOG(TURRET_ONLY, "Y_Tgt:%.2f, Y_Real:%.2f, Y_RelSens:%.2f\n", savedYawCont, yawContDeg, getTurretRelativeYawDegFromSensor());
    }

bool TankTurret::isReady() const { return ready; }

bool TankTurret::isHealthy() const {
        return ready && imuIsHealthy() && yawSensorIsHealthy() && isYawSensorFresh();
    }
