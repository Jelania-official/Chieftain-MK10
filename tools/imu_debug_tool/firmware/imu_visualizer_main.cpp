#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "config/RobotConfig.h"
#include "sensors/Mpu6x00Compat.h"

namespace {
constexpr uint8_t kChassisImuAddress = 0x68;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kFusionPeriodUs = 5000;      // 200Hz 姿态融合
constexpr uint32_t kTelemetryPeriodMs = 20;     // 50Hz USB 串口输出
constexpr uint32_t kStartupI2cTimeoutMs = 20;   // 启动诊断时允许总线恢复
constexpr int kGyroCalibSamples = 1200;
constexpr int kStressTestSamples = 1000;
constexpr float kRadToDeg = 57.2957795131f;
constexpr uint8_t kAccelOutRegister = 0x3B;
constexpr size_t kFrameBytes = 14;
constexpr float kAccelLsbPerG4G = 8192.0f;
constexpr float kGyroLsbPerDps500 = 65.5f;

uint8_t activeMpuAddress = kChassisImuAddress;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

uint32_t lastFusionUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t runtimeReadFailuresTotal = 0;
uint32_t runtimeConsecutiveFailures = 0;

struct ImuFrame {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;       // m/s^2
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;       // rad/s，已减零偏
    float temp = 0.0f;                           // Celsius
};

struct EulerDeg {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

class MahonyImuFusion {
public:
    void resetFromAccel(float ax, float ay, float az) {
        float roll = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
        setEulerRad(roll, pitch, 0.0f);
        integralX = integralY = integralZ = 0.0f;
    }

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        if (dt <= 0.0f || dt > 0.1f) return;

        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (!isfinite(norm) || norm < 0.001f) {
            integrateGyro(gx, gy, gz, dt);
            return;
        }

        ax /= norm;
        ay /= norm;
        az /= norm;

        // 当前四元数预测出的重力方向。误差项用"测得重力"和"预测重力"的叉乘得到。
        float halfVx = q1 * q3 - q0 * q2;
        float halfVy = q0 * q1 + q2 * q3;
        float halfVz = q0 * q0 - 0.5f + q3 * q3;

        float halfEx = ay * halfVz - az * halfVy;
        float halfEy = az * halfVx - ax * halfVz;
        float halfEz = ax * halfVy - ay * halfVx;

        if (twoKi > 0.0f) {
            integralX += twoKi * halfEx * dt;
            integralY += twoKi * halfEy * dt;
            integralZ += twoKi * halfEz * dt;
            gx += integralX;
            gy += integralY;
            gz += integralZ;
        }

        gx += twoKp * halfEx;
        gy += twoKp * halfEy;
        gz += twoKp * halfEz;

        integrateGyro(gx, gy, gz, dt);
    }

    EulerDeg eulerDeg() const {
        EulerDeg e;
        e.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        1.0f - 2.0f * (q1 * q1 + q2 * q2)) * kRadToDeg;

        float sinPitch = 2.0f * (q0 * q2 - q3 * q1);
        sinPitch = constrain(sinPitch, -1.0f, 1.0f);
        e.pitch = asinf(sinPitch) * kRadToDeg;

        e.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                       1.0f - 2.0f * (q2 * q2 + q3 * q3)) * kRadToDeg;
        return e;
    }

    void quaternion(float& w, float& x, float& y, float& z) const {
        w = q0; x = q1; y = q2; z = q3;
    }

private:
    float twoKp = 1.2f;
    float twoKi = 0.02f;

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float integralX = 0.0f, integralY = 0.0f, integralZ = 0.0f;

    void setEulerRad(float roll, float pitch, float yaw) {
        float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);

        q0 = cr * cp * cy + sr * sp * sy;
        q1 = sr * cp * cy - cr * sp * sy;
        q2 = cr * sp * cy + sr * cp * sy;
        q3 = cr * cp * sy - sr * sp * cy;
        normalize();
    }

    void integrateGyro(float gx, float gy, float gz, float dt) {
        gx *= 0.5f * dt;
        gy *= 0.5f * dt;
        gz *= 0.5f * dt;

        float qa = q0;
        float qb = q1;
        float qc = q2;

        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += ( qa * gx + qc * gz - q3 * gy);
        q2 += ( qa * gy - qb * gz + q3 * gx);
        q3 += ( qa * gz + qb * gy - qc * gx);
        normalize();
    }

    void normalize() {
        float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        if (!isfinite(norm) || norm < 0.001f) {
            q0 = 1.0f; q1 = q2 = q3 = 0.0f;
            return;
        }
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
};

MahonyImuFusion fusion;
ImuFrame latest;

bool readRegister(uint8_t address, uint8_t reg, uint8_t& value, bool repeatedStart = false) {
    return Mpu6x00Compat::readRegister(Wire, address, reg, value, repeatedStart);
}

bool diagnoseWhoAmI(uint32_t clockHz, bool repeatedStart = false) {
    Wire.setClock(clockHz);
    int validReads = 0;
    int supportedReads = 0;
    uint8_t firstSupportedId = 0;
    bool consistentId = true;
    const char* mode = repeatedStart ? "restart" : "stop";

    for (int attempt = 1; attempt <= 5; ++attempt) {
        uint8_t value = 0;
        bool ok = readRegister(kChassisImuAddress, Mpu6x00Compat::WHO_AM_I_REG,
                               value, repeatedStart);
        if (ok) {
            ++validReads;
            if (Mpu6x00Compat::isSupportedId(value)) {
                ++supportedReads;
                if (firstSupportedId == 0) firstSupportedId = value;
                if (value != firstSupportedId) consistentId = false;
            }
            Serial.printf("INFO,who_am_i,%lu,%s,attempt_%d,0x%02X\n",
                          (unsigned long)clockHz, mode, attempt, value);
        } else {
            Serial.printf("ERR,who_am_i_read_failed,%lu,%s,attempt_%d\n",
                          (unsigned long)clockHz, mode, attempt);
        }
        delay(5);
    }

    Serial.printf("INFO,who_am_i_summary,%lu,%s,valid_%d,supported_%d,id_0x%02X\n",
                  (unsigned long)clockHz, mode, validReads, supportedReads,
                  firstSupportedId);
    // 切换时钟后的第一次交易可能因总线恢复而超时。
    // 4/5 次均为同一个支持的 ID 已足以确认身份；后续压力测试
    // 仍会严格统计每一次传输错误。
    return validReads >= 4 && supportedReads == validReads && consistentId;
}

void restartI2c(uint32_t clockHz) {
    Wire.end();
    delay(10);
    Wire.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL);
    Wire.setClock(clockHz);
    Wire.setTimeOut(kStartupI2cTimeoutMs);
    delay(10);
}

bool readImu(ImuFrame& out) {
    uint8_t frame[kFrameBytes] = {};
    if (!Mpu6x00Compat::readRegisters(
            Wire, activeMpuAddress, kAccelOutRegister, frame, kFrameBytes)) {
        return false;
    }
    auto decode = [&](size_t index) -> int16_t {
        return (int16_t)(((uint16_t)frame[index] << 8) | frame[index + 1]);
    };

    float accelScale = SENSORS_GRAVITY_STANDARD / kAccelLsbPerG4G;
    float gyroScale = DEG_TO_RAD / kGyroLsbPerDps500;
    out.ax = decode(0) * accelScale;
    out.ay = decode(2) * accelScale;
    out.az = decode(4) * accelScale;
    out.temp = (decode(6) / 340.0f) + 36.53f;
    out.gx = decode(8) * gyroScale - gyroBiasX;
    out.gy = decode(10) * gyroScale - gyroBiasY;
    out.gz = decode(12) * gyroScale - gyroBiasZ;

    return isfinite(out.ax) && isfinite(out.ay) && isfinite(out.az) &&
           isfinite(out.gx) && isfinite(out.gy) && isfinite(out.gz);
}

struct StressResult {
    int transportFailures = 0;
    int invalidSamples = 0;
    int maxConsecutiveFailures = 0;
};

StressResult runI2cStressTest(uint32_t clockHz) {
    Wire.setClock(clockHz);
    delay(20);

    StressResult result;
    int consecutiveFailures = 0;
    ImuFrame sample;
    for (int i = 0; i < kStressTestSamples; ++i) {
        if (!readImu(sample)) {
            ++result.transportFailures;
            ++consecutiveFailures;
            result.maxConsecutiveFailures = max(
                result.maxConsecutiveFailures, consecutiveFailures);
            continue;
        }

        float accelNorm = sqrtf(
            sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
        bool plausible = isfinite(accelNorm) && accelNorm >= 1.0f && accelNorm <= 50.0f &&
                         fabsf(sample.gx * kRadToDeg) <= Config::IMU_GYRO_SANITY_DPS &&
                         fabsf(sample.gy * kRadToDeg) <= Config::IMU_GYRO_SANITY_DPS &&
                         fabsf(sample.gz * kRadToDeg) <= Config::IMU_GYRO_SANITY_DPS &&
                         sample.temp >= -40.0f && sample.temp <= 125.0f;
        if (!plausible) ++result.invalidSamples;
        consecutiveFailures = 0;
    }

    Serial.printf(
        "INFO,i2c_stress,%lu,total_%d,transport_fail_%d,invalid_%d,max_consecutive_%d\n",
        (unsigned long)clockHz, kStressTestSamples, result.transportFailures,
        result.invalidSamples, result.maxConsecutiveFailures);
    return result;
}

void calibrateGyro() {
    Serial.printf("INFO,calibrating_gyro,%d\n", kGyroCalibSamples);

    double sx = 0.0, sy = 0.0, sz = 0.0;
    
    // 临时清零零偏，直接读取原始陀螺仪值；失败样本不进入标定均值。
    float savedBiasX = gyroBiasX;
    float savedBiasY = gyroBiasY;
    float savedBiasZ = gyroBiasZ;
    gyroBiasX = gyroBiasY = gyroBiasZ = 0.0f;
    
    ImuFrame sample;
    int validSamples = 0;
    for (int attempt = 0;
         attempt < kGyroCalibSamples * 2 && validSamples < kGyroCalibSamples;
         ++attempt) {
        if (!readImu(sample)) {
            delay(2);
            continue;
        }
        sx += sample.gx;
        sy += sample.gy;
        sz += sample.gz;
        ++validSamples;
        delay(2);
    }

    if (validSamples != kGyroCalibSamples) {
        gyroBiasX = savedBiasX;
        gyroBiasY = savedBiasY;
        gyroBiasZ = savedBiasZ;
        Serial.printf("ERR,gyro_calibration_failed,%d,%d\n", validSamples, kGyroCalibSamples);
        return;
    }

    gyroBiasX = sx / validSamples;
    gyroBiasY = sy / validSamples;
    gyroBiasZ = sz / validSamples;

    if (readImu(latest)) {
        fusion.resetFromAccel(latest.ax, latest.ay, latest.az);
    }

    Serial.printf("INFO,gyro_bias,%.7f,%.7f,%.7f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void handleSerialCommand() {
    static char line[32] = {};
    static size_t len = 0;

    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            line[len] = '\0';
            if (strcmp(line, "ZERO") == 0) {
                readImu(latest);
                fusion.resetFromAccel(latest.ax, latest.ay, latest.az);
                Serial.println("INFO,zeroed");
            } else if (strcmp(line, "CAL") == 0) {
                calibrateGyro();
            }
            len = 0;
            continue;
        }

        if (len < sizeof(line) - 1) {
            line[len++] = c;
        } else {
            len = 0;
        }
    }
}

void sendTelemetry(float dt) {
    float qw, qx, qy, qz;
    fusion.quaternion(qw, qx, qy, qz);
    EulerDeg e = fusion.eulerDeg();

    Serial.printf(
        "IMU,%lu,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.2f\n",
        (unsigned long)millis(),
        dt,
        latest.ax, latest.ay, latest.az,
        latest.gx * kRadToDeg, latest.gy * kRadToDeg, latest.gz * kRadToDeg,
        qw, qx, qy, qz,
        e.roll, e.pitch, e.yaw,
        latest.temp
    );
}
}

void setup() {
    Serial.begin(kSerialBaud);
    delay(1000);
    
    Serial.println("\n\nINFO,imu_visualizer_start");
    Serial.println("INFO,checking_config");
    Serial.printf("INFO,i2c_sda_pin,%u\n", Config::I2C_IMU_SDA);
    Serial.printf("INFO,i2c_scl_pin,%u\n", Config::I2C_IMU_SCL);
    Serial.printf("INFO,mpu_addr,0x%02X\n", kChassisImuAddress);
    
    // 改用 Wire (I2C0)，GPIO21/22 是 I2C0 的默认引脚
    Serial.println("INFO,init_i2c");
    Wire.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(kStartupI2cTimeoutMs);
    
    // 扫描 I2C 总线
    Serial.println("INFO,scanning_i2c_bus");
    bool found_any = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("INFO,i2c_device_found,0x%02X\n", addr);
            found_any = true;
        }
        delay(1);
    }
    
    if (!found_any) {
        Serial.println("ERR,no_i2c_devices_found");
        Serial.println("ERR,check_wiring_and_power");
    }
    Serial.println("INFO,i2c_scan_complete");
    
    // 尝试连接 MPU6050
    Serial.println("INFO,trying_mpu6050");

    // 地址 ACK 只证明总线上存在从机；经典版 ID=0x68，商家新版兼容 ID=0x70。
    // 先在整车使用的 400kHz 下验证，再用 100kHz 区分信号完整性问题。
    bool whoAmIStable = diagnoseWhoAmI(400000);
    if (!whoAmIStable) {
        Serial.println("INFO,retrying_who_am_i_at_100khz");
        whoAmIStable = diagnoseWhoAmI(100000);
    }
    if (!whoAmIStable) {
        Serial.println("ERR,who_am_i_not_stable_supported_id");
    }

    // 诊断期间出现过超时时，ESP32 I2C 控制器可能保留错误状态。
    // 正式配置传感器前重建驱动，避免诊断本身导致后续全部读取失败。
    restartI2c(100000);
    Mpu6x00Compat::InitResult initResult =
        Mpu6x00Compat::initialize(Wire, kChassisImuAddress);
    if (!initResult.ok) {
        uint8_t altAddress = (kChassisImuAddress == 0x68) ? 0x69 : 0x68;
        Serial.printf("INFO,trying_alt_addr_0x%02X\n", altAddress);
        initResult = Mpu6x00Compat::initialize(Wire, altAddress);
        if (initResult.ok) activeMpuAddress = altAddress;
    } else {
        activeMpuAddress = kChassisImuAddress;
    }

    if (!initResult.ok) {
        Serial.printf("ERR,mpu_init_failed,stage_%s,last_id_0x%02X\n",
                      Mpu6x00Compat::stageName(initResult.failedStage),
                      initResult.whoAmI);
        Serial.println("ERR,mpu6050_not_found");
        Serial.println("ERR,check_connections");
        while (true) {
            delay(1000);
            Serial.println("ERR,waiting_for_mpu6050");
        }
    }

    Serial.printf("INFO,mpu_compatible_found,address_0x%02X,id_0x%02X\n",
                  activeMpuAddress, initResult.whoAmI);
    Serial.println("INFO,mpu_configured,rate_200hz,accel_4g,gyro_500dps,dlpf_44hz");

    StressResult stress100 = runI2cStressTest(100000);
    StressResult stress400 = runI2cStressTest(400000);
    int errors100 = stress100.transportFailures + stress100.invalidSamples;
    int errors400 = stress400.transportFailures + stress400.invalidSamples;
    uint32_t selectedClock = errors400 <= errors100 ? 400000 : 100000;
    Wire.setClock(selectedClock);
    Serial.printf("INFO,i2c_selected_clock,%lu\n", (unsigned long)selectedClock);
    if (min(errors100, errors400) > 0) {
        Serial.println("WARN,i2c_stress_not_clean");
    }

    calibrateGyro();
    runtimeReadFailuresTotal = 0;
    runtimeConsecutiveFailures = 0;
    lastFusionUs = micros();
    lastTelemetryMs = millis();
    Serial.println("INFO,ready");
}

void loop() {
    handleSerialCommand();

    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastFusionUs) < kFusionPeriodUs) return;

    float dt = (nowUs - lastFusionUs) * 1e-6f;
    lastFusionUs = nowUs;

    if (!readImu(latest)) {
        ++runtimeReadFailuresTotal;
        ++runtimeConsecutiveFailures;
        if (runtimeConsecutiveFailures == 1) {
            Serial.printf("WARN,imu_runtime_read_failed,consecutive_1,total_%lu\n",
                          (unsigned long)runtimeReadFailuresTotal);
        } else if (runtimeConsecutiveFailures == 3) {
            Serial.printf("ERR,imu_runtime_unhealthy,consecutive_3,total_%lu\n",
                          (unsigned long)runtimeReadFailuresTotal);
        }
        // 不再把上一帧缓存冒充为新数据发给上位机。
        return;
    }

    if (runtimeConsecutiveFailures > 0) {
        Serial.printf("INFO,imu_runtime_recovered,previous_consecutive_%lu,total_%lu\n",
                      (unsigned long)runtimeConsecutiveFailures,
                      (unsigned long)runtimeReadFailuresTotal);
        runtimeConsecutiveFailures = 0;
    }
    fusion.update(latest.gx, latest.gy, latest.gz, latest.ax, latest.ay, latest.az, dt);

    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastTelemetryMs) >= kTelemetryPeriodMs) {
        lastTelemetryMs = nowMs;
        sendTelemetry(dt);
    }
}
