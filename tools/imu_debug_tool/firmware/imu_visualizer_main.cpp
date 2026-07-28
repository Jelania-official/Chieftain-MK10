#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "config/RobotConfig.h"

namespace {
constexpr uint8_t kChassisImuAddress = 0x68;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kFusionPeriodUs = 5000;      // 200Hz 姿态融合
constexpr uint32_t kTelemetryPeriodMs = 20;     // 50Hz USB 串口输出
constexpr int kGyroCalibSamples = 1200;
constexpr float kRadToDeg = 57.2957795131f;
constexpr uint8_t kAccelOutRegister = 0x3B;
constexpr size_t kFrameBytes = 14;
constexpr float kAccelLsbPerG4G = 8192.0f;
constexpr float kGyroLsbPerDps500 = 65.5f;

Adafruit_MPU6050 mpu;
uint8_t activeMpuAddress = kChassisImuAddress;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

uint32_t lastFusionUs = 0;
uint32_t lastTelemetryMs = 0;

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

bool readImu(ImuFrame& out) {
    Wire.beginTransmission(activeMpuAddress);
    Wire.write(kAccelOutRegister);
    if (Wire.endTransmission(false) != 0) return false;

    size_t received = Wire.requestFrom(activeMpuAddress, (uint8_t)kFrameBytes, (uint8_t)true);
    if (received != kFrameBytes || Wire.available() < (int)kFrameBytes) {
        while (Wire.available() > 0) Wire.read();
        return false;
    }

    uint8_t frame[kFrameBytes] = {};
    for (size_t i = 0; i < kFrameBytes; ++i) frame[i] = (uint8_t)Wire.read();
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
    Wire.setTimeOut(Config::IMU_I2C_TIMEOUT_MS);
    
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
    
    bool mpu_ok = false;
    if (mpu.begin(kChassisImuAddress, &Wire)) {
        Serial.printf("INFO,mpu6050_found_at_0x%02X\n", kChassisImuAddress);
        activeMpuAddress = kChassisImuAddress;
        mpu_ok = true;
    } else {
        uint8_t alt_addr = (kChassisImuAddress == 0x68) ? 0x69 : 0x68;
        Serial.printf("INFO,trying_alt_addr_0x%02X\n", alt_addr);
        if (mpu.begin(alt_addr, &Wire)) {
            Serial.printf("INFO,mpu6050_found_at_0x%02X\n", alt_addr);
            activeMpuAddress = alt_addr;
            mpu_ok = true;
        }
    }
    
    if (!mpu_ok) {
        Serial.println("ERR,mpu6050_not_found");
        Serial.println("ERR,check_connections");
        while (true) {
            delay(1000);
            Serial.println("ERR,waiting_for_mpu6050");
        }
    }

    // 配置 MPU6050
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    
    Serial.println("INFO,mpu6050_configured");

    calibrateGyro();
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

    if (readImu(latest)) {
        fusion.update(latest.gx, latest.gy, latest.gz, latest.ax, latest.ay, latest.az, dt);
    }

    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastTelemetryMs) >= kTelemetryPeriodMs) {
        lastTelemetryMs = nowMs;
        sendTelemetry(dt);
    }
}
