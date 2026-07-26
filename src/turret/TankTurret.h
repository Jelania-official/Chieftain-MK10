#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <SimpleFOC.h>
#include "config/DebugLog.h"
#include "config/RobotConfig.h"
#include "control/Controllers.h"

// Core 0 的 FOC 任务和 Core 1 的控制任务会共享炮塔状态，用这个锁保护共享变量。
extern portMUX_TYPE turretStateMux;

// ==========================================
// 5. 炮塔双稳系统
// ==========================================
// TankTurret 负责三件事：
// 1. 读取底盘 IMU、炮塔 IMU 和 AS5600 yaw 编码器；
// 2. 维护炮管 pitch 目标和炮塔 yaw 世界系目标；
// 3. 在稳定模式开启时输出舵机角度和 yaw FOC 目标电压。
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
    float t_gyroZ_offset = 0, t_gyroX_offset = 0; // 炮塔 IMU 零偏
    float c_gyroZ_offset = 0, c_gyroX_offset = 0; // 底盘 IMU 零偏
    float t_gx_cal_deg = 0.0f; // 炮管 pitch 角速度，deg/s
    float t_gz_cal_deg = 0.0f; // 炮塔 yaw 角速度，deg/s
    float c_gx_cal_deg = 0.0f; // 底盘 pitch 角速度，deg/s
    float c_gz_cal_deg = 0.0f; // 底盘 yaw 角速度，deg/s
    float chassisPitchFiltered = 0.0f; // 底盘坡度角，deg
    bool ready = false;

    // 以下变量会被 FOC 任务和主控制任务共享，因此读写时要进临界区。
    volatile bool imuHealthy = false;
    volatile bool yawSensorHealthy = false;
    volatile float cachedTurretMechYawDeg = 0.0f;
    volatile uint32_t lastYawSensorUpdateUs = 0;
    volatile float pendingYawTarget = 0.0f;
    uint32_t lastYawSensorCheckUs = 0;

    float wrapAngle180(float angleDeg);
    float getRearDeckMinPitch(float yawDeg);
    float protectPitchForRearDeck(float pitchDeg, float yawDeg);

    // 直接从 AS5600 读取 0~360 度机械角。失败时返回 false。
    bool readAS5600MechanicalDeg(float& angleDeg);

    void setImuHealthy(bool healthy);
    void setStabilizationEnabled(bool enabled);
    bool isStabilizationEnabled() const;
    void updateYawSensorCache(bool healthy, float sensorDeg = 0.0f);
    bool readFreshYawSensorDeg(float& sensorDeg) const;
    bool controlSensorsHealthy() const;
    void publishYawTarget(float targetVoltage);
    bool readFocCommand(float& targetVoltage) const;
    bool isYawSensorFresh() const;

    // 返回相对车体正前方的炮塔 yaw 角，范围约为 -180~180 度。
    float getTurretRelativeYawDegFromSensor();

    bool validateImuEvent(float valueDegPerSec);

public:
    TankTurret(Adafruit_MPU6050& c, Adafruit_MPU6050& t);

    // 初始化舵机、双 IMU、AS5600、SimpleFOC yaw 电机。失败时返回 false。
    bool init();

    // 上电静态标定：估计 IMU 陀螺仪零偏，标定期间必须保持车体静止。
    void calibrate();

    // 读取并滤波 IMU 数据。dt 单位秒，通常由 TankRobot 以 500Hz 调用。
    void updateIMU(float dt);

    // 给底盘模块读取的底盘姿态量。
    float getLatestChassisPitchRate();
    float getLatestChassisYawRate();
    float getChassisPitchAngle();

    // 高频 FOC 入口，由 Core 0 任务循环调用。
    void runFOC();

    // 传感器故障或低压截止时调用：关闭稳定、清空目标、停止 yaw 输出。
    void enterSafeState();

    // 手柄断连时调用：停止执行目标，但不把传感器标记为故障。
    void enterDisconnectedState();

    // 处理 A 键开关和右摇杆手动瞄准输入。
    void handleUI(bool aPressed, float joyX, float joyY, float dt);

    // 稳定模式控制入口：根据保存的目标角和 IMU/编码器反馈更新舵机和 yaw 电压。
    void updateStabilization(float dt);

    bool isReady() const;
    bool isHealthy() const;
};
