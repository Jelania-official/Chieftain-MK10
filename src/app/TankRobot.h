#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_MPU6050.h>
#include "config/DebugLog.h"
#include "config/RobotConfig.h"
#include "chassis/TankChassis.h"
#include "turret/TankTurret.h"
#include "ControlInput.h"

// ==========================================
// 6. 应用调度层
// ==========================================
// app 文件夹放“把各模块组装起来运行”的代码。
// TankRobot 不实现底盘/炮塔算法本身，而是负责：
// 1. 初始化 I2C、串口、手柄、电池检测、底盘和炮塔；
// 2. 按不同频率调度 IMU、UI、底盘控制和炮塔稳定；
// 3. 处理断连、电池低压、炮塔不可用等整车级状态。
class TankRobot {
private:
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
    Adafruit_MPU6050 mpuChassis, mpuTurret;
    TankChassis chassis;
    TankTurret turret;

    // 各任务上一次运行时间，用 micros() 做非阻塞定时调度。
    uint32_t lastIMU = 0, lastUI = 0, lastCtrl = 0;

    // 手柄最后一次收到数据包的时间，用于判断“连接但不再更新”的假连接。
    uint32_t lastControllerPacketMs = 0;

    // 电池采样和告警节流状态。
    uint32_t lastBatterySampleMs = 0;
    uint32_t lastBatteryCutoffLogMs = 0;
    bool chassisReady = false;
    bool turretReady = false;
    float batteryVoltage = 0.0f;
    bool batteryValid = false;

    // 读取 ADC 并按分压电阻换算为电池实际电压。
    float readBatteryVoltage();

    // 按 Config::VBAT_SAMPLE_MS 采样，并做一阶低通滤波。
    void updateBatteryMonitor();

    // 达到硬截止电压时返回 true；上层会立即停止底盘并让炮塔进入安全状态。
    bool batteryCritical() const;

    // 达到低电压预警阈值时返回 true；只打印警告，不切断动力。
    bool batteryWarning() const;

    // 从手柄库取最近数据包时间，更新 lastControllerPacketMs。
    void updateControllerPacketClock();

    // 手柄库的 isConnected() 不是 const 成员，所以这里不能声明成 const。
    // 除了蓝牙连接状态，还要求最近收到过有效数据包。
    bool xboxControllerHealthy();

    // 将 Xbox 原始按键/摇杆数据归一化到 ControlInput。
    bool readControlInput(ControlInput& out);

public:
    TankRobot();

    // Arduino setup() 中调用一次：初始化整车硬件和状态。
    void setup();

    // 只运行炮塔 yaw 电机的 FOC 高频循环；由 Core 0 任务调用。
    void runFOC_Only();

    // 运行除 FOC 以外的主循环；由 Arduino loop() 在 Core 1 调用。
    // 内部分别以 500Hz/50Hz/200Hz 调度 IMU、手柄 UI、底盘和稳定控制。
    void loop_without_FOC();
};
