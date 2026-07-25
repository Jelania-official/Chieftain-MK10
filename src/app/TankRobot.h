#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_MPU6050.h>
#include "../config/DebugLog.h"
#include "../config/RobotConfig.h"
#include "../chassis/TankChassis.h"
#include "../turret/TankTurret.h"
#include "ControlInput.h"

// ==========================================
// 6. 顶层统筹与任务调度
// ==========================================
class TankRobot {
private:
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
    Adafruit_MPU6050 mpuChassis, mpuTurret;
    TankChassis chassis; TankTurret turret;
    uint32_t lastIMU = 0, lastUI = 0, lastCtrl = 0;
    uint32_t lastControllerPacketMs = 0;
    uint32_t lastBatterySampleMs = 0;
    uint32_t lastBatteryCutoffLogMs = 0;
    bool chassisReady = false;
    bool turretReady = false;
    float batteryVoltage = 0.0f;
    bool batteryValid = false;

    float readBatteryVoltage() {
        uint32_t adcMilliVolts = analogReadMilliVolts(Config::VBAT_ADC_PIN);
        float adcVolts = adcMilliVolts * 0.001f;
        return adcVolts * ((Config::VBAT_DIVIDER_R1 + Config::VBAT_DIVIDER_R2) / Config::VBAT_DIVIDER_R2);
    }

    void updateBatteryMonitor() {
        if (!Config::ENABLE_BATTERY_MONITOR) {
            batteryValid = false;
            batteryVoltage = 0.0f;
            return;
        }

        uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastBatterySampleMs) < Config::VBAT_SAMPLE_MS) return;
        lastBatterySampleMs = nowMs;

        float sample = readBatteryVoltage();
        if (!isfinite(sample) || sample <= 0.0f) {
            batteryValid = false;
            return;
        }

        if (!batteryValid) {
            batteryVoltage = sample;
            batteryValid = true;
        } else {
            batteryVoltage += Config::VBAT_LPF * (sample - batteryVoltage);
        }
    }

    bool batteryCritical() const {
        if (!Config::ENABLE_BATTERY_MONITOR) return false;
        return batteryValid && batteryVoltage <= Config::VBAT_CUTOFF;
    }

    bool batteryWarning() const {
        if (!Config::ENABLE_BATTERY_MONITOR) return false;
        return batteryValid && batteryVoltage <= Config::VBAT_WARN;
    }

    void updateControllerPacketClock() {
        unsigned long receivedAt = xboxController.getReceiveNotificationAt();
        if (receivedAt != 0) {
            lastControllerPacketMs = receivedAt;
        }
    }

    // 手柄库的 isConnected() 不是 const 成员，所以这里不能把方法声明成 const。
    bool xboxControllerHealthy() {
        return xboxController.isConnected() &&
               lastControllerPacketMs != 0 &&
               ((uint32_t)(millis() - lastControllerPacketMs) <= Config::CONTROLLER_TIMEOUT_MS);
    }

    bool readControlInput(ControlInput& out) {
        if (!xboxControllerHealthy()) return false;
        out.triggerL = xboxController.xboxNotif.trigLT / 1023.0f;
        out.triggerR = xboxController.xboxNotif.trigRT / 1023.0f;
        out.joyLX = (xboxController.xboxNotif.joyLHori - 32767.5f) / 32767.5f;
        out.joyRX = (xboxController.xboxNotif.joyRHori - 32767.5f) / 32767.5f;
        out.joyRY = (xboxController.xboxNotif.joyRVert - 32767.5f) / 32767.5f;
        out.aPressed = xboxController.xboxNotif.btnA;
        return true;
    }

public:
    TankRobot() : xboxController(Config::XBOX_MAC), turret(mpuChassis, mpuTurret) {}


    void setup() {
        Serial.begin(921600);
        Wire.begin(Config::I2C_FOC_SDA, Config::I2C_FOC_SCL); Wire.setClock(400000); 
        Wire1.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL); Wire1.setClock(400000); 
        if (Config::ENABLE_BATTERY_MONITOR) {
            analogReadResolution(12);
            analogSetPinAttenuation(Config::VBAT_ADC_PIN, ADC_11db);
            pinMode(Config::VBAT_ADC_PIN, INPUT);
            updateBatteryMonitor();
        } else {
            LOG_ALWAYS(">>> Battery monitor disabled; use external low-voltage alarm.\n");
        }
        chassis.init();
        chassisReady = true;
        turretReady = turret.init();
        if (turretReady) {
            delay(200);
            turret.calibrate();
        } else {
            LOG_ALWAYS("!!! Turret unavailable; chassis control remains enabled.\n");
        }
        xboxController.begin();
        lastControllerPacketMs = 0;

        uint32_t now = micros();
        lastIMU = now; lastUI = now; lastCtrl = now;
    }

    
    void runFOC_Only() {
        if (!turretReady) return;
        turret.runFOC(); // 内部调用 yawMotor.loopFOC() 和 move()
    }

    // Core 1 主循环：低频 UI、中频控制、高频 IMU，和 Core 0 的 FOC 任务解耦。
    void loop_without_FOC() {
        // 蓝牙、IMU、底盘动力学都在 Core 1 执行
        uint32_t nowMicros = micros();
        updateBatteryMonitor();

        if (turretReady && nowMicros - lastIMU >= 2000) { // 500Hz IMU
            float dtIMU = (nowMicros - lastIMU) * 1e-6f; 
            lastIMU = nowMicros;
            turret.updateIMU(dtIMU);
        }
        if (nowMicros - lastUI >= 20000) { // 50Hz UI
            float dtUI = (nowMicros - lastUI) * 1e-6f;
            lastUI = nowMicros;
            xboxController.onLoop();
            updateControllerPacketClock();

            ControlInput input;
            if (turretReady && readControlInput(input)) {
                turret.handleUI(input.aPressed, input.joyRX, input.joyRY, dtUI);
            }
        }
        if (nowMicros - lastCtrl >= 5000) { // 200Hz 控制
            float dtCtrl = (nowMicros - lastCtrl) * 1e-6f;
            lastCtrl = nowMicros;
            if (batteryCritical()) {
                if (chassisReady) chassis.stop();
                if (turretReady) turret.enterSafeState();
                if (millis() - lastBatteryCutoffLogMs >= 1000) {
                    lastBatteryCutoffLogMs = millis();
                    LOG_ALWAYS("!!! Battery cutoff active: %.2fV\n", batteryVoltage);
                }
            } else {
                ControlInput input;
                if (chassisReady && readControlInput(input)) {
                    // 坡度角用于底盘动力学中的重力分量，pitch/yaw rate 用于生成虚拟旋转惯量。
                    float pRate = turretReady ? turret.getLatestChassisPitchRate() : 0.0f;
                    float yRate = turretReady ? turret.getLatestChassisYawRate() : 0.0f;
                    float pAngle = turretReady ? turret.getChassisPitchAngle() : 0.0f;
                    chassis.processKinematics(input.triggerL, input.triggerR, input.joyLX, dtCtrl, pRate, yRate, pAngle);
                    if (turretReady) turret.updateStabilization(dtCtrl);

                    static uint32_t lastBatteryWarnLogMs = 0;
                    if (batteryWarning() && (millis() - lastBatteryWarnLogMs >= 1000)) {
                        lastBatteryWarnLogMs = millis();
                        LOG_ALWAYS("*** Battery low warning: %.2fV\n", batteryVoltage);
                    }
                } else {
                    if (chassisReady) chassis.stop();
                    if (turretReady) turret.enterDisconnectedState();
                }
            }
        }
    }
};


