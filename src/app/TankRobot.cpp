#include "TankRobot.h"

float TankRobot::readBatteryVoltage() {
    uint32_t adcMilliVolts = analogReadMilliVolts(Config::VBAT_ADC_PIN);
    float adcVolts = adcMilliVolts * 0.001f;
    return adcVolts * ((Config::VBAT_DIVIDER_R1 + Config::VBAT_DIVIDER_R2) / Config::VBAT_DIVIDER_R2);
}

void TankRobot::updateBatteryMonitor() {
    if (!Config::ENABLE_BATTERY_MONITOR) {
        batteryValid = false;
        batteryVoltage = 0.0f;
        return;
    }

    // 电池电压变化慢，不需要每个控制周期都读 ADC；按采样周期读取并低通即可。
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

bool TankRobot::batteryCritical() const {
    if (!Config::ENABLE_BATTERY_MONITOR) return false;
    return batteryValid && batteryVoltage <= Config::VBAT_CUTOFF;
}

bool TankRobot::batteryWarning() const {
    if (!Config::ENABLE_BATTERY_MONITOR) return false;
    return batteryValid && batteryVoltage <= Config::VBAT_WARN;
}

void TankRobot::updateControllerPacketClock() {
#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_XBOX
    unsigned long receivedAt = xboxController.getReceiveNotificationAt();
    if (receivedAt != 0) {
        lastControllerPacketMs = receivedAt;
    }
#endif
}

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_XBOX
bool TankRobot::xboxControllerHealthy() {
    // 有些情况下蓝牙仍显示 connected，但数据包已经停止刷新；这里同时检查包时间。
    return xboxController.isConnected() &&
           lastControllerPacketMs != 0 &&
           ((uint32_t)(millis() - lastControllerPacketMs) <= Config::CONTROLLER_TIMEOUT_MS);
}

bool TankRobot::readXboxControlInput(ControlInput& out) {
    if (!xboxControllerHealthy()) return false;
    out.triggerL = xboxController.xboxNotif.trigLT / 1023.0f;
    out.triggerR = xboxController.xboxNotif.trigRT / 1023.0f;
    out.joyLX = (xboxController.xboxNotif.joyLHori - 32767.5f) / 32767.5f;
    out.joyRX = (xboxController.xboxNotif.joyRHori - 32767.5f) / 32767.5f;
    out.joyRY = (xboxController.xboxNotif.joyRVert - 32767.5f) / 32767.5f;
    out.aPressed = xboxController.xboxNotif.btnA;
    return true;
}
#endif

bool TankRobot::readActiveControlInput(ControlInput& out) {
#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG
    return debugLink.readInput(out);
#elif ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_XBOX
    return readXboxControlInput(out);
#endif
}

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG
void TankRobot::publishDebugTelemetry() {
    DebugTelemetry t;
    t.timeMs = millis();
    t.chassisReady = chassisReady;
    t.turretReady = turretReady;
    t.batteryVoltage = batteryVoltage;
    t.batteryValid = batteryValid;

    if (chassisReady) {
        chassis.getTrackTelemetry(t.leftTarget, t.leftActual, t.leftPwm, t.leftStalled,
                                  t.rightTarget, t.rightActual, t.rightPwm, t.rightStalled);
    }

    if (turretReady) {
        t.pitchTarget = turret.getPitchTargetDeg();
        t.pitchActual = turret.getPitchActualDeg();
        t.pitchServo = turret.getPitchServoDeg();
        t.yawTarget = turret.getYawTargetDeg();
        t.yawActual = turret.getYawActualDeg();
        t.yawRelative = turret.getYawRelativeDeg();
        t.yawVoltage = turret.getYawVoltageTarget();
        t.chassisPitch = turret.getChassisPitchAngle();
        t.chassisPitchRate = turret.getLatestChassisPitchRate();
        t.chassisYawRate = turret.getLatestChassisYawRate();
        t.stabilizationEnabled = turret.stabilizationActive();
        t.imuHealthy = turret.imuIsHealthy();
        t.yawSensorHealthy = turret.yawSensorIsHealthy();
    }

    debugLink.sendTelemetry(t, Config::DEBUG_TELEMETRY_MS);
}
#endif

TankRobot::TankRobot()
#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_XBOX
    : xboxController(Config::XBOX_MAC), turret(mpuChassis, mpuTurret) {}
#else
    : turret(mpuChassis, mpuTurret) {}
#endif

void TankRobot::setup() {
    Serial.begin(921600);

    // Wire 给 FOC/AS5600 用，Wire1 给两颗 MPU6050 用；分总线可以减少 I2C 争用。
    Wire.begin(Config::I2C_FOC_SDA, Config::I2C_FOC_SCL);
    Wire.setClock(400000);
    Wire1.begin(Config::I2C_IMU_SDA, Config::I2C_IMU_SCL);
    Wire1.setClock(400000);

    if (Config::ENABLE_BATTERY_MONITOR) {
        analogReadResolution(12);
        analogSetPinAttenuation(Config::VBAT_ADC_PIN, ADC_11db);
        pinMode(Config::VBAT_ADC_PIN, INPUT);
        updateBatteryMonitor();
    } else {
        LOG_ALWAYS(">>> Battery monitor disabled; use external low-voltage alarm.\n");
    }

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG
    if (!debugLink.begin(Config::DEBUG_BT_NAME)) {
        LOG_ALWAYS("!!! Bluetooth debug link init failed.\n");
    }
#endif

    chassis.init();
    chassisReady = true;

    turretReady = turret.init();
    if (turretReady) {
        delay(200);
        turret.calibrate();
    } else {
        LOG_ALWAYS("!!! Turret unavailable; chassis control remains enabled.\n");
    }

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_XBOX
    xboxController.begin();
#endif
    lastControllerPacketMs = 0;

    uint32_t now = micros();
    lastIMU = now;
    lastUI = now;
    lastCtrl = now;
}

void TankRobot::runFOC_Only() {
    if (!turretReady) return;
    turret.runFOC();
}

void TankRobot::loop_without_FOC() {
    // Core 1 主循环使用 micros() 做分频调度，避免 delay 阻塞蓝牙和控制。
    uint32_t nowMicros = micros();
#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG
    debugLink.update();
#endif
    updateBatteryMonitor();

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG
    if (debugLink.stopRequested()) {
        if (chassisReady) chassis.stop();
        if (turretReady) turret.enterDisconnectedState();
        publishDebugTelemetry();
        return;
    }
#endif

    if (turretReady && nowMicros - lastIMU >= 2000) { // 500Hz IMU
        float dtIMU = (nowMicros - lastIMU) * 1e-6f;
        lastIMU = nowMicros;
        turret.updateIMU(dtIMU);
    }

    if (nowMicros - lastUI >= 20000) { // 50Hz UI
        float dtUI = (nowMicros - lastUI) * 1e-6f;
        lastUI = nowMicros;
#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_XBOX
        xboxController.onLoop();
        updateControllerPacketClock();
#endif

        ControlInput input;
        if (turretReady && readActiveControlInput(input)) {
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
            if (chassisReady && readActiveControlInput(input)) {
                // 底盘控制复用炮塔模块读取的底盘 IMU：坡度用于重力补偿，角速度用于虚拟惯量补偿。
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

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG
    publishDebugTelemetry();
#endif
}
