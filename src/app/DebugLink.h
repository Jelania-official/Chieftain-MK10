#pragma once

#include "config/RobotConfig.h"

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "app/ControlInput.h"

struct DebugTelemetry {
    uint32_t timeMs = 0;

    float leftTarget = 0.0f;
    float leftFastActual = 0.0f;
    float leftControlActual = 0.0f;
    float leftDisplayActual = 0.0f;
    float leftPhaseDeg = 0.0f;
    float leftPwm = 0.0f;
    bool leftStalled = false;

    float rightTarget = 0.0f;
    float rightFastActual = 0.0f;
    float rightControlActual = 0.0f;
    float rightDisplayActual = 0.0f;
    float rightPhaseDeg = 0.0f;
    float rightPwm = 0.0f;
    bool rightStalled = false;

    float pitchTarget = 0.0f;
    float pitchActual = 0.0f;
    float pitchServo = 0.0f;

    float yawTarget = 0.0f;
    float yawActual = 0.0f;
    float yawRelative = 0.0f;
    float yawVoltage = 0.0f;
    bool yawStalled = false;

    float chassisPitch = 0.0f;
    float chassisPitchRate = 0.0f;
    float chassisYawRate = 0.0f;

    bool stabilizationEnabled = false;
    bool imuHealthy = false;
    bool chassisImuHealthy = false;
    bool turretImuHealthy = false;
    bool yawSensorHealthy = false;
    bool chassisImuInitialized = false;
    bool turretImuInitialized = false;
    bool yawSensorInitialized = false;
    bool yawFocInitialized = false;
    bool chassisReady = false;
    bool turretReady = false;

    float batteryVoltage = 0.0f;
    bool batteryValid = false;
};

enum class DebugPidCommandType : uint8_t {
    None,
    Get,
    Set,
    RestoreDefaults,
};

struct DebugPidCommand {
    DebugPidCommandType type = DebugPidCommandType::None;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

class DebugLink {
private:
    BluetoothSerial serial;
    ControlInput latestInput;
    bool inputFresh = false;
    bool emergencyStop = false;
    bool directPwmActive = false;
    float directLeftPwm = 0.0f;
    float directRightPwm = 0.0f;
    uint32_t lastInputMs = 0;
    uint32_t lastTelemetryMs = 0;
    char rxLine[160] = {};
    size_t rxLen = 0;
    DebugPidCommand pendingPidCommand;

    void handleLine(char* line);

public:
    bool begin(const char* deviceName);
    void update();

    bool connected();
    bool hasFreshInput(uint32_t timeoutMs);
    bool readInput(ControlInput& out);
    bool readDirectPwm(float& leftPwm, float& rightPwm);
    bool stopRequested();

    // 读取一次 PC 发来的运行时 PID 调参命令；读取后清除挂起状态。
    bool readPidCommand(DebugPidCommand& out);
    void sendPidValues(float kp, float ki, float kd);
    void sendPidError(const char* reason);

    void sendTelemetry(const DebugTelemetry& telemetry, uint32_t intervalMs);
};

#endif
