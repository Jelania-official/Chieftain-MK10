#pragma once

#include "config/RobotConfig.h"

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "app/ControlInput.h"

struct DebugTelemetry {
    uint32_t timeMs = 0;

    float leftTarget = 0.0f;
    float leftActual = 0.0f;
    float leftPwm = 0.0f;
    bool leftStalled = false;

    float rightTarget = 0.0f;
    float rightActual = 0.0f;
    float rightPwm = 0.0f;
    bool rightStalled = false;

    float pitchTarget = 0.0f;
    float pitchActual = 0.0f;
    float pitchServo = 0.0f;

    float yawTarget = 0.0f;
    float yawActual = 0.0f;
    float yawRelative = 0.0f;
    float yawVoltage = 0.0f;

    float chassisPitch = 0.0f;
    float chassisPitchRate = 0.0f;
    float chassisYawRate = 0.0f;

    bool stabilizationEnabled = false;
    bool imuHealthy = false;
    bool yawSensorHealthy = false;
    bool chassisReady = false;
    bool turretReady = false;

    float batteryVoltage = 0.0f;
    bool batteryValid = false;
};

class DebugLink {
private:
    BluetoothSerial serial;
    ControlInput latestInput;
    bool inputFresh = false;
    bool emergencyStop = false;
    uint32_t lastInputMs = 0;
    uint32_t lastTelemetryMs = 0;
    char rxLine[160] = {};
    size_t rxLen = 0;

    void handleLine(char* line);

public:
    bool begin(const char* deviceName);
    void update();

    bool connected();
    bool hasFreshInput(uint32_t timeoutMs);
    bool readInput(ControlInput& out);
    bool stopRequested();

    void sendTelemetry(const DebugTelemetry& telemetry, uint32_t intervalMs);
};

#endif
