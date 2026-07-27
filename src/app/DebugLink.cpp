#include "DebugLink.h"

#if ROBOT_INPUT_MODE == ROBOT_INPUT_MODE_PC_DEBUG

bool DebugLink::begin(const char* deviceName) {
    return serial.begin(deviceName);
}

bool DebugLink::connected() {
    return serial.hasClient();
}

bool DebugLink::hasFreshInput(uint32_t timeoutMs) {
    return connected() &&
           inputFresh &&
           ((uint32_t)(millis() - lastInputMs) <= timeoutMs) &&
           !emergencyStop;
}

bool DebugLink::readInput(ControlInput& out) {
    if (!hasFreshInput(Config::DEBUG_INPUT_TIMEOUT_MS)) return false;
    out = latestInput;
    return true;
}

bool DebugLink::stopRequested() {
    return emergencyStop ||
           (connected() && inputFresh && ((uint32_t)(millis() - lastInputMs) > Config::DEBUG_INPUT_TIMEOUT_MS));
}

void DebugLink::handleLine(char* line) {
    if (strcmp(line, "STOP") == 0) {
        latestInput = ControlInput{};
        inputFresh = true;
        emergencyStop = true;
        lastInputMs = millis();
        return;
    }

    if (strncmp(line, "IN,", 3) != 0) return;

    char* token = strtok(line + 3, ",");
    // PC 端协议依次发送 5 个模拟量，再发送 A 键和急停标志：
    // IN,triggerL,triggerR,joyLX,joyRX,joyRY,aPressed,stop
    float values[5] = {};
    for (int i = 0; i < 5; ++i) {
        if (token == nullptr) return;
        values[i] = atof(token);
        token = strtok(nullptr, ",");
    }

    int aPressed = 0;
    int stop = 0;
    if (token != nullptr) {
        aPressed = atoi(token);
        token = strtok(nullptr, ",");
    }
    if (token != nullptr) {
        stop = atoi(token);
    }

    latestInput.triggerL = constrain(values[0], 0.0f, 1.0f);
    latestInput.triggerR = constrain(values[1], 0.0f, 1.0f);
    latestInput.joyLX = constrain(values[2], -1.0f, 1.0f);
    latestInput.joyRX = constrain(values[3], -1.0f, 1.0f);
    latestInput.joyRY = constrain(values[4], -1.0f, 1.0f);
    latestInput.aPressed = (aPressed != 0);
    inputFresh = true;
    emergencyStop = (stop != 0);
    lastInputMs = millis();
}

void DebugLink::update() {
    while (serial.available() > 0) {
        char c = (char)serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            rxLine[rxLen] = '\0';
            if (rxLen > 0) handleLine(rxLine);
            rxLen = 0;
            continue;
        }
        if (rxLen < sizeof(rxLine) - 1) {
            rxLine[rxLen++] = c;
        } else {
            rxLen = 0;
        }
    }

    if (!connected()) {
        inputFresh = false;
        emergencyStop = false;
        rxLen = 0;
    }
}

void DebugLink::sendTelemetry(const DebugTelemetry& t, uint32_t intervalMs) {
    if (!connected()) return;
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastTelemetryMs) < intervalMs) return;
    lastTelemetryMs = nowMs;

    serial.printf(
        "TEL,%lu,%.3f,%.3f,%.1f,%d,%.3f,%.3f,%.1f,%d,"
        "%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%.3f,%d\n",
        (unsigned long)t.timeMs,
        t.leftTarget, t.leftActual, t.leftPwm, t.leftStalled ? 1 : 0,
        t.rightTarget, t.rightActual, t.rightPwm, t.rightStalled ? 1 : 0,
        t.pitchTarget, t.pitchActual, t.pitchServo,
        t.yawTarget, t.yawActual, t.yawRelative, t.yawVoltage,
        t.chassisPitch, t.chassisPitchRate, t.chassisYawRate,
        t.stabilizationEnabled ? 1 : 0,
        t.imuHealthy ? 1 : 0,
        t.yawSensorHealthy ? 1 : 0,
        t.chassisReady ? 1 : 0,
        t.turretReady ? 1 : 0,
        t.batteryVoltage,
        t.batteryValid ? 1 : 0
    );
}

#endif
