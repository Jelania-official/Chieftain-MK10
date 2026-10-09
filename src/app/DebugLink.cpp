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

bool DebugLink::readDirectPwm(float& leftPwm, float& rightPwm) {
    if (!hasFreshInput(Config::DEBUG_INPUT_TIMEOUT_MS) || !directPwmActive) return false;
    leftPwm = directLeftPwm;
    rightPwm = directRightPwm;
    return true;
}

bool DebugLink::stopRequested() {
    return emergencyStop ||
           (connected() && inputFresh && ((uint32_t)(millis() - lastInputMs) > Config::DEBUG_INPUT_TIMEOUT_MS));
}

bool DebugLink::readPidCommand(DebugPidCommand& out) {
    if (pendingPidCommand.type == DebugPidCommandType::None) return false;
    out = pendingPidCommand;
    pendingPidCommand = DebugPidCommand{};
    return true;
}

void DebugLink::sendPidValues(float kp, float ki, float kd) {
    if (!connected()) return;
    serial.printf("PID,VALUE,%.4f,%.4f,%.4f\n", kp, ki, kd);
}

void DebugLink::sendPidError(const char* reason) {
    if (!connected()) return;
    serial.printf("PID,ERROR,%s\n", reason != nullptr ? reason : "unknown");
}

void DebugLink::handleLine(char* line) {
    if (strcmp(line, "HELLO,3") == 0) {
        serial.println("HELLO,ChieftainMK10,3");
        return;
    }

    if (strcmp(line, "STOP") == 0) {
        latestInput = ControlInput{};
        inputFresh = true;
        emergencyStop = true;
        directPwmActive = false;
        directLeftPwm = 0.0f;
        directRightPwm = 0.0f;
        lastInputMs = millis();
        return;
    }

    if (strcmp(line, "PID,GET") == 0) {
        pendingPidCommand = DebugPidCommand{};
        pendingPidCommand.type = DebugPidCommandType::Get;
        return;
    }

    if (strcmp(line, "PID,DEFAULT") == 0) {
        pendingPidCommand = DebugPidCommand{};
        pendingPidCommand.type = DebugPidCommandType::RestoreDefaults;
        return;
    }

    if (strncmp(line, "PID,SET,", 8) == 0) {
        char* token = strtok(line + 8, ",");
        if (token == nullptr) {
            sendPidError("format");
            return;
        }
        float kp = atof(token);
        token = strtok(nullptr, ",");
        if (token == nullptr) {
            sendPidError("format");
            return;
        }
        float ki = atof(token);
        token = strtok(nullptr, ",");
        if (token == nullptr) {
            sendPidError("format");
            return;
        }
        float kd = atof(token);

        // 手调范围只用于防止误输入，不改变正常控制输出的最终 ±255 安全限幅。
        if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
            kp < 0.0f || kp > 50.0f ||
            ki < 0.0f || ki > 50.0f ||
            kd < 0.0f || kd > 10.0f) {
            sendPidError("range");
            return;
        }
        pendingPidCommand = DebugPidCommand{};
        pendingPidCommand.type = DebugPidCommandType::Set;
        pendingPidCommand.kp = kp;
        pendingPidCommand.ki = ki;
        pendingPidCommand.kd = kd;
        return;
    }

    if (strncmp(line, "PWM,", 4) == 0) {
        // 固定 PWM 指令不能自行解除急停；必须先收到一次明确的零输入解锁。
        if (emergencyStop) return;
        char* token = strtok(line + 4, ",");
        if (token == nullptr) return;
        float leftPwm = atof(token);
        token = strtok(nullptr, ",");
        if (token == nullptr) return;
        float rightPwm = atof(token);

        latestInput = ControlInput{};
        directLeftPwm = constrain(leftPwm, -Config::DEBUG_DIRECT_PWM_MAX, Config::DEBUG_DIRECT_PWM_MAX);
        directRightPwm = constrain(rightPwm, -Config::DEBUG_DIRECT_PWM_MAX, Config::DEBUG_DIRECT_PWM_MAX);
        directPwmActive = true;
        inputFresh = true;
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
    directPwmActive = false;
    directLeftPwm = 0.0f;
    directRightPwm = 0.0f;
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
        directPwmActive = false;
        directLeftPwm = 0.0f;
        directRightPwm = 0.0f;
        pendingPidCommand = DebugPidCommand{};
        rxLen = 0;
    }
}

void DebugLink::sendTelemetry(const DebugTelemetry& t, uint32_t intervalMs) {
    if (!connected()) return;
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastTelemetryMs) < intervalMs) return;
    lastTelemetryMs = nowMs;

    // 机械诊断数据使用独立帧，避免改变既有 TEL v3 字段位置。
    // 旧版 PC 工具会忽略 SPD 帧，仍可正常连接并读取包括电池电压在内的遥测。
    serial.printf(
        "SPD,%lu,%.3f,%.3f,%.3f,%.3f\n",
        (unsigned long)t.timeMs,
        t.leftFastActual, t.leftPhaseDeg,
        t.rightFastActual, t.rightPhaseDeg
    );

    serial.printf(
        "TEL,%lu,%.3f,%.3f,%.3f,%.1f,%d,%.3f,%.3f,%.3f,%.1f,%d,"
        "%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%d,"
        "%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%d\n",
        (unsigned long)t.timeMs,
        t.leftTarget, t.leftControlActual, t.leftDisplayActual, t.leftPwm, t.leftStalled ? 1 : 0,
        t.rightTarget, t.rightControlActual, t.rightDisplayActual, t.rightPwm, t.rightStalled ? 1 : 0,
        t.pitchTarget, t.pitchActual, t.pitchServo,
        t.yawTarget, t.yawActual, t.yawRelative, t.yawVoltage, t.yawStalled ? 1 : 0,
        t.chassisPitch, t.chassisPitchRate, t.chassisYawRate,
        t.stabilizationEnabled ? 1 : 0,
        t.imuHealthy ? 1 : 0,
        t.chassisImuHealthy ? 1 : 0,
        t.turretImuHealthy ? 1 : 0,
        t.yawSensorHealthy ? 1 : 0,
        t.chassisImuInitialized ? 1 : 0,
        t.turretImuInitialized ? 1 : 0,
        t.yawSensorInitialized ? 1 : 0,
        t.yawFocInitialized ? 1 : 0,
        t.chassisReady ? 1 : 0,
        t.turretReady ? 1 : 0,
        t.batteryVoltage,
        t.batteryValid ? 1 : 0
    );
}

#endif
