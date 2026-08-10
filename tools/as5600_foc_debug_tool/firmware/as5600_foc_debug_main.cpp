#include <Arduino.h>
#include <BluetoothSerial.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <stdarg.h>

#include "config/RobotConfig.h"

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t CONTROL_PERIOD_US = 5000;       // 与整车炮塔控制一致：200Hz
constexpr uint32_t SENSOR_CHECK_PERIOD_US = 10000; // 与整车额外 AS5600 检查一致：100Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;       // 与整车 PC 遥测一致：20Hz
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;
constexpr uint32_t BATTERY_SAMPLE_MS = 100;
constexpr uint32_t DIRECT_RUN_MAX_MS = 3000;
constexpr float DEFAULT_VOLTAGE_LIMIT = 1.0f;
constexpr float MIN_VOLTAGE_LIMIT = 0.5f;
constexpr float POSITION_LIMIT_DEG = 90.0f;
constexpr float VELOCITY_LIMIT_DPS = 25.0f;
// 闭环中必须能及时反向制动；过慢的斜率限制会额外引入约百毫秒延迟，
// 在低惯量裸转子上反而容易形成来回摆动。安全幅值仍由 voltage_limit 保证。
constexpr float VOLTAGE_SLEW_VPS = 100.0f;
constexpr float POWER_MIN_V = 6.0f;
constexpr float POWER_MAX_V = 16.5f;
constexpr uint32_t SENSOR_STALE_US = 50000;
constexpr size_t RX_LINE_CAPACITY = 160;
constexpr size_t TX_LINE_CAPACITY = 320;
constexpr float RAW_VELOCITY_FILTER_TAU_S = 0.02f;

// 独立工具的速度来自AS5600差分，不是整车使用的IMU陀螺仪。
// 因此从无D、低增益开始，避免12位角度量化噪声被微分项放大。
constexpr float DEBUG_OUTER_KP = 0.4f;
constexpr float DEBUG_OUTER_KD = 0.0f;
constexpr float DEBUG_INNER_KP = 0.01f;
constexpr float DEBUG_INNER_KI = 0.0f;
constexpr float DEBUG_INNER_KD = 0.0f;

enum class BenchState : uint8_t {
    SensorOnly = 0,
    Aligning = 1,
    Ready = 2,
    Running = 3,
    Fault = 4,
};

enum class BenchMode : uint8_t {
    Idle = 0,
    Voltage = 1,
    Velocity = 2,
    Position = 3,
};

enum class FaultCode : uint8_t {
    None = 0,
    EmergencyStop = 1,
    CommandTimeout = 2,
    SensorFailure = 3,
    SensorStale = 4,
    PowerInvalid = 5,
    Stall = 6,
    DirectRunTimeout = 7,
    FocAlignment = 8,
    InvalidNumber = 9,
};

struct RuntimePid {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integralLimit = 0.0f;
    float integral = 0.0f;
    float previousError = 0.0f;
    bool derivativeReady = false;

    void reset() {
        integral = 0.0f;
        previousError = 0.0f;
        derivativeReady = false;
    }

    float calculate(float target, float actual, float dt, float outputLimit) {
        if (!isfinite(dt) || dt <= 0.0f) dt = CONTROL_PERIOD_US * 1e-6f;
        float error = target - actual;
        integral += error * dt;
        integral = constrain(integral, -integralLimit, integralLimit);

        float derivative = 0.0f;
        if (derivativeReady) derivative = (error - previousError) / dt;
        previousError = error;
        derivativeReady = true;

        float output = (kp * error) + (ki * integral) + (kd * derivative);
        return constrain(output, -outputLimit, outputLimit);
    }
};

BLDCMotor motor(7);
BLDCDriver3PWM driver(Config::FOC_PWM_A, Config::FOC_PWM_B, Config::FOC_PWM_C);
MagneticSensorI2C sensor(AS5600_I2C);
BluetoothSerial bluetooth;

TaskHandle_t focTaskHandle = nullptr;
portMUX_TYPE sharedMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool sensorInitialized = false;
volatile bool alignmentInProgress = false;
volatile bool focReady = false;
volatile bool outputEnabled = false;
volatile float pendingVoltage = 0.0f;
volatile float cachedShaftAngleRad = 0.0f;
volatile float cachedShaftVelocityRadS = 0.0f;
volatile uint16_t cachedRawCount = 0;
volatile float cachedRawDeg = 0.0f;
volatile float cachedRawVelocityDps = 0.0f;
volatile bool cachedSensorHealthy = false;
volatile uint32_t cachedSensorUpdatedUs = 0;
volatile uint32_t focFrequencyHz = 0;

bool bluetoothReady = false;
bool motorHardwareInitialized = false;

BenchState state = BenchState::SensorOnly;
BenchMode mode = BenchMode::Idle;
FaultCode faultCode = FaultCode::None;

RuntimePid velocityPid;
float outerKp = DEBUG_OUTER_KP;
float outerKd = DEBUG_OUTER_KD;
float runtimeVoltageLimit = DEFAULT_VOLTAGE_LIMIT;

float controlZeroRad = 0.0f;
float targetPositionDeg = 0.0f;
float requestedVelocityDps = 0.0f;
float targetVelocityDps = 0.0f;
float requestedVoltage = 0.0f;
float appliedVoltage = 0.0f;
float batteryVoltage = 0.0f;
bool runRequested = false;
bool stallLatched = false;
uint32_t lastCommandMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryMs = 0;
uint32_t directRunStartedMs = 0;
uint32_t i2cErrorCount = 0;

bool rawVelocityReady = false;
float previousRawDeg = 0.0f;
uint32_t previousRawVelocityUs = 0;
float rawVelocityFilteredDps = 0.0f;
// AS5600 原始计数有固定的芯片正方向，而 SimpleFOC 会在 initFOC() 时
// 根据相序自动确定 sensor_direction。闭环控制必须统一使用 SimpleFOC 的
// 机械坐标，否则某些接线/安装方向下速度反馈会反号并变成正反馈。
float focSensorDirectionSign = 1.0f;

bool stallCandidateActive = false;
uint32_t stallCandidateSinceMs = 0;
float stallCandidateStartDeg = 0.0f;

struct ReceiveBuffer {
    char line[RX_LINE_CAPACITY] = {};
    size_t length = 0;
};

ReceiveBuffer usbReceive;
ReceiveBuffer bluetoothReceive;

void broadcastLine(const char* line) {
    Serial.println(line);
    if (bluetoothReady && bluetooth.hasClient()) bluetooth.println(line);
}

void broadcastFormatted(const char* format, ...) {
    char line[TX_LINE_CAPACITY] = {};
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);

    Serial.print(line);
    if (bluetoothReady && bluetooth.hasClient()) bluetooth.print(line);
}

const char* stateName(BenchState value) {
    switch (value) {
        case BenchState::SensorOnly: return "SENSOR_ONLY";
        case BenchState::Aligning: return "ALIGNING";
        case BenchState::Ready: return "READY";
        case BenchState::Running: return "RUNNING";
        case BenchState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

const char* modeName(BenchMode value) {
    switch (value) {
        case BenchMode::Idle: return "IDLE";
        case BenchMode::Voltage: return "VOLTAGE";
        case BenchMode::Velocity: return "VELOCITY";
        case BenchMode::Position: return "POSITION";
    }
    return "UNKNOWN";
}

const char* faultName(FaultCode value) {
    switch (value) {
        case FaultCode::None: return "none";
        case FaultCode::EmergencyStop: return "emergency_stop";
        case FaultCode::CommandTimeout: return "command_timeout";
        case FaultCode::SensorFailure: return "sensor_failure";
        case FaultCode::SensorStale: return "sensor_stale";
        case FaultCode::PowerInvalid: return "power_invalid";
        case FaultCode::Stall: return "stall";
        case FaultCode::DirectRunTimeout: return "direct_run_timeout";
        case FaultCode::FocAlignment: return "foc_alignment";
        case FaultCode::InvalidNumber: return "invalid_number";
    }
    return "unknown";
}

void resetControllers() {
    velocityPid.reset();
    targetVelocityDps = 0.0f;
    appliedVoltage = 0.0f;
    stallCandidateActive = false;
    stallCandidateSinceMs = 0;
    stallCandidateStartDeg = 0.0f;
}

void publishVoltage(float voltage, bool enabled) {
    portENTER_CRITICAL(&sharedMux);
    pendingVoltage = constrain(voltage, -runtimeVoltageLimit, runtimeVoltageLimit);
    outputEnabled = enabled;
    portEXIT_CRITICAL(&sharedMux);
}

void stopOutput() {
    runRequested = false;
    requestedVoltage = 0.0f;
    requestedVelocityDps = 0.0f;
    targetVelocityDps = 0.0f;
    directRunStartedMs = 0;
    resetControllers();
    publishVoltage(0.0f, false);
}

void enterFault(FaultCode code) {
    stopOutput();
    mode = BenchMode::Idle;
    faultCode = code;
    state = BenchState::Fault;
    stallLatched = (code == FaultCode::Stall);
    broadcastFormatted("FAULT,%u,%s\n", (unsigned)code, faultName(code));
}

bool readAS5600Raw(uint16_t& raw, float& angleDeg) {
    Wire.beginTransmission(Config::AS5600_ADDR);
    Wire.write(Config::AS5600_ANGLE_REG);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t received = Wire.requestFrom(Config::AS5600_ADDR, (uint8_t)2);
    if (received != 2 || Wire.available() < 2) {
        while (Wire.available() > 0) Wire.read();
        return false;
    }

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    raw = ((uint16_t)(msb & 0x0F) << 8) | lsb;
    angleDeg = raw * (360.0f / 4096.0f);
    return isfinite(angleDeg);
}

void updateSensorCacheFromCore0(uint32_t nowUs) {
    uint16_t raw = 0;
    float rawDeg = 0.0f;
    bool healthy = readAS5600Raw(raw, rawDeg);

    if (healthy) {
        if (rawVelocityReady) {
            float dt = (nowUs - previousRawVelocityUs) * 1e-6f;
            if (dt > 0.0f && dt <= 0.1f) {
                float deltaDeg = rawDeg - previousRawDeg;
                if (deltaDeg > 180.0f) deltaDeg -= 360.0f;
                if (deltaDeg < -180.0f) deltaDeg += 360.0f;
                float instantaneousDps = deltaDeg / dt;
                float alpha = dt / (RAW_VELOCITY_FILTER_TAU_S + dt);
                rawVelocityFilteredDps += alpha * (instantaneousDps - rawVelocityFilteredDps);
            } else {
                rawVelocityFilteredDps = 0.0f;
            }
        } else {
            rawVelocityFilteredDps = 0.0f;
            rawVelocityReady = true;
        }
        previousRawDeg = rawDeg;
        previousRawVelocityUs = nowUs;
    } else {
        rawVelocityReady = false;
        rawVelocityFilteredDps = 0.0f;
    }

    portENTER_CRITICAL(&sharedMux);
    cachedSensorHealthy = healthy;
    if (healthy) {
        cachedRawCount = raw;
        cachedRawDeg = rawDeg;
        cachedRawVelocityDps = rawVelocityFilteredDps;
        cachedSensorUpdatedUs = nowUs;
    } else {
        cachedRawVelocityDps = 0.0f;
    }
    portEXIT_CRITICAL(&sharedMux);

    if (!healthy) {
        portENTER_CRITICAL(&sharedMux);
        ++i2cErrorCount;
        portEXIT_CRITICAL(&sharedMux);
    }
}

struct SensorSnapshot {
    float shaftAngleRad = 0.0f;
    float shaftVelocityRadS = 0.0f;
    uint16_t rawCount = 0;
    float rawDeg = 0.0f;
    float rawVelocityDps = 0.0f;
    bool healthy = false;
    uint32_t updatedUs = 0;
    uint32_t focHz = 0;
    uint32_t i2cErrors = 0;
};

SensorSnapshot readSnapshot() {
    SensorSnapshot snapshot;
    portENTER_CRITICAL(&sharedMux);
    snapshot.shaftAngleRad = cachedShaftAngleRad;
    snapshot.shaftVelocityRadS = cachedShaftVelocityRadS;
    snapshot.rawCount = cachedRawCount;
    snapshot.rawDeg = cachedRawDeg;
    snapshot.rawVelocityDps = cachedRawVelocityDps;
    snapshot.healthy = cachedSensorHealthy;
    snapshot.updatedUs = cachedSensorUpdatedUs;
    snapshot.focHz = focFrequencyHz;
    snapshot.i2cErrors = i2cErrorCount;
    portEXIT_CRITICAL(&sharedMux);
    return snapshot;
}

float readBatteryVoltage() {
    uint32_t millivolts = analogReadMilliVolts(Config::VBAT_ADC_PIN);
    float adcVoltage = millivolts * 0.001f;
    return adcVoltage * ((Config::VBAT_DIVIDER_R1 + Config::VBAT_DIVIDER_R2) /
                         Config::VBAT_DIVIDER_R2);
}

bool powerIsValid() {
    return isfinite(batteryVoltage) &&
           batteryVoltage >= POWER_MIN_V &&
           batteryVoltage <= POWER_MAX_V;
}

void focTask(void*) {
    uint32_t lastSensorCheckUs = 0;
    uint32_t frequencyWindowStartUs = micros();
    uint32_t loopCount = 0;

    for (;;) {
        uint32_t nowUs = micros();
        bool initialized;
        bool aligning;
        bool aligned;
        float voltage;
        bool enabled;

        portENTER_CRITICAL(&sharedMux);
        initialized = sensorInitialized;
        aligning = alignmentInProgress;
        aligned = focReady;
        voltage = pendingVoltage;
        enabled = outputEnabled;
        portEXIT_CRITICAL(&sharedMux);

        if (initialized && !aligning) {
            if (aligned) {
                motor.loopFOC();
                motor.target = enabled ? voltage : 0.0f;
                motor.move(enabled ? voltage : 0.0f);

                portENTER_CRITICAL(&sharedMux);
                cachedShaftAngleRad = motor.shaft_angle;
                cachedShaftVelocityRadS = motor.shaft_velocity;
                portEXIT_CRITICAL(&sharedMux);
                ++loopCount;
            }

            if ((uint32_t)(nowUs - lastSensorCheckUs) >= SENSOR_CHECK_PERIOD_US) {
                lastSensorCheckUs = nowUs;
                updateSensorCacheFromCore0(nowUs);
            }
        }

        if ((uint32_t)(nowUs - frequencyWindowStartUs) >= 1000000) {
            uint32_t elapsedUs = nowUs - frequencyWindowStartUs;
            uint32_t measuredHz = elapsedUs > 0
                ? (uint32_t)((uint64_t)loopCount * 1000000ULL / elapsedUs)
                : 0;
            portENTER_CRITICAL(&sharedMux);
            focFrequencyHz = measuredHz;
            portEXIT_CRITICAL(&sharedMux);
            loopCount = 0;
            frequencyWindowStartUs = nowUs;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool parseFloatStrict(const char* text, float& value) {
    if (text == nullptr || *text == '\0') return false;
    char* end = nullptr;
    value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(value);
}

void sendPidValues() {
    broadcastFormatted("PID,VALUE,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                       outerKp, outerKd,
                       velocityPid.kp, velocityPid.ki, velocityPid.kd);
}

void setDefaultPidValues() {
    outerKp = DEBUG_OUTER_KP;
    outerKd = DEBUG_OUTER_KD;
    velocityPid.kp = DEBUG_INNER_KP;
    velocityPid.ki = DEBUG_INNER_KI;
    velocityPid.kd = DEBUG_INNER_KD;
    velocityPid.integralLimit = 5.0f;
    resetControllers();
}

bool initializeMotorHardware() {
    if (motorHardwareInitialized) return true;

    // 普通上电只读取AS5600，不切换LEDC/PWM。只有操作者明确执行FOC对齐时，
    // 才初始化驱动和电机，避免固件启动阶段产生额外的PWM状态切换。
    driver.voltage_power_supply = 12.0f;
    driver.voltage_limit = runtimeVoltageLimit;
    if (!driver.init()) return false;

    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = runtimeVoltageLimit;
    motor.PID_velocity.P = 0.0f;
    motor.PID_velocity.I = 0.0f;
    motor.PID_velocity.D = 0.0f;
    motor.LPF_velocity.Tf = RAW_VELOCITY_FILTER_TAU_S;
    motor.init();

    motorHardwareInitialized = true;
    return true;
}

void performFocAlignment() {
    if (state != BenchState::SensorOnly || focReady) {
        broadcastLine("ERR,ALIGN,state");
        return;
    }
    SensorSnapshot snapshot = readSnapshot();
    if (!snapshot.healthy ||
        (uint32_t)(micros() - snapshot.updatedUs) > SENSOR_STALE_US) {
        broadcastLine("ERR,ALIGN,sensor");
        return;
    }
    if (!powerIsValid()) {
        broadcastFormatted("ERR,ALIGN,power,%.3f\n", batteryVoltage);
        return;
    }

    state = BenchState::Aligning;
    portENTER_CRITICAL(&sharedMux);
    alignmentInProgress = true;
    outputEnabled = false;
    pendingVoltage = 0.0f;
    portEXIT_CRITICAL(&sharedMux);
    delay(20);

    if (!initializeMotorHardware()) {
        portENTER_CRITICAL(&sharedMux);
        alignmentInProgress = false;
        portEXIT_CRITICAL(&sharedMux);
        enterFault(FaultCode::FocAlignment);
        broadcastLine("ACK,ALIGN,0,driver_init");
        return;
    }

    motor.voltage_limit = runtimeVoltageLimit;
    driver.voltage_limit = runtimeVoltageLimit;
    int result = motor.initFOC();

    if (result == 1) {
        motor.target = 0.0f;
        motor.move(0.0f);
        focSensorDirectionSign = (float)motor.sensor_direction;
        if (focSensorDirectionSign != 1.0f && focSensorDirectionSign != -1.0f) {
            focSensorDirectionSign = 1.0f;
        }
        float shaft = motor.shaftAngle();
        controlZeroRad = shaft;
        portENTER_CRITICAL(&sharedMux);
        cachedShaftAngleRad = shaft;
        cachedShaftVelocityRadS = 0.0f;
        focReady = true;
        alignmentInProgress = false;
        portEXIT_CRITICAL(&sharedMux);
        faultCode = FaultCode::None;
        state = BenchState::Ready;
        mode = BenchMode::Idle;
        broadcastFormatted("ACK,ALIGN,1,%d,%.6f\n",
                           motor.sensor_direction,
                           motor.zero_electric_angle);
    } else {
        portENTER_CRITICAL(&sharedMux);
        focReady = false;
        alignmentInProgress = false;
        portEXIT_CRITICAL(&sharedMux);
        enterFault(FaultCode::FocAlignment);
        broadcastFormatted("ACK,ALIGN,0,%d,0\n", result);
    }
}

void setMode(BenchMode nextMode) {
    if (!focReady || state == BenchState::SensorOnly || state == BenchState::Aligning) {
        broadcastLine("ERR,MODE,not_aligned");
        return;
    }
    if (state == BenchState::Fault) {
        broadcastLine("ERR,MODE,fault");
        return;
    }
    stopOutput();
    mode = nextMode;
    state = BenchState::Ready;
    targetPositionDeg = 0.0f;
    broadcastFormatted("ACK,MODE,%s\n", modeName(mode));
}

void clearFault() {
    if (state != BenchState::Fault) {
        broadcastLine("ACK,CLEAR,no_fault");
        return;
    }
    SensorSnapshot snapshot = readSnapshot();
    bool sensorOk = snapshot.healthy &&
                    (uint32_t)(micros() - snapshot.updatedUs) <= SENSOR_STALE_US;
    if (!sensorOk || (focReady && !powerIsValid())) {
        broadcastLine("ERR,CLEAR,health");
        return;
    }
    stopOutput();
    stallLatched = false;
    faultCode = FaultCode::None;
    mode = BenchMode::Idle;
    state = focReady ? BenchState::Ready : BenchState::SensorOnly;
    broadcastLine("ACK,CLEAR,ok");
}

void handlePidSet(char* save) {
    float values[5] = {};
    for (float& value : values) {
        char* token = strtok_r(nullptr, ",", &save);
        if (!parseFloatStrict(token, value)) {
            broadcastLine("ERR,PID,format");
            return;
        }
    }

    if (values[0] < 0.0f || values[0] > 20.0f ||
        values[1] < 0.0f || values[1] > 5.0f ||
        values[2] < 0.0f || values[2] > 5.0f ||
        values[3] < 0.0f || values[3] > 5.0f ||
        values[4] < 0.0f || values[4] > 1.0f) {
        broadcastLine("ERR,PID,range");
        return;
    }

    outerKp = values[0];
    outerKd = values[1];
    velocityPid.kp = values[2];
    velocityPid.ki = values[3];
    velocityPid.kd = values[4];
    resetControllers();
    sendPidValues();
}

void handleLine(char* line) {
    char* save = nullptr;
    char* command = strtok_r(line, ",", &save);
    if (command == nullptr) return;

    if (strcmp(command, "HELLO") == 0) {
        char* version = strtok_r(nullptr, ",", &save);
        if (version != nullptr && strcmp(version, "1") == 0) {
            broadcastLine("HELLO,Chieftain-AS5600-FOC,1");
        } else {
            broadcastLine("ERR,HELLO,version");
        }
        return;
    }

    if (strcmp(command, "HB") == 0) {
        lastCommandMs = millis();
        return;
    }

    if (strcmp(command, "ALIGN") == 0) {
        lastCommandMs = millis();
        performFocAlignment();
        return;
    }

    if (strcmp(command, "STOP") == 0) {
        lastCommandMs = millis();
        enterFault(FaultCode::EmergencyStop);
        return;
    }

    if (strcmp(command, "CLEAR") == 0) {
        lastCommandMs = millis();
        clearFault();
        return;
    }

    if (strcmp(command, "ZERO") == 0) {
        lastCommandMs = millis();
        if (!focReady || state == BenchState::Running || state == BenchState::Fault) {
            broadcastLine("ERR,ZERO,state");
            return;
        }
        SensorSnapshot snapshot = readSnapshot();
        controlZeroRad = snapshot.shaftAngleRad;
        targetPositionDeg = 0.0f;
        resetControllers();
        broadcastLine("ACK,ZERO,ok");
        return;
    }

    if (strcmp(command, "MODE") == 0) {
        lastCommandMs = millis();
        char* value = strtok_r(nullptr, ",", &save);
        if (value == nullptr) {
            broadcastLine("ERR,MODE,format");
        } else if (strcmp(value, "IDLE") == 0) {
            setMode(BenchMode::Idle);
        } else if (strcmp(value, "VOLTAGE") == 0) {
            setMode(BenchMode::Voltage);
        } else if (strcmp(value, "VELOCITY") == 0) {
            setMode(BenchMode::Velocity);
        } else if (strcmp(value, "POSITION") == 0) {
            setMode(BenchMode::Position);
        } else {
            broadcastLine("ERR,MODE,value");
        }
        return;
    }

    if (strcmp(command, "SET") == 0) {
        lastCommandMs = millis();
        float value = 0.0f;
        if (!parseFloatStrict(strtok_r(nullptr, ",", &save), value)) {
            broadcastLine("ERR,SET,format");
            return;
        }
        if (mode == BenchMode::Voltage) {
            requestedVoltage = constrain(value, -runtimeVoltageLimit, runtimeVoltageLimit);
        } else if (mode == BenchMode::Velocity) {
            requestedVelocityDps = constrain(value, -VELOCITY_LIMIT_DPS, VELOCITY_LIMIT_DPS);
        } else if (mode == BenchMode::Position) {
            targetPositionDeg = constrain(value, -POSITION_LIMIT_DEG, POSITION_LIMIT_DEG);
        } else {
            broadcastLine("ERR,SET,mode");
            return;
        }
        return;
    }

    if (strcmp(command, "RUN") == 0) {
        lastCommandMs = millis();
        char* value = strtok_r(nullptr, ",", &save);
        bool requested = value != nullptr && strcmp(value, "1") == 0;
        if (!requested) {
            stopOutput();
            if (state != BenchState::Fault) state = focReady ? BenchState::Ready : BenchState::SensorOnly;
            return;
        }
        if (!focReady || state == BenchState::Fault || mode == BenchMode::Idle || !powerIsValid()) {
            broadcastLine("ERR,RUN,state");
            stopOutput();
            return;
        }
        runRequested = true;
        state = BenchState::Running;
        if (mode == BenchMode::Voltage && directRunStartedMs == 0) {
            directRunStartedMs = millis();
        }
        return;
    }

    if (strcmp(command, "LIMIT") == 0) {
        lastCommandMs = millis();
        float value = 0.0f;
        if (!parseFloatStrict(strtok_r(nullptr, ",", &save), value)) {
            broadcastLine("ERR,LIMIT,format");
            return;
        }
        if (state == BenchState::Running || state == BenchState::Aligning) {
            broadcastLine("ERR,LIMIT,running");
            return;
        }
        runtimeVoltageLimit = constrain(value, MIN_VOLTAGE_LIMIT, Config::YAW_VOLTAGE_MAX);
        motor.voltage_limit = runtimeVoltageLimit;
        driver.voltage_limit = runtimeVoltageLimit;
        broadcastFormatted("ACK,LIMIT,%.3f\n", runtimeVoltageLimit);
        return;
    }

    if (strcmp(command, "PID") == 0) {
        lastCommandMs = millis();
        char* action = strtok_r(nullptr, ",", &save);
        if (action == nullptr) {
            broadcastLine("ERR,PID,format");
        } else if (strcmp(action, "GET") == 0) {
            sendPidValues();
        } else if (strcmp(action, "DEFAULT") == 0) {
            setDefaultPidValues();
            sendPidValues();
        } else if (strcmp(action, "SET") == 0) {
            handlePidSet(save);
        } else {
            broadcastLine("ERR,PID,action");
        }
        return;
    }

    broadcastLine("ERR,COMMAND,unknown");
}

void pollStream(Stream& stream, ReceiveBuffer& receive) {
    while (stream.available() > 0) {
        char c = (char)stream.read();
        if (c == '\n' || c == '\r') {
            if (receive.length > 0) {
                receive.line[receive.length] = '\0';
                handleLine(receive.line);
                receive.length = 0;
            }
        } else if (receive.length + 1 < RX_LINE_CAPACITY) {
            receive.line[receive.length++] = c;
        } else {
            receive.length = 0;
            broadcastLine("ERR,COMMAND,too_long");
        }
    }
}

void pollTransports() {
    pollStream(Serial, usbReceive);
    if (bluetoothReady && bluetooth.hasClient()) {
        pollStream(bluetooth, bluetoothReceive);
    } else {
        bluetoothReceive.length = 0;
    }
}

void updateStallProtection(float positionDeg, float voltage, float positionErrorDeg) {
    bool highEffort = abs(voltage) >= Config::YAW_STALL_VOLTAGE_MIN;
    bool demandExists = mode != BenchMode::Position ||
                        abs(positionErrorDeg) >= Config::YAW_STALL_ERROR_MIN_DEG;
    if (!runRequested || !highEffort || !demandExists) {
        stallCandidateActive = false;
        return;
    }

    uint32_t nowMs = millis();
    if (!stallCandidateActive) {
        stallCandidateActive = true;
        stallCandidateSinceMs = nowMs;
        stallCandidateStartDeg = positionDeg;
        return;
    }

    if (abs(positionDeg - stallCandidateStartDeg) > Config::YAW_STALL_MAX_TRAVEL_DEG) {
        stallCandidateSinceMs = nowMs;
        stallCandidateStartDeg = positionDeg;
        return;
    }

    if ((uint32_t)(nowMs - stallCandidateSinceMs) >= Config::YAW_STALL_CONFIRM_MS) {
        enterFault(FaultCode::Stall);
    }
}

void updateControl() {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;
    float dt = (nowUs - lastControlUs) * 1e-6f;
    lastControlUs = nowUs;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.05f) dt = CONTROL_PERIOD_US * 1e-6f;

    SensorSnapshot snapshot = readSnapshot();
    bool stale = snapshot.updatedUs == 0 ||
                 (uint32_t)(nowUs - snapshot.updatedUs) > SENSOR_STALE_US;

    if (state == BenchState::Running) {
        if (!snapshot.healthy) {
            enterFault(FaultCode::SensorFailure);
            return;
        }
        if (stale) {
            enterFault(FaultCode::SensorStale);
            return;
        }
        if (!powerIsValid()) {
            enterFault(FaultCode::PowerInvalid);
            return;
        }
        if ((uint32_t)(millis() - lastCommandMs) > COMMAND_TIMEOUT_MS) {
            enterFault(FaultCode::CommandTimeout);
            return;
        }
        if (mode == BenchMode::Voltage && directRunStartedMs != 0 &&
            (uint32_t)(millis() - directRunStartedMs) > DIRECT_RUN_MAX_MS) {
            enterFault(FaultCode::DirectRunTimeout);
            return;
        }
    }

    if (state != BenchState::Running || !runRequested || !focReady) {
        appliedVoltage = 0.0f;
        publishVoltage(0.0f, false);
        return;
    }

    float positionDeg = (snapshot.shaftAngleRad - controlZeroRad) * RAD_TO_DEG;
    // motor.shaftAngle() 已由 SimpleFOC 按 sensor_direction 修正；这里的
    // AS5600 原始差分速度也必须做同样修正，才能形成真正的负反馈。
    float actualVelocityDps = snapshot.rawVelocityDps * focSensorDirectionSign;
    float positionErrorDeg = targetPositionDeg - positionDeg;
    float desiredVoltage = 0.0f;

    if (mode == BenchMode::Voltage) {
        targetVelocityDps = 0.0f;
        desiredVoltage = requestedVoltage;
    } else if (mode == BenchMode::Velocity) {
        targetVelocityDps = requestedVelocityDps;
        desiredVoltage = velocityPid.calculate(
            targetVelocityDps, actualVelocityDps, dt, runtimeVoltageLimit);
    } else if (mode == BenchMode::Position) {
        targetVelocityDps = (outerKp * positionErrorDeg) -
                            (outerKd * actualVelocityDps);
        targetVelocityDps = constrain(
            targetVelocityDps, -Config::YAW_OUTER_RATE_MAX, Config::YAW_OUTER_RATE_MAX);
        desiredVoltage = velocityPid.calculate(
            targetVelocityDps, actualVelocityDps, dt, runtimeVoltageLimit);
    }

    desiredVoltage = constrain(desiredVoltage, -runtimeVoltageLimit, runtimeVoltageLimit);
    float maxDelta = VOLTAGE_SLEW_VPS * dt;
    appliedVoltage += constrain(desiredVoltage - appliedVoltage, -maxDelta, maxDelta);
    appliedVoltage = constrain(appliedVoltage, -runtimeVoltageLimit, runtimeVoltageLimit);
    publishVoltage(appliedVoltage, true);
    updateStallProtection(positionDeg, appliedVoltage, positionErrorDeg);
}

void sendTelemetry() {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastTelemetryMs) < TELEMETRY_PERIOD_MS) return;
    lastTelemetryMs = nowMs;

    SensorSnapshot snapshot = readSnapshot();
    float positionDeg = focReady
        ? (snapshot.shaftAngleRad - controlZeroRad) * RAD_TO_DEG
        : 0.0f;
    float velocityDps = focReady
        ? snapshot.rawVelocityDps * focSensorDirectionSign
        : 0.0f;
    float positionErrorDeg = targetPositionDeg - positionDeg;

    broadcastFormatted(
        "TEL,%lu,%u,%u,%u,%u,%.3f,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%lu,%lu\n",
        (unsigned long)nowMs,
        (unsigned)state,
        (unsigned)mode,
        focReady ? 1U : 0U,
        snapshot.healthy ? 1U : 0U,
        batteryVoltage,
        (unsigned)snapshot.rawCount,
        snapshot.rawDeg,
        positionDeg,
        targetPositionDeg,
        targetVelocityDps,
        velocityDps,
        appliedVoltage,
        runtimeVoltageLimit,
        positionErrorDeg,
        stallLatched ? 1U : 0U,
        (unsigned long)snapshot.i2cErrors,
        (unsigned long)snapshot.focHz);
}

} // namespace

void setup() {
    pinMode(Config::FOC_PWM_A, OUTPUT);
    pinMode(Config::FOC_PWM_B, OUTPUT);
    pinMode(Config::FOC_PWM_C, OUTPUT);
    digitalWrite(Config::FOC_PWM_A, LOW);
    digitalWrite(Config::FOC_PWM_B, LOW);
    digitalWrite(Config::FOC_PWM_C, LOW);

    Serial.begin(SERIAL_BAUD);
    bluetoothReady = bluetooth.begin(Config::DEBUG_BT_NAME);
    Wire.begin(Config::I2C_FOC_SDA, Config::I2C_FOC_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(3);

    analogReadResolution(12);
    analogSetPinAttenuation(Config::VBAT_ADC_PIN, ADC_11db);
    batteryVoltage = readBatteryVoltage();

    sensor.init();
    sensorInitialized = true;

    setDefaultPidValues();
    lastCommandMs = millis();
    lastControlUs = micros();

    xTaskCreatePinnedToCore(
        focTask,
        "FOC_Debug_Task",
        8192,
        nullptr,
        5,
        &focTaskHandle,
        0);

    broadcastFormatted("BOOT,Chieftain-AS5600-FOC,1,BT=%u,%s\n",
                       bluetoothReady ? 1U : 0U,
                       Config::DEBUG_BT_NAME);
}

void loop() {
    pollTransports();

    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastBatteryMs) >= BATTERY_SAMPLE_MS) {
        lastBatteryMs = nowMs;
        float measured = readBatteryVoltage();
        if (isfinite(measured)) batteryVoltage += 0.2f * (measured - batteryVoltage);
    }

    updateControl();
    sendTelemetry();
}
