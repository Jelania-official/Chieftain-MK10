#pragma once

#include <Arduino.h>
#include <Wire.h>

// 项目内的 MPU6050 兼容初始化层。
// 经典 MPU6050 的 WHO_AM_I 为 0x68；商家“新版 MPU6050”返回 0x70，
// 但声明量程、寄存器和数据格式与 MPU6050 一致。
namespace Mpu6x00Compat {

constexpr uint8_t WHO_AM_I_REG = 0x75;
constexpr uint8_t CLASSIC_ID = 0x68;
constexpr uint8_t COMPATIBLE_ID = 0x70;

constexpr uint8_t SAMPLE_RATE_DIV_REG = 0x19;
constexpr uint8_t CONFIG_REG = 0x1A;
constexpr uint8_t GYRO_CONFIG_REG = 0x1B;
constexpr uint8_t ACCEL_CONFIG_REG = 0x1C;
constexpr uint8_t FIFO_ENABLE_REG = 0x23;
constexpr uint8_t INT_ENABLE_REG = 0x38;
constexpr uint8_t USER_CONTROL_REG = 0x6A;
constexpr uint8_t POWER_MGMT_1_REG = 0x6B;
constexpr uint8_t POWER_MGMT_2_REG = 0x6C;

constexpr uint8_t SAMPLE_RATE_DIV_200_HZ = 4; // DLPF 开启时：1kHz / (1 + 4)
constexpr uint8_t DLPF_44_HZ = 3;
constexpr uint8_t GYRO_RANGE_500_DPS = 1u << 3;
constexpr uint8_t ACCEL_RANGE_4_G = 1u << 3;

enum class InitStage : uint8_t {
    None,
    ProbeIdentity,
    Reset,
    Wake,
    DisableAuxiliaryFeatures,
    SampleRate,
    Filter,
    GyroRange,
    AccelRange,
    PostInitIdentity
};

struct InitResult {
    bool ok = false;
    uint8_t whoAmI = 0;
    InitStage failedStage = InitStage::None;
};

inline const char* stageName(InitStage stage) {
    switch (stage) {
        case InitStage::None: return "none";
        case InitStage::ProbeIdentity: return "probe_identity";
        case InitStage::Reset: return "reset";
        case InitStage::Wake: return "wake";
        case InitStage::DisableAuxiliaryFeatures: return "disable_auxiliary_features";
        case InitStage::SampleRate: return "sample_rate";
        case InitStage::Filter: return "filter";
        case InitStage::GyroRange: return "gyro_range";
        case InitStage::AccelRange: return "accel_range";
        case InitStage::PostInitIdentity: return "post_init_identity";
    }
    return "unknown";
}

inline bool isSupportedId(uint8_t id) {
    return id == CLASSIC_ID || id == COMPATIBLE_ID;
}

inline bool writeRegister(TwoWire& wire, uint8_t address, uint8_t reg, uint8_t value) {
    wire.beginTransmission(address);
    wire.write(reg);
    wire.write(value);
    return wire.endTransmission(true) == 0;
}

inline bool readRegisters(TwoWire& wire, uint8_t address, uint8_t startReg,
                          uint8_t* data, size_t length, bool repeatedStart = false) {
    if (data == nullptr || length == 0 || length > 255) return false;

    wire.beginTransmission(address);
    wire.write(startReg);
    if (wire.endTransmission(!repeatedStart) != 0) return false;

    size_t received = wire.requestFrom(address, (uint8_t)length, (uint8_t)true);
    if (received != length || wire.available() < (int)length) {
        while (wire.available() > 0) wire.read();
        return false;
    }

    for (size_t i = 0; i < length; ++i) data[i] = (uint8_t)wire.read();
    return true;
}

inline bool readRegister(TwoWire& wire, uint8_t address, uint8_t reg,
                         uint8_t& value, bool repeatedStart = false) {
    return readRegisters(wire, address, reg, &value, 1, repeatedStart);
}

inline bool probeIdentity(TwoWire& wire, uint8_t address, uint8_t& detectedId,
                          uint8_t maxAttempts = 12, uint8_t requiredConsecutive = 3) {
    uint8_t lastId = 0;
    uint8_t consecutive = 0;

    for (uint8_t attempt = 0; attempt < maxAttempts; ++attempt) {
        uint8_t id = 0;
        if (readRegister(wire, address, WHO_AM_I_REG, id) && isSupportedId(id)) {
            if (id == lastId) {
                ++consecutive;
            } else {
                lastId = id;
                consecutive = 1;
            }
            if (consecutive >= requiredConsecutive) {
                detectedId = id;
                return true;
            }
        } else {
            consecutive = 0;
        }
        delay(2);
    }
    return false;
}

inline bool writeVerified(TwoWire& wire, uint8_t address, uint8_t reg,
                          uint8_t value, uint8_t mask = 0xFF, uint8_t attempts = 3) {
    for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
        uint8_t readback = 0;
        if (writeRegister(wire, address, reg, value)) {
            delay(2);
            if (readRegister(wire, address, reg, readback) &&
                (readback & mask) == (value & mask)) {
                return true;
            }
        }
        delay(2);
    }
    return false;
}

inline InitResult initialize(TwoWire& wire, uint8_t address) {
    InitResult result;

    if (!probeIdentity(wire, address, result.whoAmI)) {
        result.failedStage = InitStage::ProbeIdentity;
        return result;
    }

    if (!writeRegister(wire, address, POWER_MGMT_1_REG, 0x80)) {
        result.failedStage = InitStage::Reset;
        return result;
    }
    delay(100);

    // 唤醒并使用 X 轴陀螺仪 PLL；同时确认休眠、循环和温度禁用位均已清除。
    if (!writeVerified(wire, address, POWER_MGMT_1_REG, 0x01, 0x67) ||
        !writeVerified(wire, address, POWER_MGMT_2_REG, 0x00)) {
        result.failedStage = InitStage::Wake;
        return result;
    }

    if (!writeVerified(wire, address, USER_CONTROL_REG, 0x00) ||
        !writeVerified(wire, address, FIFO_ENABLE_REG, 0x00) ||
        !writeVerified(wire, address, INT_ENABLE_REG, 0x00)) {
        result.failedStage = InitStage::DisableAuxiliaryFeatures;
        return result;
    }

    if (!writeVerified(wire, address, SAMPLE_RATE_DIV_REG, SAMPLE_RATE_DIV_200_HZ)) {
        result.failedStage = InitStage::SampleRate;
        return result;
    }
    if (!writeVerified(wire, address, CONFIG_REG, DLPF_44_HZ, 0x07)) {
        result.failedStage = InitStage::Filter;
        return result;
    }
    if (!writeVerified(wire, address, GYRO_CONFIG_REG, GYRO_RANGE_500_DPS, 0x18)) {
        result.failedStage = InitStage::GyroRange;
        return result;
    }
    if (!writeVerified(wire, address, ACCEL_CONFIG_REG, ACCEL_RANGE_4_G, 0x18)) {
        result.failedStage = InitStage::AccelRange;
        return result;
    }

    uint8_t postInitId = 0;
    if (!probeIdentity(wire, address, postInitId, 8, 2) || postInitId != result.whoAmI) {
        result.failedStage = InitStage::PostInitIdentity;
        return result;
    }

    result.ok = true;
    result.failedStage = InitStage::None;
    return result;
}

} // namespace Mpu6x00Compat
