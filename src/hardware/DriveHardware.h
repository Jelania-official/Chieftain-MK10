#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>
#include "config/DebugLog.h"
#include "config/RobotConfig.h"

// ==========================================
// 3. 底盘硬件抽象
// ==========================================

// TB6612 直流电机封装。
// drive() 的输入约定为 -255~255：绝对值是 PWM，占空比；正负号代表方向。
class DCMotor {
private:
    uint8_t in1, in2, pwmPin, pwmCh;
    bool isLeft;

public:
    // pin1/pin2 是方向脚，pwmPin 是 PWM 输出脚，ch 是 ESP32 LEDC 通道。
    // left=true 时方向逻辑反相，用来抵消左右履带安装方向不同。
    DCMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, uint8_t ch, bool left = false);

    // 配置 GPIO 和 LEDC PWM 通道。
    void init();

    // 根据 output 设置方向脚和 PWM；小于死区时完全断开电机输出。
    void drive(float output);
};

// N20 编码器封装。
// 使用 ESP32 PCNT 外设对 AB 相计数，再换算成真车等效速度 km/h。
class CustomEncoder {
private:
    uint8_t pinA, pinB;
    pcnt_unit_t unit;
    uint32_t lastSampleUs = 0;
    float lastSpeed = 0.0f;

public:
    // p_unit 指定使用哪个 PCNT 单元；左右履带需要不同单元。
    CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit);

    // 配置 PCNT 计数模式、滤波和初始采样时间。
    void init();

    // 按 Config::ENCODER_SAMPLE_US 周期刷新速度；周期未到时返回上一次滤波结果。
    float getRealSpeedKMH();
};
