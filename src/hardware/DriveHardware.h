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
    int8_t directionSign;
    uint32_t lastSampleUs = 0;
    uint32_t lastPulseUs = 0;
    uint32_t lastDisplayUs = 0;
    float displayRawSpeedSamples[3] = {};
    uint8_t displayRawSpeedSampleIndex = 0;
    float displayMeasuredSpeed = 0.0f;
    float lastSpeed = 0.0f;
    float displaySpeed = 0.0f;
    uint32_t controlSampleId = 0;
    float controlSampleDt = 0.0f;
    bool stopTimeoutPublished = false;

public:
    // p_unit 指定使用哪个 PCNT 单元；左右履带需要不同单元。
    CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit, int8_t directionSign = 1);

    // 配置 PCNT 计数模式、滤波和初始采样时间。
    void init();

    // 使用自适应脉冲累计窗口刷新控制用速度；窗口未完成时返回上一次结果。
    float getRealSpeedKMH();

    // 返回额外平滑的遥测速度，不参与 PI 和堵转判断。
    float getDisplaySpeedKMH() const;

    // 每产生一个新的有效控制测速（含首次超时归零）就递增，用于同步 PID 的 D 项。
    uint32_t getControlSampleId() const;
    float getControlSampleDt() const;
};
