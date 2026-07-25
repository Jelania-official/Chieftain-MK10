#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>
#include "../config/DebugLog.h"
#include "../config/RobotConfig.h"

// ==========================================
// 3. 硬件抽象 (电机与编码器)
// ==========================================
// TB6612 有刷电机抽象：输入范围约定为 -255~255，符号代表方向。
class DCMotor {
private:
    uint8_t in1, in2, pwmPin, pwmCh; bool isLeft;
public:
    DCMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, uint8_t ch, bool left = false)
        : in1(pin1), in2(pin2), pwmPin(pwmPin), pwmCh(ch), isLeft(left) {}
    void init() {
        pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
        ledcSetup(pwmCh, Config::PWM_FREQ, Config::PWM_RES);
        ledcAttachPin(pwmPin, pwmCh);
    }
    void drive(float output) {
        // 1. 限制范围并处理死区
        float absOut = constrain(abs(output), 0.0f, 255.0f);
        if (absOut < Config::MOTOR_PWM_DEADZONE) {
            // 完全停止：不仅 PWM 给 0，电机引脚也要拉低，防止发热
            digitalWrite(in1, LOW); 
            digitalWrite(in2, LOW); 
            ledcWrite(pwmCh, 0);
            return;
        }

        // 2. 物理驱动方向逻辑；静摩擦补偿由履带前馈控制器负责。
        if (output > 0) {
            digitalWrite(in1, isLeft ? HIGH : LOW); 
            digitalWrite(in2, isLeft ? LOW : HIGH);
            ledcWrite(pwmCh, (uint32_t)absOut);
        } else {
            digitalWrite(in1, isLeft ? LOW : HIGH); 
            digitalWrite(in2, isLeft ? HIGH : LOW);
            ledcWrite(pwmCh, (uint32_t)absOut);
        }
    }
};

// N20 编码器抽象：用 ESP32 的 PCNT 外设做 AB 相计数，再换算成真车等效速度。
class CustomEncoder {
private:
    uint8_t pinA, pinB; pcnt_unit_t unit; uint32_t lastSampleUs = 0; float lastSpeed = 0.0f;
public:
    CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit) : pinA(pinA), pinB(pinB), unit(p_unit) {}
    void init() {
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);
        pcnt_config_t cfg = {};
        cfg.pulse_gpio_num = pinA; cfg.ctrl_gpio_num = pinB;
        cfg.channel = PCNT_CHANNEL_0; cfg.unit = unit;
        cfg.pos_mode = PCNT_COUNT_INC; cfg.neg_mode = PCNT_COUNT_DEC;
        cfg.lctrl_mode = PCNT_MODE_KEEP; cfg.hctrl_mode = PCNT_MODE_REVERSE;
        cfg.counter_h_lim = 32767; cfg.counter_l_lim = -32768;
        esp_err_t err = pcnt_unit_config(&cfg);
        if (err != ESP_OK) {
            LOG_ALWAYS("!!! PCNT config failed: unit=%d err=%d\n", (int)unit, (int)err);
        }
        pcnt_set_filter_value(unit, 1000);
        pcnt_filter_enable(unit);
        pcnt_counter_pause(unit); pcnt_counter_clear(unit); pcnt_counter_resume(unit);
        lastSampleUs = micros();
    }
    float getRealSpeedKMH() {
        uint32_t now = micros();
        uint32_t dtUs = now - lastSampleUs;
        if (dtUs >= Config::ENCODER_SAMPLE_US) {
            int16_t count = 0;
            pcnt_counter_pause(unit);
            pcnt_get_counter_value(unit, &count);
            pcnt_counter_clear(unit);
            pcnt_counter_resume(unit);

            float rpm = (count / (float)Config::ENCODER_PPR / Config::GEAR_RATIO) * (60000000.0f / dtUs);
            float measuredSpeed = rpm * Config::RPM_TO_REAL_KMH; // 输出真车等效速度
            lastSpeed += Config::ENCODER_SPEED_LPF * (measuredSpeed - lastSpeed);
            if (count == 0 && abs(lastSpeed) < Config::TRACK_STOP_DEADZONE_KMH) lastSpeed = 0.0f;
            lastSampleUs = now;
        }
        return lastSpeed;
    }
};


