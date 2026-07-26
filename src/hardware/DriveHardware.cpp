#include "DriveHardware.h"

DCMotor::DCMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, uint8_t ch, bool left)
    : in1(pin1), in2(pin2), pwmPin(pwmPin), pwmCh(ch), isLeft(left) {}

void DCMotor::init() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    ledcSetup(pwmCh, Config::PWM_FREQ, Config::PWM_RES);
    ledcAttachPin(pwmPin, pwmCh);
}

void DCMotor::drive(float output) {
    float absOut = constrain(abs(output), 0.0f, 255.0f);
    if (absOut < Config::MOTOR_PWM_DEADZONE) {
        // 完全停止时不仅 PWM 归零，也把方向脚拉低，避免电机桥臂持续发热。
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(pwmCh, 0);
        return;
    }

    // 左右履带电机安装方向不同，isLeft 用来把“正 PWM”统一成车辆前进方向。
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

CustomEncoder::CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit)
    : pinA(pinA), pinB(pinB), unit(p_unit) {}

void CustomEncoder::init() {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);

    // PCNT 直接在硬件里对 AB 相计数，比在中断里手动计数更稳。
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = pinA;
    cfg.ctrl_gpio_num = pinB;
    cfg.channel = PCNT_CHANNEL_0;
    cfg.unit = unit;
    cfg.pos_mode = PCNT_COUNT_INC;
    cfg.neg_mode = PCNT_COUNT_DEC;
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_REVERSE;
    cfg.counter_h_lim = 32767;
    cfg.counter_l_lim = -32768;

    esp_err_t err = pcnt_unit_config(&cfg);
    if (err != ESP_OK) {
        LOG_ALWAYS("!!! PCNT config failed: unit=%d err=%d\n", (int)unit, (int)err);
    }

    pcnt_set_filter_value(unit, 1000);
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
    lastSampleUs = micros();
}

float CustomEncoder::getRealSpeedKMH() {
    uint32_t now = micros();
    uint32_t dtUs = now - lastSampleUs;
    if (dtUs >= Config::ENCODER_SAMPLE_US) {
        int16_t count = 0;

        // 读取后立即清零，得到这一个采样窗口内的增量脉冲数。
        pcnt_counter_pause(unit);
        pcnt_get_counter_value(unit, &count);
        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);

        float rpm = (count / (float)Config::ENCODER_PPR / Config::GEAR_RATIO) * (60000000.0f / dtUs);
        float measuredSpeed = rpm * Config::RPM_TO_REAL_KMH;
        lastSpeed += Config::ENCODER_SPEED_LPF * (measuredSpeed - lastSpeed);
        if (count == 0 && abs(lastSpeed) < Config::TRACK_STOP_DEADZONE_KMH) lastSpeed = 0.0f;
        lastSampleUs = now;
    }
    return lastSpeed;
}
