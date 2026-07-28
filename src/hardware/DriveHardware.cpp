#include "DriveHardware.h"

DCMotor::DCMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, uint8_t ch, bool left)
    : in1(pin1), in2(pin2), pwmPin(pwmPin), pwmCh(ch), isLeft(left) {}

void DCMotor::init() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    ledcSetup(pwmCh, Config::PWM_FREQ, Config::PWM_RES);
    ledcAttachPin(pwmPin, pwmCh);
    // 初始化完成的第一条动作必须是明确关断，不能让驱动输入保持浮空或旧占空比。
    drive(0.0f);
}

void DCMotor::drive(float output) {
    // 物理输出的最后一道保护：NaN/Inf 不能被转成未定义的 PWM 整数。
    if (!isfinite(output)) output = 0.0f;
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

CustomEncoder::CustomEncoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t p_unit, int8_t directionSign)
    : pinA(pinA), pinB(pinB), unit(p_unit), directionSign(directionSign >= 0 ? 1 : -1) {}

static float medianOfThree(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

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
    lastPulseUs = lastSampleUs;
    lastDisplayUs = lastSampleUs;
    displayRawSpeedSamples[0] = 0.0f;
    displayRawSpeedSamples[1] = 0.0f;
    displayRawSpeedSamples[2] = 0.0f;
    displayRawSpeedSampleIndex = 0;
    displayMeasuredSpeed = 0.0f;
    lastSpeed = 0.0f;
    displaySpeed = 0.0f;
    controlSampleId = 0;
    controlSampleDt = 0.0f;
    stopTimeoutPublished = false;
}

float CustomEncoder::getRealSpeedKMH() {
    uint32_t now = micros();
    uint32_t dtUs = now - lastSampleUs;
    int16_t observedCount = 0;
    pcnt_get_counter_value(unit, &observedCount);

    bool enoughTimeAndCounts =
        dtUs >= Config::ENCODER_MIN_SAMPLE_US &&
        abs((int)observedCount) >= Config::ENCODER_MIN_COUNTS;
    bool maxWindowReached = dtUs >= Config::ENCODER_MAX_SAMPLE_US;

    if (enoughTimeAndCounts || maxWindowReached) {
        int16_t count = observedCount;

        // 只有完成一个自适应窗口后才清零；低速时不会丢掉零散脉冲。
        pcnt_counter_pause(unit);
        pcnt_get_counter_value(unit, &count);
        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);

        if (count != 0) {
            float sampleDt = dtUs * 0.000001f;
            float rpm = (count / (float)Config::ENCODER_COUNTS_PER_MOTOR_REV / Config::GEAR_RATIO) *
                        (60.0f / sampleDt);
            float measuredSpeed = rpm * Config::RPM_TO_REAL_KMH * directionSign;

            // 控制链路只拒绝明显超出机构能力的值，正常测量直接进入一阶低通。
            // 不在控制链路使用中值或加速度钳制，避免给速度 PID 和堵转判断增加延迟。
            if (abs(measuredSpeed) <= Config::ENCODER_MAX_VALID_SPEED_KMH) {
                lastPulseUs = now;
                stopTimeoutPublished = false;

                float alpha = sampleDt / (Config::ENCODER_CONTROL_FILTER_TAU_S + sampleDt);
                lastSpeed += alpha * (measuredSpeed - lastSpeed);
                controlSampleDt = sampleDt;
                ++controlSampleId;

                // 三点中值只服务于遥测显示，不参与任何底盘控制判断。
                displayRawSpeedSamples[displayRawSpeedSampleIndex] = measuredSpeed;
                displayRawSpeedSampleIndex = (displayRawSpeedSampleIndex + 1) % 3;
                displayMeasuredSpeed = medianOfThree(displayRawSpeedSamples[0],
                                                      displayRawSpeedSamples[1],
                                                      displayRawSpeedSamples[2]);
            }
        }
        lastSampleUs = now;
    }

    // 固定窗口的“零脉冲”不能立即证明停车；超过超时仍没有边沿才明确归零。
    if ((uint32_t)(now - lastPulseUs) >= Config::ENCODER_STOP_TIMEOUT_US &&
        !stopTimeoutPublished) {
        float timeoutDt = (now - lastPulseUs) * 0.000001f;
        lastSpeed = 0.0f;
        displayMeasuredSpeed = 0.0f;
        displayRawSpeedSamples[0] = 0.0f;
        displayRawSpeedSamples[1] = 0.0f;
        displayRawSpeedSamples[2] = 0.0f;
        displayRawSpeedSampleIndex = 0;
        controlSampleDt = timeoutDt;
        ++controlSampleId;
        stopTimeoutPublished = true;
    }

    // 遥测单独平滑，避免为了曲线观感给控制环增加额外延迟。
    uint32_t displayDtUs = now - lastDisplayUs;
    if (displayDtUs > 0) {
        float displayDt = displayDtUs * 0.000001f;
        float displayAlpha = displayDt / (Config::ENCODER_DISPLAY_FILTER_TAU_S + displayDt);
        displaySpeed += displayAlpha * (displayMeasuredSpeed - displaySpeed);
        if (lastSpeed == 0.0f && abs(displaySpeed) < Config::TRACK_STOP_DEADZONE_KMH) {
            displaySpeed = 0.0f;
        }
        lastDisplayUs = now;
    }

    return lastSpeed;
}

float CustomEncoder::getDisplaySpeedKMH() const {
    return displaySpeed;
}

uint32_t CustomEncoder::getControlSampleId() const {
    return controlSampleId;
}

float CustomEncoder::getControlSampleDt() const {
    return controlSampleDt;
}
