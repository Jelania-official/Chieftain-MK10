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

void CustomEncoder::clearSpeedHistory() {
    speedHistoryHead = 0;
    speedHistoryCount = 0;
    historyDirection = 0;
}

void CustomEncoder::clearRollingWindow() {
    rollingHead = 0;
    rollingCount = 0;
    for (uint8_t i = 0; i < Config::ENCODER_ROLLING_BINS; ++i) {
        rollingCounts[i] = 0;
        rollingDtUs[i] = 0;
    }
}

void CustomEncoder::pushSpeedWindow(int16_t count, uint32_t dtUs) {
    if (count != 0) {
        int8_t direction = count > 0 ? 1 : -1;
        if (historyDirection != 0 && direction != historyDirection) {
            // 换向后的旧计数不能进入新方向的一圈平均。
            clearSpeedHistory();
        }
        historyDirection = direction;
    }

    speedHistory[speedHistoryHead].count = count;
    speedHistory[speedHistoryHead].dtUs = dtUs;
    speedHistoryHead = (speedHistoryHead + 1) % Config::ENCODER_REV_HISTORY_SIZE;
    if (speedHistoryCount < Config::ENCODER_REV_HISTORY_SIZE) ++speedHistoryCount;
}

bool CustomEncoder::getOneRevolutionAverage(float& count, float& dtSeconds) const {
    if (speedHistoryCount == 0 || historyDirection == 0) return false;

    int accumulatedCounts = 0;
    float accumulatedDtUs = 0.0f;
    const int requiredCounts = Config::ENCODER_COUNTS_PER_SPROCKET_REV;

    // 从最新窗口向前回看。跨过826个计数时只按比例取最老窗口的一部分，
    // 因而每次输出都对应“最近一圈”，而不是相互独立的一整圈。
    for (uint8_t i = 0; i < speedHistoryCount; ++i) {
        int index = (int)speedHistoryHead - 1 - i;
        if (index < 0) index += Config::ENCODER_REV_HISTORY_SIZE;
        const SpeedWindow& window = speedHistory[index];

        if (window.count != 0) {
            int8_t direction = window.count > 0 ? 1 : -1;
            if (direction != historyDirection) break;
        }

        int windowCounts = abs((int)window.count);
        int countsStillNeeded = requiredCounts - accumulatedCounts;
        if (windowCounts >= countsStillNeeded && windowCounts > 0) {
            float fraction = countsStillNeeded / (float)windowCounts;
            accumulatedCounts += countsStillNeeded;
            accumulatedDtUs += window.dtUs * fraction;
            count = historyDirection * (float)accumulatedCounts;
            dtSeconds = accumulatedDtUs * 0.000001f;
            return dtSeconds > 0.0f;
        }

        accumulatedCounts += windowCounts;
        accumulatedDtUs += window.dtUs;
    }

    return false;
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
    cumulativeOutputCount = 0;
    controlSampleId = 0;
    controlSampleDt = 0.0f;
    stopTimeoutPublished = false;
    clearRollingWindow();
    clearSpeedHistory();
}

float CustomEncoder::getRealSpeedKMH() {
    uint32_t now = micros();
    uint32_t dtUs = now - lastSampleUs;
    if (dtUs >= Config::ENCODER_UPDATE_US) {
        int16_t count = 0;

        // 每10ms结算一次硬件计数。短片段随后进入四个bin的重叠滚动窗口，
        // 不让PCNT长期累计，避免16位计数器达到上下限。
        pcnt_counter_pause(unit);
        pcnt_get_counter_value(unit, &count);
        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);

        // PCNT每个测速片段都会清零，因此另存本次启动内的累计计数。
        // 乘方向符号后，正相位统一对应车辆前进方向。
        cumulativeOutputCount += (int64_t)count * directionSign;

        pushSpeedWindow(count, dtUs);

        int previousRollingCount = 0;
        for (uint8_t i = 0; i < rollingCount; ++i) {
            previousRollingCount += rollingCounts[i];
        }
        if (count != 0 && previousRollingCount != 0 &&
            ((count > 0) != (previousRollingCount > 0))) {
            // 换向时不能让新旧方向的计数在40ms窗口里互相抵消。
            clearRollingWindow();
        }

        rollingCounts[rollingHead] = count;
        rollingDtUs[rollingHead] = dtUs;
        rollingHead = (rollingHead + 1) % Config::ENCODER_ROLLING_BINS;
        if (rollingCount < Config::ENCODER_ROLLING_BINS) ++rollingCount;

        int rollingTotalCount = 0;
        uint32_t rollingTotalDtUs = 0;
        for (uint8_t i = 0; i < rollingCount; ++i) {
            rollingTotalCount += rollingCounts[i];
            rollingTotalDtUs += rollingDtUs[i];
        }

        float updateDt = rollingTotalDtUs * 0.000001f;
        float rawMeasuredSpeed = 0.0f;
        bool rawMeasurementValid = false;
        if (rollingCount == Config::ENCODER_ROLLING_BINS &&
            rollingTotalCount != 0 && updateDt > 0.0f) {
            float rawRpm = (rollingTotalCount / (float)Config::ENCODER_COUNTS_PER_MOTOR_REV /
                            Config::GEAR_RATIO) *
                           (60.0f / updateDt);
            rawMeasuredSpeed = rawRpm * Config::RPM_TO_REAL_KMH * directionSign;
            rawMeasurementValid = abs(rawMeasuredSpeed) <= Config::ENCODER_MAX_VALID_SPEED_KMH;
        }

        // 编码器层直接输出40ms快速速度，不叠加原有60ms一阶低通；
        // 控制器另行生成中值/陷波反馈，安全判断始终保留这里的快速速度。
        if (count != 0) {
            // PCNT不给出单个边沿时间戳；用当前10ms结算时刻近似最后活动时间，
            // 误差上限约一个发布周期，不再被前一bin中的旧计数反复续期。
            lastPulseUs = now;
            stopTimeoutPublished = false;
        }
        if (rawMeasurementValid) {
            lastSpeed = rawMeasuredSpeed;
            controlSampleDt = dtUs * 0.000001f;
            ++controlSampleId;
        }

        // 最近一圈平均只供上位机平滑显示和机械诊断；历史尚不足一圈时临时显示
        // 当前快速测速，确保起步阶段仍有可见反馈。
        float averagedCount = 0.0f;
        float averagedDt = 0.0f;
        bool hasFullRevolution = getOneRevolutionAverage(averagedCount, averagedDt);
        float displayCandidate = rawMeasuredSpeed;
        bool displayCandidateValid = rawMeasurementValid;
        if (hasFullRevolution && averagedDt > 0.0f) {
            float averagedRpm = (averagedCount / (float)Config::ENCODER_COUNTS_PER_MOTOR_REV /
                                 Config::GEAR_RATIO) * (60.0f / averagedDt);
            float revolutionSpeed = averagedRpm * Config::RPM_TO_REAL_KMH * directionSign;
            if (abs(revolutionSpeed) <= Config::ENCODER_MAX_VALID_SPEED_KMH) {
                displayCandidate = revolutionSpeed;
                displayCandidateValid = true;
            }
        }

        if (displayCandidateValid) {
            displayRawSpeedSamples[displayRawSpeedSampleIndex] = displayCandidate;
            displayRawSpeedSampleIndex = (displayRawSpeedSampleIndex + 1) % 3;
            displayMeasuredSpeed = medianOfThree(displayRawSpeedSamples[0],
                                                  displayRawSpeedSamples[1],
                                                  displayRawSpeedSamples[2]);
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
        clearSpeedHistory();
        clearRollingWindow();
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

float CustomEncoder::getRelativeSprocketPhaseDeg() const {
    const int64_t countsPerRev = Config::ENCODER_COUNTS_PER_SPROCKET_REV;
    int64_t phaseCount = cumulativeOutputCount % countsPerRev;
    if (phaseCount < 0) phaseCount += countsPerRev;
    return phaseCount * (360.0f / (float)countsPerRev);
}
