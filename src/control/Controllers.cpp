#include "Controllers.h"

CustomPID::CustomPID(float p, float i, float d, float mi, float mo)
    : kp(p), ki(i), kd(d), maxI(mi), maxOut(mo) {}

float CustomPID::calculate(float target, float actual, float dt) {
    if (dt <= 0.0f) dt = 0.001f;

    float error = target - actual;
    integral += error * dt;
    integral = constrain(integral, -maxI, maxI);

    float derivative = (error - prevError) / dt;
    prevError = error;

    return constrain(kp * error + ki * integral + kd * derivative, -maxOut, maxOut);
}

void CustomPID::reset() {
    integral = 0;
    prevError = 0;
}

CascadePID::CascadePID(CustomPID out, CustomPID in, float ff) : outer(out), inner(in), ff_gain(ff) {}

float CascadePID::calculate(float posRef, float posFdb, float posRateRef,
                            float velFdb, float chassisVel, float dt) {
    // yaw 参数放在 Config 里，允许调参时只改配置，不用追到构造函数。
    outer.kp = Config::YAW_OUTER_KP;
    outer.kd = Config::YAW_OUTER_KD;
    outer.maxOut = Config::YAW_OUTER_RATE_MAX;
    inner.kp = Config::YAW_INNER_KP;
    inner.ki = Config::YAW_INNER_KI;
    inner.kd = Config::YAW_INNER_KD;
    inner.maxOut = Config::YAW_VOLTAGE_MAX;
    ff_gain = Config::YAW_CHASSIS_FF_GAIN;

    // 两自由度外环：
    // 1. 手柄角速度直接前馈，持续转动不需要靠累积很大的位置误差来“推着走”；
    // 2. P 项消除位置偏差；
    // 3. D 项直接比较目标/实际角速度，不再对 50Hz 阶梯位置目标求导，避免周期性微分冲击。
    float positionError = posRef - posFdb;
    float rateError = posRateRef - velFdb;
    float targetVel = posRateRef +
                      (outer.kp * positionError) +
                      (outer.kd * rateError);
    targetVel = constrain(targetVel, -outer.maxOut, outer.maxOut);
    float innerOut = inner.calculate(targetVel, velFdb, dt);
    return constrain(innerOut + (ff_gain * chassisVel),
                     -Config::YAW_VOLTAGE_MAX,
                     Config::YAW_VOLTAGE_MAX);
}

void CascadePID::reset() {
    outer.reset();
    inner.reset();
}

float TrackVelocityController::runtimeKp = Config::TRACK_PID_KP;
float TrackVelocityController::runtimeKi = Config::TRACK_PID_KI;
float TrackVelocityController::runtimeKd = Config::TRACK_PID_KD;

TrackVelocityController::TrackVelocityController(bool leftTrack) : isLeft(leftTrack) {}

float TrackVelocityController::updateFeedbackMedian(float input) {
    medianSamples[medianSampleIndex] = input;
    medianSampleIndex = (medianSampleIndex + 1) % 3;
    if (medianSampleCount < 3) ++medianSampleCount;
    if (medianSampleCount < 3) return input;

    float a = medianSamples[0], b = medianSamples[1], c = medianSamples[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

float TrackVelocityController::updateNotch(NotchState& state, float input,
                                           float frequencyHz, float sampleRateHz) {
    if (!state.ready) {
        state.x1 = state.x2 = input;
        state.y1 = state.y2 = input;
        state.ready = true;
        return input;
    }

    // RBJ二阶陷波；Q决定带宽，中心频率随目标主动轮转频缓慢移动。
    float omega = 2.0f * PI * frequencyHz / sampleRateHz;
    float sinOmega = sinf(omega);
    float cosOmega = cosf(omega);
    float alpha = sinOmega / (2.0f * Config::ENCODER_NOTCH_Q);
    float invA0 = 1.0f / (1.0f + alpha);
    float b0 = invA0;
    float b1 = -2.0f * cosOmega * invA0;
    float b2 = invA0;
    float a1 = b1;
    float a2 = (1.0f - alpha) * invA0;

    float output = b0 * input + b1 * state.x1 + b2 * state.x2
                 - a1 * state.y1 - a2 * state.y2;
    state.x2 = state.x1;
    state.x1 = input;
    state.y2 = state.y1;
    state.y1 = output;
    return isfinite(output) ? output : input;
}

void TrackVelocityController::resetNotches(float seed) {
    notch = NotchState();
    notch.x1 = notch.x2 = notch.y1 = notch.y2 = seed;
    notch.ready = true;
    notchMix = 0.0f;
    notchFrequencySpeed = abs(seed);
}

void TrackVelocityController::clearFeedbackState(float seed) {
    integralPwm = 0.0f;
    heldPidPwm = 0.0f;
    lastActual = seed;
    filteredActualDerivative = 0.0f;
    derivativeReady = false;
    feedbackActual = seed;
    lastNotchTarget = 0.0f;
    notchTargetReady = false;
    medianSamples[0] = medianSamples[1] = medianSamples[2] = seed;
    medianSampleCount = 0;
    medianSampleIndex = 0;
    resetNotches(seed);
}

void TrackVelocityController::setRuntimePidGains(float kp, float ki, float kd) {
    runtimeKp = kp;
    runtimeKi = ki;
    runtimeKd = kd;
}

void TrackVelocityController::resetRuntimePidGains() {
    setRuntimePidGains(Config::TRACK_PID_KP,
                       Config::TRACK_PID_KI,
                       Config::TRACK_PID_KD);
}

void TrackVelocityController::getRuntimePidGains(float& kp, float& ki, float& kd) {
    kp = runtimeKp;
    ki = runtimeKi;
    kd = runtimeKd;
}

static float lookupTrackPwm(float speedKmh, bool reverse) {
    static const float speedPoints[] = {2.8f, 3.9f, 6.1f, 10.5f, 15.0f,
                                        19.0f, 24.0f, 28.0f, 35.0f};
    static const float pwmPoints[] = {35.0f, 40.0f, 50.0f, 70.0f, 90.0f,
                                      110.0f, 130.0f, 150.0f, 180.0f};
    constexpr size_t pointCount = sizeof(speedPoints) / sizeof(speedPoints[0]);

    speedKmh = max(0.0f, speedKmh);
    if (speedKmh <= speedPoints[0]) {
        float ratio = speedKmh / speedPoints[0];
        // 起步完成后的减速区必须能连续回到0 PWM；最低维持PWM只属于开环起步状态。
        return ratio * pwmPoints[0];
    }

    for (size_t i = 1; i < pointCount; ++i) {
        if (speedKmh <= speedPoints[i]) {
            float ratio = (speedKmh - speedPoints[i - 1]) /
                          (speedPoints[i] - speedPoints[i - 1]);
            return pwmPoints[i - 1] + ratio * (pwmPoints[i] - pwmPoints[i - 1]);
        }
    }

    // 35km/h以上暂按最后两点线性外推并由最终±255限幅；补测高PWM后可直接扩表。
    float lastSlope = (pwmPoints[pointCount - 1] - pwmPoints[pointCount - 2]) /
                      (speedPoints[pointCount - 1] - speedPoints[pointCount - 2]);
    return pwmPoints[pointCount - 1] +
           (speedKmh - speedPoints[pointCount - 1]) * lastSlope;
}

float TrackVelocityController::calculate(float target, float actual, float dt, float externalPwm,
                                         float batteryVoltage, bool brakingActive,
                                         bool motionDemandActive, bool pivotLaunch,
                                         bool newSpeedSample, float speedSampleDt) {
    (void)brakingActive;  // 普通制动不再旁路PI反馈陷波；安全判断使用外层快速速度。
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;
    externalPwm = constrain(externalPwm,
                            -Config::TRACK_EXTERNAL_PWM_MAX,
                            Config::TRACK_EXTERNAL_PWM_MAX);

    bool targetActive = abs(target) >= Config::TRACK_STOP_DEADZONE_KMH;
    float dir = (target > 0.0f) ? 1.0f : -1.0f;
    bool directionChanged = lastDir != 0.0f && targetActive && dir != lastDir;
    uint32_t nowMs = millis();
    float safeBatteryVoltage = batteryVoltage;
    if (!isfinite(safeBatteryVoltage) || safeBatteryVoltage < 1.0f) {
        safeBatteryVoltage = Config::TRACK_FF_REFERENCE_VOLTAGE;
    }
    float batteryScale = constrain(
        Config::TRACK_FF_REFERENCE_VOLTAGE / safeBatteryVoltage,
        Config::TRACK_FF_BATTERY_SCALE_MIN,
        Config::TRACK_FF_BATTERY_SCALE_MAX);

    // 起步失败锁存只在驾驶者真正松开该侧目标后解除，避免卡住时周期性自动重试。
    if (!motionDemandActive) launchLockout = false;

    // 正常减速与制动始终留在PI链路中，不在低速区另造PWM卸载曲线。
    // 只有底盘动力学目标真正归零或方向改变时，才结束本次闭环并清零输出。
    if (lowSpeedState == LowSpeedState::ClosedLoop &&
        (directionChanged || !targetActive)) {
        lowSpeedState = LowSpeedState::Stopped;
        lowSpeedPwm = 0.0f;
        lastOutputPwm = 0.0f;
        effectiveTarget = 0.0f;
        lastDir = 0.0f;
        clearFeedbackState(actual);
    }

    if (lowSpeedState == LowSpeedState::Launching &&
        (directionChanged || !targetActive)) {
        lowSpeedState = LowSpeedState::Stopped;
        lowSpeedPwm = 0.0f;
        lastOutputPwm = 0.0f;
        effectiveTarget = 0.0f;
        lastDir = 0.0f;
        clearFeedbackState(actual);
    }

    // 驾驶者在开环起步期间松开输入，但动力学目标尚未自然降到零时，立即把当前
    // PWM无扰交给PI继续减速；不能继续执行起步增力，也不能另行定时卸载。
    if (lowSpeedState == LowSpeedState::Launching &&
        !motionDemandActive && targetActive && !directionChanged) {
        lowSpeedState = LowSpeedState::ClosedLoop;
        effectiveTarget = target;
        float nominalPwm = lookupTrackPwm(abs(target), lastDir < 0.0f) *
                           batteryScale * lastDir;
        float initialError = target - actual;
        integralPwm = constrain(lowSpeedPwm - nominalPwm - externalPwm -
                                runtimeKp * initialError,
                                -Config::TRACK_PID_I_MAX_PWM,
                                Config::TRACK_PID_I_MAX_PWM);
        heldPidPwm = runtimeKp * initialError + integralPwm;
        resetNotches(actual);
    }

    // 一旦有明确非零履带目标就开始跨越机械死区，不再等待重车目标先慢慢爬到2.8km/h。
    // 但只有目标本身也进入可持续运行区后，才允许从开环起步切到闭环。
    if (lowSpeedState == LowSpeedState::Stopped && targetActive &&
        motionDemandActive && !launchLockout) {
        lowSpeedState = LowSpeedState::Launching;
        lowSpeedPwm = 0.0f;
        launchStartedMs = nowMs;
        launchStableEvidenceMs = 0.0f;
        launchDropSinceMs = 0;
        launchMotionSeen = false;
        launchPivotMode = pivotLaunch;
        lastDir = dir;
        clearFeedbackState(actual);
    }

    if (lowSpeedState == LowSpeedState::Stopped) {
        effectiveTarget = 0.0f;
        feedbackActual = actual;
        lastOutputPwm = 0.0f;
        return 0.0f;
    }

    if (lowSpeedState == LowSpeedState::Launching) {
        bool reverse = lastDir < 0.0f;
        float runLaunchMagnitude = (reverse ? Config::TRACK_FF_RUN_REVERSE_PWM
                                            : Config::TRACK_FF_RUN_FORWARD_PWM) * batteryScale;
        float baseLaunchMagnitude = (launchPivotMode ? Config::TRACK_PIVOT_START_PWM
            : (reverse ? Config::TRACK_FF_START_REVERSE_PWM
                       : Config::TRACK_FF_START_FORWARD_PWM)) * batteryScale;
        float maxLaunchMagnitude = (launchPivotMode ? Config::TRACK_PIVOT_MAX_PWM
            : (reverse ? Config::TRACK_LAUNCH_MAX_REVERSE_PWM
                       : Config::TRACK_LAUNCH_MAX_FORWARD_PWM)) * batteryScale;
        uint32_t launchElapsedMs = (uint32_t)(nowMs - launchStartedMs);
        if (newSpeedSample) {
            float evidenceStepMs = constrain(
                (speedSampleDt > 0.0f ? speedSampleDt : 0.02f) * 1000.0f,
                5.0f, 50.0f);

            float speedInLaunchDirection = actual * lastDir;
            if (speedInLaunchDirection >= Config::TRACK_LAUNCH_MOTION_SPEED_KMH) {
                launchMotionSeen = true;
                launchDropSinceMs = 0;
            } else if (speedInLaunchDirection < Config::TRACK_LAUNCH_DROP_SPEED_KMH) {
                if (launchDropSinceMs == 0) launchDropSinceMs = nowMs;
                if ((uint32_t)(nowMs - launchDropSinceMs) >=
                    Config::TRACK_LAUNCH_DROP_RESET_MS) {
                    launchMotionSeen = false;
                    launchStableEvidenceMs = 0.0f;
                }
            } else {
                launchDropSinceMs = 0;
            }

            if (launchMotionSeen &&
                speedInLaunchDirection >= Config::TRACK_LAUNCH_STABLE_SPEED_KMH) {
                launchStableEvidenceMs = min(
                    (float)Config::TRACK_LAUNCH_STABLE_EVIDENCE_MS,
                    launchStableEvidenceMs + evidenceStepMs);
            } else {
                launchStableEvidenceMs = max(0.0f,
                    launchStableEvidenceMs - 0.5f * evidenceStepMs);
            }
        }

        float desiredLaunchMagnitude = 0.0f;
        if (launchElapsedMs < Config::TRACK_LAUNCH_RUN_RAMP_MS) {
            desiredLaunchMagnitude = runLaunchMagnitude * launchElapsedMs /
                (float)max((uint32_t)1, Config::TRACK_LAUNCH_RUN_RAMP_MS);
        } else if (launchElapsedMs < Config::TRACK_LAUNCH_BOOST_DELAY_MS) {
            float startBlend = constrain(
                (launchElapsedMs - Config::TRACK_LAUNCH_RUN_RAMP_MS) /
                    (float)max((uint32_t)1, Config::TRACK_LAUNCH_START_RAMP_MS),
                0.0f, 1.0f);
            desiredLaunchMagnitude = runLaunchMagnitude + startBlend *
                (baseLaunchMagnitude - runLaunchMagnitude);
        } else {
            float boostBlend = constrain(
                (launchElapsedMs - Config::TRACK_LAUNCH_BOOST_DELAY_MS) /
                    (float)max((uint32_t)1, Config::TRACK_LAUNCH_BOOST_RAMP_MS),
                0.0f, 1.0f);
            desiredLaunchMagnitude = baseLaunchMagnitude + boostBlend *
                (maxLaunchMagnitude - baseLaunchMagnitude);
        }
        float launchPwm = desiredLaunchMagnitude * lastDir;
        lowSpeedPwm = launchPwm;

        effectiveTarget = lastDir * Config::TRACK_MIN_CLOSED_LOOP_SPEED_KMH;
        feedbackActual = actual;

        if (launchStableEvidenceMs < Config::TRACK_LAUNCH_STABLE_EVIDENCE_MS &&
            launchElapsedMs >= Config::TRACK_LAUNCH_FAIL_TIMEOUT_MS) {
            // 没有任何可信运动，视为起步失败/可能被卡住。卸载后等待松开目标再重试。
            launchLockout = true;
            lowSpeedState = LowSpeedState::Stopped;
            lowSpeedPwm = 0.0f;
            clearFeedbackState(actual);
            lastOutputPwm = 0.0f;
            return 0.0f;
        }
        if (launchStableEvidenceMs >= Config::TRACK_LAUNCH_STABLE_EVIDENCE_MS) {
            lowSpeedState = LowSpeedState::ClosedLoop;
            // 接管瞬间用当前实际速度建立等效PI工作点，但不覆盖底盘动力学目标。
            float nominalPwm = lookupTrackPwm(Config::TRACK_MIN_CLOSED_LOOP_SPEED_KMH,
                                              reverse) * batteryScale * lastDir;
            float initialCorrection = lowSpeedPwm - nominalPwm - externalPwm;
            float initialError = effectiveTarget - actual;
            integralPwm = constrain(initialCorrection - runtimeKp * initialError,
                                    -Config::TRACK_PID_I_MAX_PWM,
                                    Config::TRACK_PID_I_MAX_PWM);
            heldPidPwm = runtimeKp * initialError + integralPwm;
            resetNotches(actual);
        }
        lastOutputPwm = constrain(lowSpeedPwm, -255.0f, 255.0f);
        return lastOutputPwm;
    }

    // 起步完成后严格追随底盘动力学目标，包括从2.8km/h以下连续减速到零。
    // 机构最低稳定速度只用于开环起步判断，不再钳制正常PI目标。
    effectiveTarget = target;
    targetActive = true;
    dir = lastDir;

    bool reverse = dir < 0.0f;
    float baseFf = lookupTrackPwm(abs(target), reverse) * batteryScale * dir;

    // 新测速到达时，PI反馈支路对实测主动轮9阶啮合扰动做窄带抑制。
    // 换向、接近停车和高速采样余量不足时旁路；停车/堵转始终由快速速度判断。
    if (newSpeedSample) {
        float sampleDt = speedSampleDt > 0.0f ? speedSampleDt : 0.02f;
        sampleDt = constrain(sampleDt, 0.005f, 0.05f);
        // 主动轮扰动频率由实际转速决定。使用慢速跟踪的实际速度定位陷波中心，
        // 避免加减速时目标速度与真实主动轮转频相差较大而陷波偏离。
        float frequencyBlend = sampleDt /
            (Config::ENCODER_NOTCH_CENTER_TAU_S + sampleDt);
        float medianActual = updateFeedbackMedian(actual);
        notchFrequencySpeed += frequencyBlend *
            (abs(medianActual) - notchFrequencySpeed);
        bool directionMismatch = target * actual < 0.0f;
        // 普通松油和刹车继续使用陷波；只有方向不一致/换向以及接近停车时旁路。
        // 停车和堵转保护始终使用外层未陷波的40ms快速速度。
        bool filterAllowed = Config::ENCODER_NOTCH_ENABLED && targetActive &&
                             !directionMismatch;

        float desiredMix = 0.0f;
        if (filterAllowed) {
            float lowSpeedMix = constrain(
                (notchFrequencySpeed - Config::ENCODER_NOTCH_FADE_START_KMH) /
                (Config::ENCODER_NOTCH_FADE_FULL_KMH -
                 Config::ENCODER_NOTCH_FADE_START_KMH),
                0.0f, 1.0f);
            float highSpeedMix = constrain(
                (Config::ENCODER_NOTCH_HIGH_FADE_END_KMH - notchFrequencySpeed) /
                (Config::ENCODER_NOTCH_HIGH_FADE_END_KMH -
                 Config::ENCODER_NOTCH_HIGH_FADE_START_KMH),
                0.0f, 1.0f);
            desiredMix = lowSpeedMix * highSpeedMix * Config::ENCODER_NOTCH_MAX_MIX;
        }

        if (!filterAllowed || desiredMix <= 0.0f) {
            resetNotches(medianActual);
            feedbackActual = medianActual;
        } else {
            float sampleRateHz = 1.0f / sampleDt;
            float notchHz = Config::ENCODER_NOTCH_ORDER *
                Config::ENCODER_SPROCKET_HZ_PER_KMH * notchFrequencySpeed;
            notchHz = constrain(notchHz, 0.10f, sampleRateHz * 0.45f);
            float notchOutput = updateNotch(notch, medianActual, notchHz, sampleRateHz);
            float blend = sampleDt /
                (Config::ENCODER_NOTCH_BLEND_TAU_S + sampleDt);
            notchMix += blend * (desiredMix - notchMix);
            feedbackActual = medianActual + notchMix * (notchOutput - medianActual);
        }
        lastNotchTarget = target;
        notchTargetReady = true;
    }

    // D 对PI实际反馈求导，且只在50Hz新样本时更新。KD默认仍为0。
    if (newSpeedSample) {
        if (derivativeReady && speedSampleDt > 0.0f) {
            float rawActualDerivative = (feedbackActual - lastActual) / speedSampleDt;
            float derivativeAlpha = speedSampleDt /
                (Config::TRACK_PID_D_FILTER_TAU_S + speedSampleDt);
            filteredActualDerivative += derivativeAlpha *
                (rawActualDerivative - filteredActualDerivative);
        } else {
            derivativeReady = true;
            filteredActualDerivative = 0.0f;
        }
        lastActual = feedbackActual;
    }
    float dPwm = constrain(-runtimeKd * filteredActualDerivative,
                           -Config::TRACK_PID_D_MAX_PWM,
                           Config::TRACK_PID_D_MAX_PWM);

    lastDir = targetActive ? dir : 0.0f;

    float error = target - feedbackActual;
    // 反馈修正只在50Hz新测速到达时更新。目标动力学、查表前馈和受限IMU项
    // 仍可在200Hz外层刷新；两次测速之间保持上一次PI修正。
    if (newSpeedSample) {
        float feedbackDt = speedSampleDt > 0.0f ? speedSampleDt : dt;
        feedbackDt = constrain(feedbackDt, 0.001f, 0.05f);
        if (abs(target) < Config::TRACK_STOP_DEADZONE_KMH) {
            integralPwm *= 0.5f;
        } else {
        // integralPwm 直接以 PWM 为单位，便于理解其权限。只有最终电机输出饱和且
        // 误差仍把输出推向同一方向时才暂停积分，反向误差始终可以帮助退出饱和。
        float candidateIntegralPwm = constrain(
            integralPwm + runtimeKi * error * feedbackDt,
            -Config::TRACK_PID_I_MAX_PWM,
            Config::TRACK_PID_I_MAX_PWM);
        float pPwm = runtimeKp * error;
        float candidateOutput = baseFf + pPwm + candidateIntegralPwm + dPwm + externalPwm;
        bool pushingHighSaturation = error > 0.0f && candidateOutput > 255.0f;
        bool pushingLowSaturation = error < 0.0f && candidateOutput < -255.0f;
        if (!pushingHighSaturation && !pushingLowSaturation) {
            integralPwm = candidateIntegralPwm;
        }
        }
        integralPwm = constrain(integralPwm,
                                -Config::TRACK_PID_I_MAX_PWM,
                                Config::TRACK_PID_I_MAX_PWM);
        heldPidPwm = (runtimeKp * error) + integralPwm + dPwm;
    }
    lastOutputPwm = constrain(baseFf + heldPidPwm + externalPwm, -255.0f, 255.0f);
    return lastOutputPwm;
}

void TrackVelocityController::reset() {
    integralPwm = 0.0f;
    heldPidPwm = 0.0f;
    lastActual = 0.0f;
    filteredActualDerivative = 0.0f;
    derivativeReady = false;
    lowSpeedState = LowSpeedState::Stopped;
    launchStartedMs = 0;
    launchStableEvidenceMs = 0.0f;
    launchDropSinceMs = 0;
    launchMotionSeen = false;
    launchPivotMode = false;
    launchLockout = false;
    lowSpeedPwm = 0.0f;
    lastOutputPwm = 0.0f;
    effectiveTarget = 0.0f;
    lastDir = 0.0f;
    feedbackActual = 0.0f;
    lastNotchTarget = 0.0f;
    notchTargetReady = false;
    resetNotches();
}
