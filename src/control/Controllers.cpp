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

float TrackVelocityController::calculate(float target, float actual, float dt, float externalPwm,
                                         bool newSpeedSample, float speedSampleDt) {
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;
    externalPwm = constrain(externalPwm,
                            -Config::TRACK_EXTERNAL_PWM_MAX,
                            Config::TRACK_EXTERNAL_PWM_MAX);

    if (abs(target) < Config::TRACK_STOP_DEADZONE_KMH &&
        abs(actual) < Config::TRACK_STOP_DEADZONE_KMH) {
        reset();
        return externalPwm;
    }

    bool targetActive = abs(target) >= Config::TRACK_STOP_DEADZONE_KMH;
    float dir = (target > 0.0f) ? 1.0f : -1.0f;
    if (targetActive && (!wasTargetActive || dir != lastDir)) {
        startBoostActive = true;
        startReleaseCandidateSinceMs = 0;
        startReenterCandidateSinceMs = 0;
        startBoostBlend = 1.0f;
    }

    // 静止时使用 KS_START；同方向速度稳定超过固定门限后切换到 KS_RUN。
    // 运行中如果持续接近零速则重新进入起步状态。两个门限形成迟滞，避免来回切换。
    uint32_t nowMs = millis();
    if (!targetActive) {
        startReleaseCandidateSinceMs = 0;
        startReenterCandidateSinceMs = 0;
    } else if (startBoostActive) {
        if (actual * dir >= Config::TRACK_START_RELEASE_SPEED_KMH) {
            if (startReleaseCandidateSinceMs == 0) startReleaseCandidateSinceMs = nowMs;
            if ((uint32_t)(nowMs - startReleaseCandidateSinceMs) >=
                Config::TRACK_START_RELEASE_CONFIRM_MS) {
                startBoostActive = false;
                startReleaseCandidateSinceMs = 0;
            }
        } else {
            startReleaseCandidateSinceMs = 0;
        }
    } else {
        if (actual * dir <= Config::TRACK_START_REENTER_SPEED_KMH) {
            if (startReenterCandidateSinceMs == 0) startReenterCandidateSinceMs = nowMs;
            if ((uint32_t)(nowMs - startReenterCandidateSinceMs) >=
                Config::TRACK_START_REENTER_CONFIRM_MS) {
                startBoostActive = true;
                startBoostBlend = 1.0f;
                startReenterCandidateSinceMs = 0;
            }
        } else {
            startReenterCandidateSinceMs = 0;
        }
    }

    // 起步补偿退出后短时间平滑降到滑动摩擦补偿，避免 PWM 台阶。
    if (!targetActive) {
        startBoostBlend = 0.0f;
    } else if (startBoostActive) {
        startBoostBlend = 1.0f;
    } else if (Config::TRACK_START_BLEND_DOWN_MS > 0) {
        float blendStep = dt * 1000.0f / Config::TRACK_START_BLEND_DOWN_MS;
        startBoostBlend = max(0.0f, startBoostBlend - blendStep);
    } else {
        startBoostBlend = 0.0f;
    }

    float frictionFf = 0.0f;
    if (targetActive) {
        float ksStart = isLeft ? Config::TRACK_FF_KS_START_LEFT : Config::TRACK_FF_KS_START_RIGHT;
        float ksRun = isLeft ? Config::TRACK_FF_KS_RUN_LEFT : Config::TRACK_FF_KS_RUN_RIGHT;
        float staticFf = ksRun + startBoostBlend * (ksStart - ksRun);
        frictionFf = staticFf * dir;
    }

    // D 对实际速度求导，且只在编码器给出新样本时更新，避免用 5ms 控制周期
    // 对 20~100ms 的阶梯测速差分而产生虚假尖峰。KD=0 时代码保留但不输出。
    if (newSpeedSample) {
        if (derivativeReady && speedSampleDt > 0.0f) {
            float rawActualDerivative = (actual - lastActual) / speedSampleDt;
            float derivativeAlpha = speedSampleDt /
                (Config::TRACK_PID_D_FILTER_TAU_S + speedSampleDt);
            filteredActualDerivative += derivativeAlpha *
                (rawActualDerivative - filteredActualDerivative);
        } else {
            derivativeReady = true;
            filteredActualDerivative = 0.0f;
        }
        lastActual = actual;
    }
    float dPwm = constrain(-runtimeKd * filteredActualDerivative,
                           -Config::TRACK_PID_D_MAX_PWM,
                           Config::TRACK_PID_D_MAX_PWM);

    wasTargetActive = targetActive;
    lastDir = targetActive ? dir : 0.0f;

    float error = target - actual;
    if (abs(target) < Config::TRACK_STOP_DEADZONE_KMH) {
        integralPwm *= 0.9f;
    } else {
        // integralPwm 直接以 PWM 为单位，便于理解其权限。只有最终电机输出饱和且
        // 误差仍把输出推向同一方向时才暂停积分，反向误差始终可以帮助退出饱和。
        float candidateIntegralPwm = constrain(
            integralPwm + runtimeKi * error * dt,
            -Config::TRACK_PID_I_MAX_PWM,
            Config::TRACK_PID_I_MAX_PWM);
        float pPwm = runtimeKp * error;
        float candidateOutput = frictionFf + pPwm + candidateIntegralPwm + dPwm + externalPwm;
        bool pushingHighSaturation = error > 0.0f && candidateOutput > 255.0f;
        bool pushingLowSaturation = error < 0.0f && candidateOutput < -255.0f;
        if (!pushingHighSaturation && !pushingLowSaturation) {
            integralPwm = candidateIntegralPwm;
        }
    }
    integralPwm = constrain(integralPwm,
                            -Config::TRACK_PID_I_MAX_PWM,
                            Config::TRACK_PID_I_MAX_PWM);

    float pidPwm = (runtimeKp * error) + integralPwm + dPwm;
    return constrain(frictionFf + pidPwm + externalPwm, -255.0f, 255.0f);
}

void TrackVelocityController::reset() {
    integralPwm = 0.0f;
    lastActual = 0.0f;
    filteredActualDerivative = 0.0f;
    derivativeReady = false;
    wasTargetActive = false;
    startBoostActive = false;
    startReleaseCandidateSinceMs = 0;
    startReenterCandidateSinceMs = 0;
    startBoostBlend = 0.0f;
    lastDir = 0.0f;
}
