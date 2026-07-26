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

float CascadePID::calculate(float posRef, float posFdb, float velFdb, float chassisVel, float dt) {
    // yaw 参数放在 Config 里，允许调参时只改配置，不用追到构造函数。
    outer.kp = Config::YAW_OUTER_KP;
    outer.kd = Config::YAW_OUTER_KD;
    outer.maxOut = Config::YAW_OUTER_RATE_MAX;
    inner.kp = Config::YAW_INNER_KP;
    inner.ki = Config::YAW_INNER_KI;
    inner.kd = Config::YAW_INNER_KD;
    inner.maxOut = Config::YAW_VOLTAGE_MAX;
    ff_gain = Config::YAW_CHASSIS_FF_GAIN;

    float targetVel = outer.calculate(posRef, posFdb, dt);
    float innerOut = inner.calculate(targetVel, velFdb, dt);
    return constrain(innerOut + (ff_gain * chassisVel),
                     -Config::YAW_VOLTAGE_MAX,
                     Config::YAW_VOLTAGE_MAX);
}

void CascadePID::reset() {
    outer.reset();
    inner.reset();
}

float TrackVelocityController::calculate(float target, float actual, float dt, float externalPwm) {
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
        startBoostSinceMs = millis();
    }

    // 起步静摩擦补偿只短时间生效；一旦速度起来或超时，就切回运行前馈。
    float releaseSpeed = max(Config::TRACK_START_RELEASE_MIN_KMH,
                             abs(target) * Config::TRACK_START_RELEASE_RATIO);
    bool boostTimedOut = startBoostActive &&
                         ((uint32_t)(millis() - startBoostSinceMs) >= Config::TRACK_START_BOOST_MAX_MS);
    bool boostSpeedReached = startBoostActive && abs(actual) >= releaseSpeed;
    if (!targetActive || boostTimedOut || boostSpeedReached) {
        startBoostActive = false;
    }

    float targetAccel = 0.0f;
    if (targetActive && wasTargetActive) {
        targetAccel = constrain((target - lastTarget) / dt,
                                -Config::TRACK_FF_MAX_ACCEL,
                                Config::TRACK_FF_MAX_ACCEL);
    }
    filteredTargetAccel += Config::TRACK_FF_ACCEL_LPF * (targetAccel - filteredTargetAccel);
    lastTarget = target;
    wasTargetActive = targetActive;

    float ff = 0.0f;
    if (targetActive) {
        float staticFf = startBoostActive ? Config::TRACK_FF_KS_START : Config::TRACK_FF_KS_RUN;
        ff = (staticFf * dir) +
             (Config::TRACK_FF_KV * target) +
             (Config::TRACK_FF_KA * filteredTargetAccel);
    }
    lastDir = targetActive ? dir : 0.0f;

    float error = target - actual;
    if (abs(target) < Config::TRACK_STOP_DEADZONE_KMH) {
        integral *= 0.9f;
    } else {
        integral += error * dt;
    }
    integral = constrain(integral, -Config::TRACK_PI_MAX_I, Config::TRACK_PI_MAX_I);

    // PI 只做小范围纠偏，避免编码器噪声把 PWM 拉得太猛。
    float correction = constrain((Config::TRACK_PI_KP * error) +
                                 (Config::TRACK_PI_KI * integral),
                                 -Config::TRACK_PI_MAX_CORRECTION,
                                 Config::TRACK_PI_MAX_CORRECTION);

    return constrain(ff + correction + externalPwm, -255.0f, 255.0f);
}

void TrackVelocityController::reset() {
    integral = 0.0f;
    lastTarget = 0.0f;
    filteredTargetAccel = 0.0f;
    wasTargetActive = false;
    startBoostActive = false;
    startBoostSinceMs = 0;
    lastDir = 0.0f;
}
