#pragma once

#include <Arduino.h>
#include "../config/RobotConfig.h"

// ==========================================
// 2. 基础控制算法 (PID)
// ==========================================
// 最底层的通用 PID，给炮塔级联控制使用。
class CustomPID {
public:
    float kp, ki, kd, maxOut, maxI;
    float integral = 0, prevError = 0;
    CustomPID(float p, float i, float d, float mi, float mo) 
        : kp(p), ki(i), kd(d), maxI(mi), maxOut(mo) {}

    float calculate(float target, float actual, float dt) {
        if (dt <= 0.0f) dt = 0.001f;
        float error = target - actual;
        integral += error * dt;
        integral = constrain(integral, -maxI, maxI);
        float derivative = (error - prevError) / dt;
        prevError = error;
        return constrain(kp * error + ki * integral + kd * derivative, -maxOut, maxOut);
    }
    void reset() { integral = 0; prevError = 0; }
};

// 炮塔 yaw 用的是位置环套速度环的级联结构，外环给目标角速度，内环出最终驱动量。
class CascadePID {
public:
    CustomPID outer; CustomPID inner; float ff_gain;
    CascadePID(CustomPID out, CustomPID in, float ff = 0.0f) : outer(out), inner(in), ff_gain(ff) {}
    float calculate(float posRef, float posFdb, float velFdb, float chassisVel, float dt) {
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
    void reset() { outer.reset(); inner.reset(); }
};

// 履带速度控制：前馈承担主要 PWM，PI 只修正编码器反馈误差。
class TrackVelocityController {
private:
    float integral = 0.0f;
    float lastTarget = 0.0f;
    float filteredTargetAccel = 0.0f;
    bool wasTargetActive = false;
    bool startBoostActive = false;
    uint32_t startBoostSinceMs = 0;
    float lastDir = 0.0f;

public:
    float calculate(float target, float actual, float dt, float externalPwm) {
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

        float correction = constrain((Config::TRACK_PI_KP * error) +
                                     (Config::TRACK_PI_KI * integral),
                                     -Config::TRACK_PI_MAX_CORRECTION,
                                     Config::TRACK_PI_MAX_CORRECTION);

        return constrain(ff + correction + externalPwm, -255.0f, 255.0f);
    }

    void reset() {
        integral = 0.0f;
        lastTarget = 0.0f;
        filteredTargetAccel = 0.0f;
        wasTargetActive = false;
        startBoostActive = false;
        startBoostSinceMs = 0;
        lastDir = 0.0f;
    }
};


