#pragma once

#include <Arduino.h>
#include "config/RobotConfig.h"

// ==========================================
// 2. 基础控制算法
// ==========================================

// 通用 PID：输入目标值、实际值和 dt，输出带限幅的控制量。
// 目前主要给炮塔 yaw 的串级控制使用，也可以复用到其他闭环。
class CustomPID {
public:
    float kp, ki, kd, maxOut, maxI;     // 参数允许运行时被 Config 刷新
    float integral = 0, prevError = 0;  // 积分和上一次误差是控制器内部状态

    // p/i/d：PID 参数；mi：积分限幅；mo：最终输出限幅。
    CustomPID(float p, float i, float d, float mi, float mo);

    // 计算一次 PID 输出。dt 单位是秒，函数内部会保护 dt<=0 的情况。
    float calculate(float target, float actual, float dt);

    // 清空积分和历史误差，适合模式切换、断连、进入安全状态时调用。
    void reset();
};

// 炮塔 yaw 串级 PID：外环把角度误差变成目标角速度，内环把角速度误差变成电压。
// chassisVel 是底盘 yaw 角速度前馈，用来抵消车体旋转对炮塔指向的影响。
class CascadePID {
public:
    CustomPID outer;
    CustomPID inner;
    float ff_gain;

    CascadePID(CustomPID out, CustomPID in, float ff = 0.0f);

    // posRef/posFdb 单位是度；posRateRef/velFdb/chassisVel 单位是 deg/s。
    // posRateRef 是手动目标角速度前馈；外环 D 对速度误差计算，不再对阶梯位置目标求导。
    float calculate(float posRef, float posFdb, float posRateRef,
                    float velFdb, float chassisVel, float dt);

    void reset();
};

// 单侧履带速度控制器。
// 速度-PWM查表提供正常运行所需的基础动力，低增益 PID 只修正负载和路面误差。
class TrackVelocityController {
private:
    enum class LowSpeedState : uint8_t {
        Stopped,
        Launching,
        ClosedLoop
    };

    struct NotchState {
        float x1 = 0.0f, x2 = 0.0f;
        float y1 = 0.0f, y2 = 0.0f;
        bool ready = false;
    };

    static float runtimeKp;
    static float runtimeKi;
    static float runtimeKd;

    bool isLeft;
    float integralPwm = 0.0f;
    float lastActual = 0.0f;
    float heldPidPwm = 0.0f;
    float filteredActualDerivative = 0.0f;
    bool derivativeReady = false;
    LowSpeedState lowSpeedState = LowSpeedState::Stopped;
    uint32_t launchStartedMs = 0;
    float launchStableEvidenceMs = 0.0f;
    uint32_t launchDropSinceMs = 0;
    bool launchMotionSeen = false;
    bool launchPivotMode = false;
    bool launchLockout = false;
    float lowSpeedPwm = 0.0f;
    float lastOutputPwm = 0.0f;
    float effectiveTarget = 0.0f;
    float lastDir = 0.0f;
    float feedbackActual = 0.0f;
    float notchMix = 0.0f;
    float notchFrequencySpeed = 0.0f;
    float lastNotchTarget = 0.0f;
    bool notchTargetReady = false;
    NotchState notch;
    float medianSamples[3] = {};
    uint8_t medianSampleCount = 0;
    uint8_t medianSampleIndex = 0;

    float updateNotch(NotchState& state, float input, float frequencyHz,
                      float sampleRateHz);
    float updateFeedbackMedian(float input);
    void resetNotches(float seed = 0.0f);
    void clearFeedbackState(float seed = 0.0f);

public:
    explicit TrackVelocityController(bool leftTrack = false);

    // 左右履带共享同一组运行时 PID 增益；只保存在 RAM，重启恢复 Config 默认值。
    static void setRuntimePidGains(float kp, float ki, float kd);
    static void resetRuntimePidGains();
    static void getRuntimePidGains(float& kp, float& ki, float& kd);

    // target/actual 单位是 km/h；dt 单位是秒；newSpeedSample 表示编码器刚产生新测速。
    // speedSampleDt 是该测速样本的真实累计窗口，D 项只在此时更新。
    // 返回值是 -255~255 的电机 PWM 命令。
    float calculate(float target, float actual, float dt, float externalPwm, float batteryVoltage,
                    bool brakingActive, bool motionDemandActive, bool pivotLaunch,
                    bool newSpeedSample, float speedSampleDt);

    // 返回PI真正使用的反馈速度；安全判断仍使用未陷波的40ms快速速度。
    float getFeedbackActual() const { return feedbackActual; }
    float getEffectiveTarget() const { return effectiveTarget; }
    bool isClosedLoop() const { return lowSpeedState == LowSpeedState::ClosedLoop; }

    // 清空积分、起步补偿和历史目标，适合停车、堵转保护和模式切换。
    void reset();
};
