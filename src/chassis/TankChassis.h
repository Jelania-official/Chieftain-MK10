#pragma once

#include <Arduino.h>
#include "config/DebugLog.h"
#include "config/RobotConfig.h"
#include "control/Controllers.h"
#include "hardware/DriveHardware.h"

// ==========================================
// 4. 底盘系统
// ==========================================
// 底盘分两层：
// - TankTrack：单侧履带，负责电机、编码器、速度闭环和堵转保护；
// - TankChassis：整车底盘，把手柄油门/转向转换成左右履带目标速度。

class TankTrack {
public:
    DCMotor motor;
    CustomEncoder encoder;
    TrackVelocityController controller;

    // 这几个量用于调试和上层读取 telemetry，单位见各字段注释。
    float currentSpeed = 0; // 控制和堵转判断使用的编码器速度，km/h
    float telemetrySpeed = 0; // 仅供上位机绘图的平滑速度，km/h
    float targetSpeed = 0;  // 当前目标速度，km/h
    float lastPwm = 0;      // 上一次输出给电机的 PWM，-255~255
    bool stallLatched = false; // 堵转锁存；松开目标速度后自动解除

private:
    const char* label;
    uint32_t stallCandidateSinceMs = 0;
    uint32_t lastEncoderControlSampleId = 0;

    // 目标速度回到低速区后，清除堵转锁存。
    void clearStallIfReleased();

    // 根据“高目标 + 高 PWM + 低实际速度 + 持续时间”判断履带可能堵转。
    bool updateStallProtection();

public:
    TankTrack(DCMotor m, CustomEncoder e, const char* trackLabel, bool leftTrack);

    // 初始化电机 PWM 和编码器 PCNT。
    void init();

    // 更新单侧履带速度闭环。target 单位 km/h，dt 单位秒，externalPwm 是外部补偿 PWM。
    void update(float target, float dt, float externalPwm);

    // 硬件诊断入口：绕过速度控制器和堵转判断，直接输出受限 PWM，同时保留测速。
    void driveDirect(float pwm);

    // 停止电机、清空控制器和堵转状态。
    void stop();
};

// 油门/刹车平滑器。
// rise 控制输入增大时的爬升速度，fall 控制松开或减小时的下降速度。
class ThrottleSmoother {
private:
    float current_val = 0.0f;
    float rise_rate;
    float fall_rate;

public:
    ThrottleSmoother(float rise, float fall);

    // 让 current_val 以给定速率靠近 target，返回平滑后的值。
    float update(float target, float dt);

    // 立即清零，适合刹停、断连、换向时调用。
    void reset();
};

class TankChassis {
private:
    TankTrack rightTrack, leftTrack;

    // v_real 是整车纵向速度，spinV 是左右履带差速形成的自转速度分量，单位 km/h。
    float v_real = 0, spinV = 0;

    ThrottleSmoother engineSmoother;
    ThrottleSmoother brakeSmoother;

    // 底盘动力学内部状态，用于纵向加速度、坡度滤波和虚拟惯量补偿。
    float longitudinalAccel = 0.0f;
    float gradePitchDeg = 0.0f;
    bool gradePitchReady = false;
    float lastPitchRateDeg = 0.0f;
    float filteredPitchAlpha = 0.0f;
    bool pitchRateReady = false;
    float lastYawRateDeg = 0.0f;
    float filteredYawAlpha = 0.0f;
    bool yawRateReady = false;

    float moveToward(float current, float target, float maxDelta);

    // 根据底盘 pitch 角加速度生成前后同向 PWM，模拟车体俯仰虚拟惯量。
    float calculateVirtualInertiaPwm(float pitchRateDeg, float dt);

    // 根据底盘 yaw 角加速度生成左右差速 PWM，模拟车体转动虚拟惯量。
    float calculateYawInertiaPwm(float yawRateDeg, float dt);

    // 对 IMU 给出的坡度角做低通，避免坡度重力分量抖动。
    float updateGradePitch(float pitchAngleDeg, float dt);

public:
    TankChassis();

    // 初始化左右履带。
    void init();

    // 仅清除坡度和虚拟惯量历史，不停车、不重置履带速度闭环。
    void resetImuCompensation();

    // 主底盘控制入口。
    // triggerL/triggerR：0~1 的倒车/前进输入；joyX：-1~1 的转向输入。
    // currentPitchRate/currentYawRate/pitchAngle 来自炮塔模块里的底盘 IMU，用于惯量和坡度补偿。
    void processKinematics(float triggerL, float triggerR, float joyX, float dt,
                           float currentPitchRate, float currentYawRate, float pitchAngle);

    // 调试/测试入口：绕过手柄动力学，直接指定左右履带目标速度，单位 km/h。
    void processDirectTrackTargets(float leftTarget, float rightTarget, float dt, float pitchAngle);

    // PC 固定 PWM 硬件诊断入口；仅由带超时和急停保护的 DebugLink 调用。
    void processDirectTrackPwm(float leftPwm, float rightPwm);

    // PC 调参入口：左右履带共享增益，但各自控制器状态和反馈保持独立。
    void setTrackPidGains(float kp, float ki, float kd);
    void resetTrackPidGains();
    void getTrackPidGains(float& kp, float& ki, float& kd) const;

    // 控制速度是 PI/堵转保护实际使用的反馈；显示速度只供上位机平滑绘图。
    void getTrackTelemetry(float& leftTarget, float& leftControlActual, float& leftDisplayActual,
                           float& leftPwm, bool& leftStalled,
                           float& rightTarget, float& rightControlActual, float& rightDisplayActual,
                           float& rightPwm, bool& rightStalled) const;

    // 立即停止底盘，并清空所有平滑器、滤波器和履带控制器状态。
    void stop();
};
