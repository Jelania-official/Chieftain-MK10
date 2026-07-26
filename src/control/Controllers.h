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

    // posRef/posFdb 单位是度；velFdb/chassisVel 单位是 deg/s；返回 yaw 电机目标电压。
    float calculate(float posRef, float posFdb, float velFdb, float chassisVel, float dt);

    void reset();
};

// 单侧履带速度控制器。
// 前馈负责主要 PWM，PI 只根据编码器反馈做修正；externalPwm 用于底盘虚拟惯量补偿。
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
    // target/actual 单位是 km/h；dt 单位是秒；externalPwm 会被限幅后叠加到输出。
    // 返回值是 -255~255 的电机 PWM 命令。
    float calculate(float target, float actual, float dt, float externalPwm);

    // 清空积分、起步补偿和历史目标，适合停车、堵转保护和模式切换。
    void reset();
};
