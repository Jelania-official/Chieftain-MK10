#pragma once

#include <Arduino.h>
#include "../config/DebugLog.h"
#include "../config/RobotConfig.h"
#include "../control/Controllers.h"
#include "../hardware/DriveHardware.h"

// ==========================================
// 4. 底盘系统 (真车物理模拟)
// ==========================================
// 单侧履带 = 电机 + 编码器 + 前馈/PI 速度控制器。
class TankTrack {
public:
    DCMotor motor; CustomEncoder encoder; TrackVelocityController controller;
    float currentSpeed = 0, targetSpeed = 0;
    float lastPwm = 0;
    bool stallLatched = false;
private:
    const char* label;
    uint32_t stallCandidateSinceMs = 0;

    void clearStallIfReleased() {
        if (abs(targetSpeed) <= Config::TRACK_STALL_CLEAR_TARGET_KMH) {
            stallLatched = false;
            stallCandidateSinceMs = 0;
        }
    }

    bool updateStallProtection() {
        if (abs(targetSpeed) < Config::TRACK_STALL_TARGET_MIN_KMH ||
            abs(lastPwm) < Config::TRACK_STALL_PWM_MIN ||
            abs(currentSpeed) > Config::TRACK_STALL_ACTUAL_MAX_KMH) {
            stallCandidateSinceMs = 0;
            return false;
        }

        uint32_t nowMs = millis();
        if (stallCandidateSinceMs == 0) {
            stallCandidateSinceMs = nowMs;
            return false;
        }

        if ((uint32_t)(nowMs - stallCandidateSinceMs) < Config::TRACK_STALL_GRACE_MS) {
            return false;
        }

        stallLatched = true;
        stallCandidateSinceMs = 0;
        controller.reset();
        LOG_ALWAYS("!!! Track stall latched: %s target=%.2f actual=%.2f pwm=%.1f\n",
                   label, targetSpeed, currentSpeed, lastPwm);
        return true;
    }

public:
    TankTrack(DCMotor m, CustomEncoder e, const char* trackLabel) : motor(m), encoder(e), label(trackLabel) {}
    void init() { motor.init(); encoder.init(); }
    void update(float target, float dt, float externalPwm) {
        targetSpeed = target;
        currentSpeed = encoder.getRealSpeedKMH();
        if (abs(targetSpeed) < Config::TRACK_STOP_DEADZONE_KMH) targetSpeed = 0;
        clearStallIfReleased();

        if (stallLatched) {
            lastPwm = 0;
            motor.drive(0);
            return;
        }

        lastPwm = controller.calculate(targetSpeed, currentSpeed, dt, externalPwm);
        if (updateStallProtection()) {
            lastPwm = 0;
            motor.drive(0);
            return;
        }
        motor.drive(lastPwm);
    }
    void stop() {
        targetSpeed = 0;
        lastPwm = 0;
        stallLatched = false;
        stallCandidateSinceMs = 0;
        controller.reset();
        motor.drive(0);
    }
};

// 专为重型内燃机设计的油门平滑器：踩油门迟滞(模拟涡轮/转速爬升)，松油门瞬间切断
class ThrottleSmoother {
private:
    float current_val = 0.0f;
    float rise_rate; // 踩下时的爬升速度 (对应引擎迟滞)
    float fall_rate; // 松开时的下降速度 (几乎瞬间)

public:
    ThrottleSmoother(float rise, float fall) : rise_rate(rise), fall_rate(fall) {}

    float update(float target, float dt) {
        if (target > current_val) {
            // 踩油门：缓慢建立扭矩
            current_val = min(current_val + rise_rate * dt, target);
        } else {
            // 松油门/刹车：极其迅速地卸载扭矩
            current_val = max(current_val - fall_rate * dt, target);
        }
        return current_val;
    }
    void reset() { current_val = 0.0f; }
};

// 底盘总控：把手柄输入解释成“发动机推力/刹车/阻力/坡度”的合力，再映射到双履带。
class TankChassis {
private:
    TankTrack rightTrack, leftTrack;
    float v_real = 0, spinV = 0; 

    // 引擎油门建立缓慢 (0.8)，但松油门切断极快 (10.0)
    ThrottleSmoother engineSmoother; 
    
    // 刹车液压建立较快 (3.0)，松开也快 (10.0)
    ThrottleSmoother brakeSmoother;  

    float longitudinalAccel = 0.0f;
    float gradePitchDeg = 0.0f;
    bool gradePitchReady = false;
    float lastPitchRateDeg = 0.0f;
    float filteredPitchAlpha = 0.0f;
    bool pitchRateReady = false;
    float lastYawRateDeg = 0.0f;
    float filteredYawAlpha = 0.0f;
    bool yawRateReady = false;

    float moveToward(float current, float target, float maxDelta) {
        float delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }

    float calculateVirtualInertiaPwm(float pitchRateDeg, float dt) {
        if (dt <= 0.0f) return 0.0f;
        if (!pitchRateReady) {
            lastPitchRateDeg = pitchRateDeg;
            pitchRateReady = true;
            return 0.0f;
        }

        float rawAlpha = (pitchRateDeg - lastPitchRateDeg) / dt;
        lastPitchRateDeg = pitchRateDeg;
        rawAlpha = constrain(rawAlpha,
                             -Config::V_INERTIA_ALPHA_MAX_DPS2,
                             Config::V_INERTIA_ALPHA_MAX_DPS2);

        float alphaBlend = dt / (Config::V_INERTIA_ALPHA_TAU + dt);
        filteredPitchAlpha += alphaBlend * (rawAlpha - filteredPitchAlpha);

        float effectiveAlpha = filteredPitchAlpha;
        if (abs(effectiveAlpha) < Config::V_INERTIA_ALPHA_DEADZONE_DPS2) {
            effectiveAlpha = 0.0f;
        } else {
            effectiveAlpha = copysign(abs(effectiveAlpha) - Config::V_INERTIA_ALPHA_DEADZONE_DPS2,
                                      effectiveAlpha);
        }

        return constrain(-Config::V_INERTIA_PWM_SIGN *
                        Config::V_INERTIA_PWM_GAIN *
                         effectiveAlpha,
                        -Config::V_INERTIA_PWM_MAX,
                        Config::V_INERTIA_PWM_MAX);
    }

    float calculateYawInertiaPwm(float yawRateDeg, float dt) {
        if (dt <= 0.0f) return 0.0f;
        if (!yawRateReady) {
            lastYawRateDeg = yawRateDeg;
            yawRateReady = true;
            return 0.0f;
        }

        float rawAlpha = (yawRateDeg - lastYawRateDeg) / dt;
        lastYawRateDeg = yawRateDeg;
        rawAlpha = constrain(rawAlpha,
                             -Config::YAW_INERTIA_ALPHA_MAX_DPS2,
                             Config::YAW_INERTIA_ALPHA_MAX_DPS2);

        float alphaBlend = dt / (Config::YAW_INERTIA_ALPHA_TAU + dt);
        filteredYawAlpha += alphaBlend * (rawAlpha - filteredYawAlpha);

        float effectiveAlpha = filteredYawAlpha;
        if (abs(effectiveAlpha) < Config::YAW_INERTIA_ALPHA_DEADZONE_DPS2) {
            effectiveAlpha = 0.0f;
        } else {
            effectiveAlpha = copysign(abs(effectiveAlpha) - Config::YAW_INERTIA_ALPHA_DEADZONE_DPS2,
                                      effectiveAlpha);
        }

        return constrain(-Config::YAW_INERTIA_PWM_SIGN *
                        Config::YAW_INERTIA_PWM_GAIN *
                         effectiveAlpha,
                        -Config::YAW_INERTIA_PWM_MAX,
                        Config::YAW_INERTIA_PWM_MAX);
    }

    float updateGradePitch(float pitchAngleDeg, float dt) {
        if (!gradePitchReady) {
            gradePitchDeg = pitchAngleDeg;
            gradePitchReady = true;
            return gradePitchDeg;
        }

        float gradeBlend = dt / (Config::GRADE_PITCH_TAU + dt);
        gradePitchDeg += gradeBlend * (pitchAngleDeg - gradePitchDeg);
        return gradePitchDeg;
    }

public:
    TankChassis() : 
        rightTrack(DCMotor(Config::R_IN1, Config::R_IN2, Config::R_PWM, Config::PWM_CH_R, false),
                   CustomEncoder(Config::R_ENCA, Config::R_ENCB, PCNT_UNIT_0),
                   "right"),
        leftTrack (DCMotor(Config::L_IN1, Config::L_IN2, Config::L_PWM, Config::PWM_CH_L, true),
                   CustomEncoder(Config::L_ENCA, Config::L_ENCB, PCNT_UNIT_1),
                   "left"),
        engineSmoother(0.8f, 10.0f),  // 参数可调：0.8表示油门踩到底需1秒多建立全扭矩
        brakeSmoother(4.0f, 10.0f)    // 参数可调：刹车建立很快
    {}

    void init() { rightTrack.init(); leftTrack.init(); }

    // 这里的速度单位统一用“真车等效 km/h”，这样比例映射和参数调校更直观。
    void processKinematics(float triggerL, float triggerR, float joyX, float dt, float currentPitchRate, float currentYawRate, float pitchAngle) {
        float gradePitch = updateGradePitch(pitchAngle, dt);

        // ==========================================
        // 纵向动力学 (游戏式 RT 前进 / LT 倒车)
        // ==========================================
        
        // 1. 获取平滑后的油门与刹车输入 (0.0 ~ 1.0)
        // trigger 做了平方处理，模拟摇杆的指数曲线，增加微操手感
        float forwardInput = (triggerR > Config::TRIGGER_DEADZONE) ? (triggerR * triggerR) : 0.0f;
        float reverseInput = (triggerL > Config::TRIGGER_DEADZONE) ? (triggerL * triggerL) : 0.0f;
        float driveInput = forwardInput - reverseInput;
        float desiredDir = (driveInput > 0.001f) ? 1.0f : ((driveInput < -0.001f) ? -1.0f : 0.0f);

        float raw_throttle = abs(driveInput);
        float raw_brake = 0.0f;

        // 当请求方向与当前运动方向相反时，先把该输入当成刹车；接近停稳后再自动换向。
        bool brakingToReverse = (v_real > 0.3f && desiredDir < 0.0f) || (v_real < -0.3f && desiredDir > 0.0f);
        if (brakingToReverse) {
            raw_brake = raw_throttle;
            raw_throttle = 0.0f;
            engineSmoother.reset();
        }

        float eff_throttle = engineSmoother.update(raw_throttle, dt);
        float eff_brake    = brakeSmoother.update(raw_brake, dt);

        // 2. 计算各独立作用力（换算为加速度，单位 km/h/s）
        float force_engine = eff_throttle * Config::REAL_ACCEL * desiredDir;

        float force_brake = eff_brake * Config::REAL_BRAKE;

        // 阻力：始终与当前运动方向相反
        float resDir = (v_real > 0.1f) ? 1.0f : ((v_real < -0.1f) ? -1.0f : (v_real / 0.1f));
        float airResist = 0.001f * v_real * v_real;
        float rollResist = 0.8f * constrain(abs(v_real)/1.0f, 0.0f, 1.0f); // 滚动阻力
        float force_resist = -(airResist + rollResist) * resDir;

        // 坡度重力分量
        float force_slope = -Config::SLOPE_GRAVITY_MAX * sin(gradePitch * DEG_TO_RAD);

        // 3. 施加刹车力的方向判定
        // 刹车力是没有主动方向的，它只能去“抵消”当前的速度。
        if (v_real > 0.1f) {
            force_brake = -force_brake; // 车往前走，刹车向后拉
        } else if (v_real < -0.1f) {
            force_brake = force_brake;  // 车往后走，刹车向前拉
        } else {
            // 速度极小时，重力可能导致溜车。如果刹车踩得够死，静摩擦力接管，抵消所有外力。
            if (eff_brake > 0.1f && abs(force_engine + force_slope) < (force_brake)) {
                force_engine = 0; force_slope = 0; force_resist = 0; force_brake = 0; 
                v_real = 0; // 死死刹停
            } else {
                force_brake = 0;
            }
        }

        // 4. 净力求和
        float a_net = force_engine + force_brake + force_resist + force_slope;

        // 5. jerk 限制后的实际纵向加速度。持续加速时车身持续抬头/低头，而不是只在起步瞬间响应。
        bool accelerationBuilds = (a_net * longitudinalAccel >= 0.0f) && (abs(a_net) > abs(longitudinalAccel));
        float jerkLimit = accelerationBuilds ? Config::LINEAR_JERK_ACCEL : Config::LINEAR_JERK_BRAKE;
        longitudinalAccel = moveToward(longitudinalAccel, a_net, jerkLimit * dt);
        if (abs(a_net) < 0.02f && abs(longitudinalAccel) < 0.02f) longitudinalAccel = 0.0f;

        // 6. 积分计算最终纵向速度
        v_real += longitudinalAccel * dt;

        // 极限速度钳制
        v_real = constrain(v_real, -Config::REAL_V_REV_MAX, Config::REAL_V_MAX);
        if ((v_real >= Config::REAL_V_MAX && longitudinalAccel > 0.0f) ||
            (v_real <= -Config::REAL_V_REV_MAX && longitudinalAccel < 0.0f)) {
            longitudinalAccel = 0.0f;
        }

        // ==========================================
        // 横向动力学：摇杆直接给目标差速，真实车体的转动惯量由 yaw IMU 反馈补偿。
        // ==========================================
        
        float joyX_adj = (abs(joyX) < 0.12f) ? 0 : joyX; // 死区
        float joyX_squared = copysign(joyX_adj * joyX_adj, joyX_adj); 
        
        // 随速感应灵敏度
        float dynamic_sens = Config::YAW_SENSITIVITY / (1.0f + abs(v_real) * Config::SPEED_SENS_K);
        
        // 目标自转速度
        float target_spin_v = joyX_squared * dynamic_sens;
        // 低速/原地转向更像另一组“转向油门”：先克服履带搓地阻力，反向输入先刹到接近 0。
        // 行进中则更接近双流传动的速度分配，允许更快跟随目标差速。
        float movingBlend = constrain(abs(v_real) / Config::TURN_MOVING_BLEND_KMH, 0.0f, 1.0f);
        float pivotBlend = 1.0f - movingBlend;
        bool turnInputActive = abs(joyX_adj) >= 0.12f;
        bool pivotReverseBrake = turnInputActive &&
                                 (target_spin_v * spinV < 0.0f) &&
                                 (abs(spinV) > Config::TRACK_STOP_DEADZONE_KMH);

        float pivotTargetSpin = pivotReverseBrake ? 0.0f : target_spin_v;
        float effectiveTargetSpin = pivotTargetSpin * pivotBlend + target_spin_v * movingBlend;

        float turnAccelLimit = Config::TURN_ACCEL_PIVOT +
                               (Config::TURN_ACCEL_MOVING - Config::TURN_ACCEL_PIVOT) * movingBlend;
        float turnBrakeLimit = Config::TURN_BRAKE_PIVOT +
                               (Config::TURN_BRAKE_MOVING - Config::TURN_BRAKE_PIVOT) * movingBlend;
        bool turnBuilds = (effectiveTargetSpin * spinV >= 0.0f) &&
                          (abs(effectiveTargetSpin) > abs(spinV)) &&
                          !pivotReverseBrake;
        float turnRateLimit = turnBuilds ? turnAccelLimit : turnBrakeLimit;
        spinV = moveToward(spinV, effectiveTargetSpin, turnRateLimit * dt);

        // ==========================================
        // 双流耦合输出 (保持不变)
        // ==========================================
        float Lv_tgt = v_real + spinV;
        float Rv_tgt = v_real - spinV;
        float pitchInertiaPwm = calculateVirtualInertiaPwm(currentPitchRate, dt);
        float yawInertiaPwm = calculateYawInertiaPwm(currentYawRate, dt);

        float max_val = max(abs(Lv_tgt), abs(Rv_tgt));
        if (max_val > Config::REAL_V_MAX) {
            float ratio = Config::REAL_V_MAX / max_val;
            Lv_tgt *= ratio; Rv_tgt *= ratio;
        }

        leftTrack.update(Lv_tgt, dt, pitchInertiaPwm + yawInertiaPwm);
        rightTrack.update(Rv_tgt, dt, pitchInertiaPwm - yawInertiaPwm);
    }

    void processDirectTrackTargets(float leftTarget, float rightTarget, float dt, float pitchAngle) {
        updateGradePitch(pitchAngle, dt);
        v_real = (leftTarget + rightTarget) * 0.5f;
        spinV = (leftTarget - rightTarget) * 0.5f;
        longitudinalAccel = 0.0f;
        leftTrack.update(leftTarget, dt, 0.0f);
        rightTrack.update(rightTarget, dt, 0.0f);
    }

    void getTrackTelemetry(float& leftTarget, float& leftActual, float& leftPwm, bool& leftStalled,
                           float& rightTarget, float& rightActual, float& rightPwm, bool& rightStalled) const {
        leftTarget = leftTrack.targetSpeed;
        leftActual = leftTrack.currentSpeed;
        leftPwm = leftTrack.lastPwm;
        leftStalled = leftTrack.stallLatched;
        rightTarget = rightTrack.targetSpeed;
        rightActual = rightTrack.currentSpeed;
        rightPwm = rightTrack.lastPwm;
        rightStalled = rightTrack.stallLatched;
    }

    void stop() { 
        v_real = 0; spinV = 0; 
        longitudinalAccel = 0.0f;
        gradePitchDeg = 0.0f;
        gradePitchReady = false;
        lastPitchRateDeg = 0.0f;
        filteredPitchAlpha = 0.0f;
        pitchRateReady = false;
        lastYawRateDeg = 0.0f;
        filteredYawAlpha = 0.0f;
        yawRateReady = false;
        engineSmoother.reset(); brakeSmoother.reset();
        leftTrack.stop(); rightTrack.stop(); 
    }
};


