#include "TankChassis.h"

void TankTrack::clearStallIfReleased() {
        if (abs(targetSpeed) <= Config::TRACK_STALL_CLEAR_TARGET_KMH) {
            stallLatched = false;
            stallCandidateSinceMs = 0;
        }
    }

// 堵转保护不是看到一次低速就触发，而是要求：
// 目标速度足够高、PWM 足够大、实际速度仍然很低，并且持续超过 grace 时间。
// 这样可以避开正常起步瞬间“速度还没起来”的误判。
bool TankTrack::updateStallProtection() {
        float targetDirection = targetSpeed >= 0.0f ? 1.0f : -1.0f;
        float speedInTargetDirection = currentSpeed * targetDirection;
        if (abs(targetSpeed) < Config::TRACK_STALL_TARGET_MIN_KMH ||
            abs(lastPwm) < Config::TRACK_STALL_PWM_MIN ||
            speedInTargetDirection > Config::TRACK_STALL_ACTUAL_MAX_KMH) {
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

TankTrack::TankTrack(DCMotor m, CustomEncoder e, const char* trackLabel, bool leftTrack)
    : motor(m), encoder(e), controller(leftTrack), label(trackLabel) {}

void TankTrack::init() { motor.init(); encoder.init(); }

void TankTrack::update(float target, float dt, float externalPwm) {
        if (!isfinite(target) || !isfinite(dt) || dt <= 0.0f || !isfinite(externalPwm)) {
            targetSpeed = 0.0f;
            currentSpeed = 0.0f;
            telemetrySpeed = 0.0f;
            lastPwm = 0.0f;
            controller.reset();
            motor.drive(0.0f);
            return;
        }
        targetSpeed = target;
        currentSpeed = encoder.getRealSpeedKMH();
        telemetrySpeed = encoder.getDisplaySpeedKMH();
        if (!isfinite(currentSpeed) || !isfinite(telemetrySpeed)) {
            targetSpeed = 0.0f;
            currentSpeed = 0.0f;
            telemetrySpeed = 0.0f;
            lastPwm = 0.0f;
            controller.reset();
            motor.drive(0.0f);
            return;
        }
        uint32_t encoderSampleId = encoder.getControlSampleId();
        bool newSpeedSample = encoderSampleId != lastEncoderControlSampleId;
        if (newSpeedSample) lastEncoderControlSampleId = encoderSampleId;
        if (abs(targetSpeed) < Config::TRACK_STOP_DEADZONE_KMH) targetSpeed = 0;
        clearStallIfReleased();

        if (stallLatched) {
            lastPwm = 0;
            motor.drive(0);
            return;
        }

        lastPwm = controller.calculate(targetSpeed, currentSpeed, dt, externalPwm,
                                       newSpeedSample, encoder.getControlSampleDt());
        if (updateStallProtection()) {
            lastPwm = 0;
            motor.drive(0);
            return;
        }
        motor.drive(lastPwm);
    }

void TankTrack::driveDirect(float pwm) {
        targetSpeed = 0.0f;
        currentSpeed = encoder.getRealSpeedKMH();
        telemetrySpeed = encoder.getDisplaySpeedKMH();
        lastEncoderControlSampleId = encoder.getControlSampleId();
        stallLatched = false;
        stallCandidateSinceMs = 0;
        controller.reset();
        lastPwm = constrain(pwm, -Config::DEBUG_DIRECT_PWM_MAX, Config::DEBUG_DIRECT_PWM_MAX);
        motor.drive(lastPwm);
    }

void TankTrack::stop() {
        targetSpeed = 0;
        lastPwm = 0;
        stallLatched = false;
        stallCandidateSinceMs = 0;
        controller.reset();
        motor.drive(0);
    }

ThrottleSmoother::ThrottleSmoother(float rise, float fall) : rise_rate(rise), fall_rate(fall) {}

float ThrottleSmoother::update(float target, float dt) {
        if (target > current_val) {
            // 踩油门：缓慢建立扭矩
            current_val = min(current_val + rise_rate * dt, target);
        } else {
            // 松油门/刹车：极其迅速地卸载扭矩
            current_val = max(current_val - fall_rate * dt, target);
        }
        return current_val;
    }

void ThrottleSmoother::reset() { current_val = 0.0f; }

float TankChassis::moveToward(float current, float target, float maxDelta) {
        float delta = target - current;
        if (delta > maxDelta) return current + maxDelta;
        if (delta < -maxDelta) return current - maxDelta;
        return target;
    }

// pitch 虚拟惯量：从底盘 pitch 角速度估算角加速度，再换算成前后同向 PWM。
// 目的不是精确物理建模，而是让车体点头/抬头时电机有一个反向补偿，手感更像有惯量的重车。
float TankChassis::calculateVirtualInertiaPwm(float pitchRateDeg, float dt) {
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

// yaw 虚拟惯量：从底盘 yaw 角速度估算角加速度，再换算成左右差速 PWM。
// 它主要用于抑制突然转向时的转动冲击，让履带输出更像有大质量车体在后面拖着。
float TankChassis::calculateYawInertiaPwm(float yawRateDeg, float dt) {
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

// 坡度角用于估算重力沿车体前后方向的分量。
// 这里做低通，是为了避免 IMU 噪声直接变成底盘油门抖动。
float TankChassis::updateGradePitch(float pitchAngleDeg, float dt) {
        if (!gradePitchReady) {
            gradePitchDeg = pitchAngleDeg;
            gradePitchReady = true;
            return gradePitchDeg;
        }

        float gradeBlend = dt / (Config::GRADE_PITCH_TAU + dt);
        gradePitchDeg += gradeBlend * (pitchAngleDeg - gradePitchDeg);
        return gradePitchDeg;
    }

TankChassis::TankChassis() : 
        rightTrack(DCMotor(Config::R_IN1, Config::R_IN2, Config::R_PWM, Config::PWM_CH_R, false),
                   CustomEncoder(Config::R_ENCA, Config::R_ENCB, PCNT_UNIT_0, Config::R_ENCODER_SIGN),
                   "right", false),
        leftTrack (DCMotor(Config::L_IN1, Config::L_IN2, Config::L_PWM, Config::PWM_CH_L, true),
                   CustomEncoder(Config::L_ENCA, Config::L_ENCB, PCNT_UNIT_1, Config::L_ENCODER_SIGN),
                   "left", true),
        engineSmoother(0.8f, 10.0f),
        brakeSmoother(4.0f, 10.0f)
{}

void TankChassis::init() { rightTrack.init(); leftTrack.init(); }

void TankChassis::resetImuCompensation() {
        gradePitchDeg = 0.0f;
        gradePitchReady = false;
        lastPitchRateDeg = 0.0f;
        filteredPitchAlpha = 0.0f;
        pitchRateReady = false;
        lastYawRateDeg = 0.0f;
        filteredYawAlpha = 0.0f;
        yawRateReady = false;
    }

// 主底盘动力学：先算纵向速度，再算转向差速，最后分配成左右履带目标。
// 这里的速度单位统一用“真车等效 km/h”，这样比例映射和参数调校更直观。
void TankChassis::processKinematics(float triggerL, float triggerR, float joyX, float dt, float currentPitchRate, float currentYawRate, float pitchAngle) {
        if (!isfinite(triggerL) || !isfinite(triggerR) || !isfinite(joyX) ||
            !isfinite(dt) || dt <= 0.0f) {
            stop();
            return;
        }
        // 姿态补偿是附加功能；任何一项异常时只禁用补偿，不牺牲基本行走能力。
        if (!isfinite(currentPitchRate)) currentPitchRate = 0.0f;
        if (!isfinite(currentYawRate)) currentYawRate = 0.0f;
        if (!isfinite(pitchAngle)) pitchAngle = 0.0f;

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

        // 阻力：运动时始终与当前运动方向相反。
        float resDir = (v_real > 0.1f) ? 1.0f : ((v_real < -0.1f) ? -1.0f : (v_real / 0.1f));
        float airResist = Config::AIR_RESIST_COEFF * v_real * v_real;
        float rollResist = Config::ROLL_RESIST_ACCEL * constrain(abs(v_real) / 1.0f, 0.0f, 1.0f);
        float force_resist = -(airResist + rollResist) * resDir;

        // 坡度重力分量
        float force_slope = -Config::SLOPE_GRAVITY_MAX * sin(gradePitch * DEG_TO_RAD);

        // 松开油门且近乎静止时，用有限的静阻力抵消小坡重力；超过阈值仍会按剩余坡力溜车。
        // 该分支不伪造电机 PWM，只改变动力学目标，因此坡上溜车仍由速度闭环真实执行。
        bool neutralCoast = raw_throttle <= 0.001f && raw_brake <= 0.001f;
        if (neutralCoast && abs(v_real) <= 0.1f) {
            if (abs(force_slope) <= Config::COAST_STATIC_RESIST_ACCEL) {
                force_slope = 0.0f;
                force_resist = 0.0f;
                longitudinalAccel = 0.0f;
                v_real = 0.0f;
            } else {
                // 刚开始溜车时速度还没有方向，用坡力方向决定静阻力方向。
                force_resist = -copysign(Config::COAST_STATIC_RESIST_ACCEL, force_slope);
            }
        }

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

        // 5. jerk 限制后的实际纵向加速度。明确刹车时必须始终使用较快的制动 jerk；
        // 否则减速度一旦与目标净加速度同号，就会被误判成普通加速建立并切回慢 jerk。
        bool accelerationBuilds = (a_net * longitudinalAccel >= 0.0f) && (abs(a_net) > abs(longitudinalAccel));
        bool serviceBrakeActive = brakingToReverse || eff_brake > 0.01f;
        float jerkLimit = serviceBrakeActive
            ? Config::LINEAR_JERK_BRAKE
            : (accelerationBuilds ? Config::LINEAR_JERK_ACCEL : Config::LINEAR_JERK_BRAKE);
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
        
        // 行进转向和原地中心转向使用不同的履带差速上限。若原地也直接沿用
        // YAW_SENSITIVITY=25 km/h，理论一圈只有约 4.7 秒，明显失去重车感。
        float movingTargetSpin = joyX_squared * dynamic_sens;
        float pivotCommandSpin = joyX_squared * Config::PIVOT_SPIN_MAX_KMH;
        // 低速/原地转向更像另一组“转向油门”：先克服履带搓地阻力，反向输入先刹到接近 0。
        // 行进中则更接近双流传动的速度分配，允许更快跟随目标差速。
        float movingBlend = constrain(abs(v_real) / Config::TURN_MOVING_BLEND_KMH, 0.0f, 1.0f);
        float pivotBlend = 1.0f - movingBlend;
        bool turnInputActive = abs(joyX_adj) >= 0.12f;
        bool pivotReverseBrake = turnInputActive &&
                                 (pivotCommandSpin * spinV < 0.0f) &&
                                 (abs(spinV) > Config::TRACK_STOP_DEADZONE_KMH);

        float pivotTargetSpin = pivotReverseBrake ? 0.0f : pivotCommandSpin;
        float effectiveTargetSpin = pivotTargetSpin * pivotBlend + movingTargetSpin * movingBlend;

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

        // 虚拟惯量补偿最后叠到履带 PWM 上，不改变目标速度本身。
        float pitchInertiaPwm = calculateVirtualInertiaPwm(currentPitchRate, dt);
        float yawInertiaPwm = calculateYawInertiaPwm(currentYawRate, dt);

        float max_val = max(abs(Lv_tgt), abs(Rv_tgt));
        if (max_val > Config::REAL_V_MAX) {
            float ratio = Config::REAL_V_MAX / max_val;
            Lv_tgt *= ratio; Rv_tgt *= ratio;
        }

        if (!isfinite(Lv_tgt) || !isfinite(Rv_tgt) ||
            !isfinite(pitchInertiaPwm) || !isfinite(yawInertiaPwm)) {
            stop();
            return;
        }

        // 某侧目标为零时绝不允许 IMU 敲击/晃动单独驱动该履带；坡度若真的让车辆
        // 开始溜动，会先生成非零动力学目标，再自然恢复所需的运动补偿。
        float leftInertiaPwm = abs(Lv_tgt) >= Config::TRACK_STOP_DEADZONE_KMH
            ? pitchInertiaPwm + yawInertiaPwm
            : 0.0f;
        float rightInertiaPwm = abs(Rv_tgt) >= Config::TRACK_STOP_DEADZONE_KMH
            ? pitchInertiaPwm - yawInertiaPwm
            : 0.0f;

        leftTrack.update(Lv_tgt, dt, leftInertiaPwm);
        rightTrack.update(Rv_tgt, dt, rightInertiaPwm);
    }

// 直接履带目标主要给测试/调试使用：绕过手柄油门、刹车、转向动力学。
void TankChassis::processDirectTrackTargets(float leftTarget, float rightTarget, float dt, float pitchAngle) {
        updateGradePitch(pitchAngle, dt);
        v_real = (leftTarget + rightTarget) * 0.5f;
        spinV = (leftTarget - rightTarget) * 0.5f;
        longitudinalAccel = 0.0f;
        leftTrack.update(leftTarget, dt, 0.0f);
        rightTrack.update(rightTarget, dt, 0.0f);
}

void TankChassis::setTrackPidGains(float kp, float ki, float kd) {
        TrackVelocityController::setRuntimePidGains(kp, ki, kd);
}

void TankChassis::resetTrackPidGains() {
        TrackVelocityController::resetRuntimePidGains();
}

void TankChassis::getTrackPidGains(float& kp, float& ki, float& kd) const {
        TrackVelocityController::getRuntimePidGains(kp, ki, kd);
}

void TankChassis::processDirectTrackPwm(float leftPwm, float rightPwm) {
        v_real = 0.0f;
        spinV = 0.0f;
        longitudinalAccel = 0.0f;
        leftTrack.driveDirect(leftPwm);
        rightTrack.driveDirect(rightPwm);
    }

// 输出 telemetry 时不直接暴露 TankTrack 对象，避免上层误改底盘内部状态。
void TankChassis::getTrackTelemetry(float& leftTarget, float& leftControlActual, float& leftDisplayActual,
                           float& leftPwm, bool& leftStalled,
                           float& rightTarget, float& rightControlActual, float& rightDisplayActual,
                           float& rightPwm, bool& rightStalled) const {
        leftTarget = leftTrack.targetSpeed;
        leftControlActual = leftTrack.currentSpeed;
        leftDisplayActual = leftTrack.telemetrySpeed;
        leftPwm = leftTrack.lastPwm;
        leftStalled = leftTrack.stallLatched;
        rightTarget = rightTrack.targetSpeed;
        rightControlActual = rightTrack.currentSpeed;
        rightDisplayActual = rightTrack.telemetrySpeed;
        rightPwm = rightTrack.lastPwm;
        rightStalled = rightTrack.stallLatched;
}

// stop() 不只是把 PWM 归零，还清掉所有滤波/积分/惯量历史。
// 这样断连或低压恢复后，不会带着旧状态突然输出一脚力。
void TankChassis::stop() { 
        v_real = 0; spinV = 0; 
        longitudinalAccel = 0.0f;
        resetImuCompensation();
        engineSmoother.reset(); brakeSmoother.reset();
        leftTrack.stop(); rightTrack.stop(); 
    }
