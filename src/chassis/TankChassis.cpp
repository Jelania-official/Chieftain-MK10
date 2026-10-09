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
        float targetDirection = controlTargetSpeed >= 0.0f ? 1.0f : -1.0f;
        float pwmInTargetDirection = lastPwm * targetDirection;
        if (abs(controlTargetSpeed) < Config::TRACK_STALL_TARGET_MIN_KMH ||
            pwmInTargetDirection < Config::TRACK_STALL_PWM_MIN ||
            abs(fastSpeed) > Config::TRACK_STALL_ACTUAL_MAX_KMH) {
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
                   label, controlTargetSpeed, fastSpeed, lastPwm);
        return true;
    }

TankTrack::TankTrack(DCMotor m, CustomEncoder e, const char* trackLabel, bool leftTrack)
    : motor(m), encoder(e), controller(leftTrack), label(trackLabel) {}

void TankTrack::init() { motor.init(); encoder.init(); }

void TankTrack::update(float target, float dt, float externalPwm, float batteryVoltage,
                       bool brakingActive, bool motionDemandActive, bool pivotLaunch) {
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
        fastSpeed = encoder.getRealSpeedKMH();
        telemetrySpeed = encoder.getDisplaySpeedKMH();
        if (!isfinite(fastSpeed) || !isfinite(telemetrySpeed)) {
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

        lastPwm = controller.calculate(targetSpeed, fastSpeed, dt, externalPwm, batteryVoltage,
                                       brakingActive, motionDemandActive, pivotLaunch, newSpeedSample,
                                       encoder.getControlSampleDt());
        // 遥测保留底盘动力学给出的原始目标；最低可执行速度只存在于控制器内部。
        controlTargetSpeed = controller.getEffectiveTarget();
        currentSpeed = controller.getFeedbackActual();
        if (updateStallProtection()) {
            lastPwm = 0;
            motor.drive(0);
            return;
        }
        motor.drive(lastPwm);
    }

void TankTrack::driveDirect(float pwm) {
        targetSpeed = 0.0f;
        controlTargetSpeed = 0.0f;
        fastSpeed = encoder.getRealSpeedKMH();
        currentSpeed = fastSpeed;
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
        controlTargetSpeed = 0;
        lastPwm = 0;
        stallLatched = false;
        stallCandidateSinceMs = 0;
        controller.reset();
        motor.drive(0);
    }

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
                   "left", true)
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
void TankChassis::processKinematics(float triggerL, float triggerR, float joyX, float dt,
                                    float currentPitchRate, float currentYawRate, float pitchAngle,
                                    float batteryVoltage) {
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
        
        // 1. 获取油门与刹车输入 (0.0 ~ 1.0)。trigger做平方处理以增加微操手感；
        // 发动机响应在后面的物理状态中建立，刹车则直接使用当前扳机值。
        float forwardInput = (triggerR > Config::TRIGGER_DEADZONE) ? (triggerR * triggerR) : 0.0f;
        float reverseInput = (triggerL > Config::TRIGGER_DEADZONE) ? (triggerL * triggerL) : 0.0f;
        float driveInput = forwardInput - reverseInput;
        int8_t requestedDir = (driveInput > 0.001f) ? 1 : ((driveInput < -0.001f) ? -1 : 0);

        // 静止起步意图必须连续保持500ms才生效。确认期间不推进虚拟速度，也不触发
        // 单侧履带起步状态机；行驶中的油门、制动和原地转向不受这个确认影响。
        bool nearlyStationaryForStart = abs(v_real) <= Config::DIRECTION_CHANGE_STOP_SPEED_KMH;
        bool oppositeDirectionRequest = requestedDir != 0 && driveDirection != 0 &&
                                        requestedDir != driveDirection;
        bool startConfirmationRequired = requestedDir != 0 && nearlyStationaryForStart &&
                                         !oppositeDirectionRequest;
        if (requestedDir == 0) {
            startIntentDirection = 0;
            startIntentSinceMs = 0;
            startIntentTiming = false;
            startIntentConfirmed = false;
        } else if (startConfirmationRequired &&
                   (!startIntentConfirmed || startIntentDirection != requestedDir)) {
            uint32_t nowMs = millis();
            if (!startIntentTiming || startIntentDirection != requestedDir) {
                startIntentDirection = requestedDir;
                startIntentSinceMs = nowMs;
                startIntentTiming = true;
                startIntentConfirmed = false;
            } else if ((uint32_t)(nowMs - startIntentSinceMs) >=
                       Config::TRACK_START_INPUT_CONFIRM_MS) {
                startIntentTiming = false;
                startIntentConfirmed = true;
            }
        } else if (!startConfirmationRequired && !oppositeDirectionRequest) {
            startIntentDirection = requestedDir;
            startIntentTiming = false;
            startIntentConfirmed = true;
        }
        bool startInputBlocked = startConfirmationRequired && !startIntentConfirmed;

        if (driveDirection == 0) {
            if (v_real > Config::DIRECTION_CHANGE_STOP_SPEED_KMH) driveDirection = 1;
            else if (v_real < -Config::DIRECTION_CHANGE_STOP_SPEED_KMH) driveDirection = -1;
            else if (requestedDir != 0 && !startInputBlocked) driveDirection = requestedDir;
        }

        float raw_throttle = abs(driveInput);
        float raw_brake = 0.0f;
        if (startInputBlocked) raw_throttle = 0.0f;

        // 刹车优先：当前前进时LT是刹车，当前倒车时RT是刹车。超过既有死区后
        // 立即切断发动机牵引；制动力仍按扳机平方值连续变化。
        bool pedalBrakeActive = (driveDirection > 0 && reverseInput > 0.0f) ||
                                (driveDirection < 0 && forwardInput > 0.0f);
        if (pedalBrakeActive) {
            raw_brake = driveDirection > 0 ? reverseInput : forwardInput;
            raw_throttle = 0.0f;
            engineCmd = 0.0f;
        }

        // 相反扳机先只充当刹车。车辆接近停稳后继续保持设定时间，才授权新方向。
        bool directionChangeBraking = requestedDir != 0 && driveDirection != 0 &&
                                      requestedDir != driveDirection;
        if (directionChangeBraking) {
            uint32_t nowMs = millis();
            raw_brake = max(raw_brake, abs(driveInput));
            raw_throttle = 0.0f;
            engineCmd = 0.0f;
            bool nearlyStopped = abs(v_real) <= Config::DIRECTION_CHANGE_STOP_SPEED_KMH;
            // 反向输入在车辆仍运动时只负责刹车；换向确认从接近静止后才开始计时。
            if (!nearlyStopped) {
                pendingDriveDirection = requestedDir;
                directionChangeSinceMs = 0;
            } else if (pendingDriveDirection != requestedDir || directionChangeSinceMs == 0) {
                pendingDriveDirection = requestedDir;
                directionChangeSinceMs = nowMs;
            }
            bool heldLongEnough = nearlyStopped && directionChangeSinceMs != 0 &&
                (uint32_t)(nowMs - directionChangeSinceMs) >= Config::DIRECTION_CHANGE_HOLD_MS;
            if (heldLongEnough && nearlyStopped) {
                driveDirection = requestedDir;
                pendingDriveDirection = 0;
                directionChangeSinceMs = 0;
                directionChangeBraking = false;
                pedalBrakeActive = false;
                raw_brake = 0.0f;
                raw_throttle = abs(driveInput);
            }
        } else {
            pendingDriveDirection = 0;
            directionChangeSinceMs = 0;
        }

        float commandedDriveInput = directionChangeBraking ? 0.0f
            : raw_throttle * (float)driveDirection;

        // 方向输入保留现有死区和手感曲线，再加入很短的TN12转向响应。
        constexpr float turnDeadzone = 0.12f;
        float turnMagnitude = (abs(joyX) <= turnDeadzone) ? 0.0f
            : (abs(joyX) - turnDeadzone) / (1.0f - turnDeadzone);
        float shapedMagnitude = Config::TURN_INPUT_LINEAR_BLEND * turnMagnitude +
            (1.0f - Config::TURN_INPUT_LINEAR_BLEND) * turnMagnitude * turnMagnitude;
        float steerCommand = copysign(shapedMagnitude, joyX);
        steerState += (dt / (Config::STEER_FILTER_TAU_S + dt)) * (steerCommand - steerState);
        if (abs(steerState) < 0.001f) steerState = 0.0f;

        // 刹车优先时立即卸载动力；正常驾驶时以0.6s一阶状态模拟发动机/TN12。
        if (directionChangeBraking || pedalBrakeActive) engineCmd = 0.0f;
        else engineCmd += (raw_throttle - engineCmd) * dt / Config::ENGINE_TAU_S;
        engineCmd = constrain(engineCmd, 0.0f, 1.0f);
        float eff_brake = raw_brake;
        bool serviceBrakeActive = directionChangeBraking || pedalBrakeActive || eff_brake > 0.01f;

        // 最大牵引力/恒功率模型及路面阻力全部使用SI制。
        float speedMps = v_real / 3.6f;
        float absSpeedMps = abs(speedMps);
        float availableTraction = min(Config::TRACTION_MAX_N,
            Config::TRACK_POWER_W / max(absSpeedMps, 1.0f));
        float forceEngine = engineCmd * availableTraction * (float)driveDirection;
        float rollingCoeff = Config::DRIVE_ROAD_PROFILE == Config::RoadProfile::Unpaved
            ? Config::UNPAVED_ROLLING_COEFF : Config::ROAD_ROLLING_COEFF;
        float terrainK = Config::DRIVE_ROAD_PROFILE == Config::RoadProfile::Unpaved
            ? Config::UNPAVED_TERRAIN_K_N_PER_MPS : Config::ROAD_TERRAIN_K_N_PER_MPS;
        float cda = Config::DRIVE_ROAD_PROFILE == Config::RoadProfile::Unpaved
            ? Config::UNPAVED_CDA_M2 : Config::ROAD_CDA_M2;
        float forceRolling = rollingCoeff * Config::VEHICLE_MASS_KG * 9.81f;
        float forceTerrain = terrainK * absSpeedMps;
        float forceAir = 0.5f * Config::AIR_DENSITY_KG_M3 * cda * absSpeedMps * absSpeedMps;
        float forceTurn = Config::TURN_RESIST_K_N_PER_MPS *
            pow(abs(steerState), Config::TURN_RESIST_EXP) * absSpeedMps;
        float forceSlope = -Config::VEHICLE_MASS_KG *
            (Config::SLOPE_GRAVITY_MAX / 3.6f) * sin(gradePitch * DEG_TO_RAD);
        float forceBrakeMagnitude = Config::BRAKE_MAX_N * eff_brake;

        float motionDirection = speedMps > 0.03f ? 1.0f : (speedMps < -0.03f ? -1.0f : 0.0f);
        float forceBeforeResistance = forceEngine + forceSlope;
        if (motionDirection == 0.0f && abs(forceBeforeResistance) > 1.0f)
            motionDirection = copysign(1.0f, forceBeforeResistance);
        float forceResistance = -motionDirection *
            (forceRolling + forceTerrain + forceAir + forceTurn);
        float forceBrake = -motionDirection * forceBrakeMagnitude;

        bool staticHold = abs(speedMps) < (0.05f / 3.6f) &&
            abs(forceBeforeResistance) <= forceRolling + forceBrakeMagnitude;
        if (staticHold) {
            v_real = 0.0f;
            longitudinalAccel = 0.0f;
        } else {
            float forceNet = forceBeforeResistance + forceResistance + forceBrake;
            float rawAccel = forceNet / Config::VEHICLE_MASS_KG;
            bool acceleratesExistingMotion = abs(speedMps) > 0.01f
                ? rawAccel * speedMps > 0.0f
                : rawAccel * (float)driveDirection > 0.0f;
            float accelLimit = acceleratesExistingMotion
                ? Config::MAX_DRIVE_ACCEL_MPS2
                : Config::MAX_BRAKE_ACCEL_MPS2;
            longitudinalAccel = constrain(rawAccel, -accelLimit, accelLimit);
            float previousSpeedMps = speedMps;
            speedMps += longitudinalAccel * dt;
            if (previousSpeedMps * speedMps < 0.0f &&
                abs(forceBeforeResistance) <= forceRolling + forceBrakeMagnitude) {
                speedMps = 0.0f;
                longitudinalAccel = 0.0f;
            }
            v_real = speedMps * 3.6f;
        }

        // 极限速度钳制
        v_real = constrain(v_real, -Config::REAL_V_REV_MAX, Config::REAL_V_MAX);
        if ((v_real >= Config::REAL_V_MAX && longitudinalAccel > 0.0f) ||
            (v_real <= -Config::REAL_V_REV_MAX && longitudinalAccel < 0.0f)) {
            longitudinalAccel = 0.0f;
        }

        // 只有无纵向输入且接近静止时才进入中心转向；油门+方向按DRIVE弧线起步。
        bool longitudinalDemand = requestedDir != 0;
        if (!pivotMode) {
            if (!longitudinalDemand && abs(v_real) < Config::PIVOT_ENTER_SPEED_KMH &&
                abs(steerState) > Config::PIVOT_ENTER_STEER) pivotMode = true;
        } else if (longitudinalDemand || abs(v_real) > Config::PIVOT_EXIT_SPEED_KMH ||
                   abs(steerState) < Config::PIVOT_EXIT_STEER) {
            pivotMode = false;
        }

        float Lv_tgt = 0.0f;
        float Rv_tgt = 0.0f;
        if (pivotMode) {
            // 正方向输入对应右转：左履带前进、右履带后退。
            float pivotCommandSpin = steerState * Config::PIVOT_SPIN_MAX_KMH;
            bool reverseBrake = pivotCommandSpin * spinV < 0.0f &&
                                abs(spinV) > Config::TRACK_STOP_DEADZONE_KMH;
            float pivotTargetSpin = reverseBrake ? 0.0f : pivotCommandSpin;
            bool builds = pivotTargetSpin * spinV >= 0.0f &&
                          abs(pivotTargetSpin) > abs(spinV) && !reverseBrake;
            spinV = moveToward(spinV, pivotTargetSpin,
                (builds ? Config::TURN_ACCEL_PIVOT : Config::TURN_BRAKE_PIVOT) * dt);
            Lv_tgt = spinV;
            Rv_tgt = -spinV;
        } else {
            // q(v)随速降低差速比例；q<1天然禁止行进中的内侧履带反转。
            float q = Config::STEER_Q0 /
                (1.0f + abs(v_real / 3.6f) / Config::STEER_VS_MPS);
            Lv_tgt = v_real * (1.0f + q * steerState);
            Rv_tgt = v_real * (1.0f - q * steerState);
            spinV = 0.5f * (Lv_tgt - Rv_tgt);
        }

        // 重车动力学可以让目标速度缓慢建立，但电机克服静摩擦的扭矩不应因此延迟数秒。
        // 用原始驾驶意图只提供方向性的小种子目标，低速状态机会立即开始120ms起步PWM斜坡；
        // 真正闭环目标和后续加速仍由 v_real/spinV 的重车动力学决定。
        float leftDriveIntent = pivotMode ? steerState : commandedDriveInput;
        float rightDriveIntent = pivotMode ? -steerState : commandedDriveInput;
        if (abs(Lv_tgt) < Config::TRACK_STOP_DEADZONE_KMH &&
            abs(leftDriveIntent) > 0.01f) {
            Lv_tgt = copysign(Config::TRACK_STOP_DEADZONE_KMH, leftDriveIntent);
        }
        if (abs(Rv_tgt) < Config::TRACK_STOP_DEADZONE_KMH &&
            abs(rightDriveIntent) > 0.01f) {
            Rv_tgt = copysign(Config::TRACK_STOP_DEADZONE_KMH, rightDriveIntent);
        }

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

        bool leftMotionDemand = abs(leftDriveIntent) > 0.01f;
        bool rightMotionDemand = abs(rightDriveIntent) > 0.01f;
        bool pivotLaunch = pivotMode;
        leftTrack.update(Lv_tgt, dt, leftInertiaPwm, batteryVoltage, serviceBrakeActive,
                         leftMotionDemand, pivotLaunch);
        rightTrack.update(Rv_tgt, dt, rightInertiaPwm, batteryVoltage, serviceBrakeActive,
                          rightMotionDemand, pivotLaunch);

        bool bothTracksClosedLoop = leftTrack.isVelocityClosedLoop() &&
                                    rightTrack.isVelocityClosedLoop();
        bool activeLaunchDemand = leftMotionDemand || rightMotionDemand;
        if (bothTracksClosedLoop && !bothTracksWereClosedLoop && activeLaunchDemand) {
            // 两侧均稳定起步后，把虚拟动力学状态同步到编码器真实速度。
            // 例如实际为2.1km/h就同步到2.1，而不是人为抬到最低闭环速度2.8。
            v_real = 0.5f * (leftTrack.fastSpeed + rightTrack.fastSpeed);
            spinV = 0.5f * (leftTrack.fastSpeed - rightTrack.fastSpeed);
            longitudinalAccel = 0.0f;
        }
        bothTracksWereClosedLoop = bothTracksClosedLoop;
    }

// 直接履带目标主要给测试/调试使用：绕过手柄油门、刹车、转向动力学。
void TankChassis::processDirectTrackTargets(float leftTarget, float rightTarget, float dt, float pitchAngle) {
        updateGradePitch(pitchAngle, dt);
        v_real = (leftTarget + rightTarget) * 0.5f;
        spinV = (leftTarget - rightTarget) * 0.5f;
        longitudinalAccel = 0.0f;
        driveDirection = 0;
        pendingDriveDirection = 0;
        directionChangeSinceMs = 0;
        startIntentDirection = 0;
        startIntentSinceMs = 0;
        startIntentTiming = false;
        startIntentConfirmed = false;
        engineCmd = 0.0f;
        steerState = 0.0f;
        pivotMode = false;
        leftTrack.update(leftTarget, dt, 0.0f, Config::TRACK_FF_REFERENCE_VOLTAGE,
                         false, abs(leftTarget) >= Config::TRACK_STOP_DEADZONE_KMH, false);
        rightTrack.update(rightTarget, dt, 0.0f, Config::TRACK_FF_REFERENCE_VOLTAGE,
                          false, abs(rightTarget) >= Config::TRACK_STOP_DEADZONE_KMH, false);
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
        bothTracksWereClosedLoop = false;
        engineCmd = 0.0f;
        steerState = 0.0f;
        pivotMode = false;
        leftTrack.driveDirect(leftPwm);
        rightTrack.driveDirect(rightPwm);
    }

// 输出 telemetry 时不直接暴露 TankTrack 对象，避免上层误改底盘内部状态。
void TankChassis::getTrackTelemetry(float& leftTarget, float& leftFastActual,
                           float& leftControlActual, float& leftDisplayActual,
                           float& leftPhaseDeg, float& leftPwm, bool& leftStalled,
                           float& rightTarget, float& rightFastActual,
                           float& rightControlActual, float& rightDisplayActual,
                           float& rightPhaseDeg, float& rightPwm, bool& rightStalled) const {
        leftTarget = leftTrack.targetSpeed;
        leftFastActual = leftTrack.fastSpeed;
        leftControlActual = leftTrack.currentSpeed;
        leftDisplayActual = leftTrack.telemetrySpeed;
        leftPhaseDeg = leftTrack.encoder.getRelativeSprocketPhaseDeg();
        leftPwm = leftTrack.lastPwm;
        leftStalled = leftTrack.stallLatched;
        rightTarget = rightTrack.targetSpeed;
        rightFastActual = rightTrack.fastSpeed;
        rightControlActual = rightTrack.currentSpeed;
        rightDisplayActual = rightTrack.telemetrySpeed;
        rightPhaseDeg = rightTrack.encoder.getRelativeSprocketPhaseDeg();
        rightPwm = rightTrack.lastPwm;
        rightStalled = rightTrack.stallLatched;
}

// stop() 不只是把 PWM 归零，还清掉所有滤波/积分/惯量历史。
// 这样断连或低压恢复后，不会带着旧状态突然输出一脚力。
void TankChassis::stop() { 
        v_real = 0; spinV = 0; 
        longitudinalAccel = 0.0f;
        // 保留最近一次行驶方向。普通安全停车后请求相反方向时，仍必须走完整
        // 的静止2秒确认；上电初始值为0，可自由选择首个方向。
        pendingDriveDirection = 0;
        directionChangeSinceMs = 0;
        startIntentDirection = 0;
        startIntentSinceMs = 0;
        startIntentTiming = false;
        startIntentConfirmed = false;
        bothTracksWereClosedLoop = false;
        resetImuCompensation();
        engineCmd = 0.0f;
        steerState = 0.0f;
        pivotMode = false;
        leftTrack.stop(); rightTrack.stop(); 
    }
