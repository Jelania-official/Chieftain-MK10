#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#define PI 3.1415926
#include "AS201.h"
#include <Servo.h>
#include <SimpleFOC.h>
#include <driver/pcnt.h>




// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
xboxController("28:ea:0b:d9:0b:9f");


//速度曲线
  float v = 0.0;
  float Lv = 0.0;
  float Rv = 0.0;
  const float motor_v_max = 14.5; // 主战坦克极限速度
const float motor_brake_Coeff = 1.5;   // 刹车强度系数（建议在 1.2 ~ 2.0）


// PID参数
  const double mortor_Kp = 2.0;
  const double mortor_Ki = 0.1;
  const double mortor_Kd = 0.5;
  double motor_ROutput = 0;
double motor_LOutput = 0;


// 行走部分驱动引脚（输出）
  #define motor_R_IN1 25
  #define motor_R_IN2 26
  #define motor_R_PWM 27

  #define motor_L_IN1 14
  #define motor_L_IN2 13
#define motor_L_PWM 23


/***************** 编码器参数 *****************/
  #define motor_R_sensorA 34    // 右轮编码器 A
  #define motor_R_sensorB 35    // 右轮编码器 B
  #define motor_L_sensorA 32    // 左轮编码器 A
  #define motor_L_sensorB 33    // 左轮编码器 B

  // 保留原先参数
  float motor_R_currentSpeed = 0;
  float motor_L_currentSpeed = 0;

  const int encoderPPR = 7;    // 编码器每转脉冲数
  const int gearRatio = 59;    // 减速比
  const float wheelRPMToSpeed = 13.3 / 540.0;  // 保持你原来的换算（m/s per RPM）

  // 用 PCNT 的单位（我们用 PCNT_UNIT_0 处理右轮，PCNT_UNIT_1 处理左轮）
  static int16_t lastRCount = 0;      // 上一次读取的 PCNT 原始计数（16-bit）
  static int16_t lastLCount = 0;

  unsigned long RlastEncoderTime = 0;
unsigned long LlastEncoderTime = 0;


// PWM 通道、频率、分辨率
  #define motor_PWMchannel_R 0  // 控制 motor_R_PWM（右轮）
  #define motor_PWMchannel_L 1  // 控制 motor_L_PWM（左轮）
  #define motor_PWM_FREQ 10000       // 10kHz，适合电机
#define motor_PWM_RESOLUTION 8     // 8位，0~255


//加入倒车
  bool reverseMode = false;           // 是否倒车模式
  unsigned long rtPressedStartTime = 0; // 右扳机按下开始时间（毫秒）
  const float rtThreshold = 0.2;      // 右扳机按下阈值（0~1之间）
  const unsigned long rtHoldDuration = 2000; // 持续按下时间，单位ms（2秒）
const float reverseVmax = 6.0;  // 倒车限速6 m/s


struct motorPID {
  double previousError = 0.0;
  double integral = 0.0;
};
  motorPID pidR;
motorPID pidL;


//双稳
  AS201 imu_chassis(16, 17, Serial2);  // 车体 IMU
  AS201 imu_turret(5, 2, Serial1);     // 炮塔 IMU

  SensorData* sensor_chassisData;  
  SensorData* sensor_turretData;  

  //打开双稳获取当前姿态
  bool switchState = false;   // A键双稳开关
  bool lastAState = false;    // 防止长按重复触发

  float turret_saved_roll  = 0;
  float turret_saved_pitch = 0;
  float turret_saved_yaw_cont   = 0;
  float turret_yaw_cont_now = 0;
  // 灵敏度（倍数）
  float turret_Sensitivity = 1.0;

  // 灵敏度范围
  const float SENS_MIN = 0.3;
  const float SENS_MAX = 2.0;

  // 最大角速度（°/s）
  const float yaw_Max_Vel   = 60.0;
  const float pitch_Max_Vel = 40.0;

  //连续角状态
  float yaw_raw_last_rad = 0.0f;   // 上一次原始 yaw（rad）
  float yaw_cont_rad     = 0.0f;   // 连续 yaw（rad）
  bool  yaw_init_done    = false;  // 首次初始化标志



//舵机
  Servo pitchServo;  // 舵机对象
  // 输入角度范围（单位：度）
  const float ANGLE_MIN = 0.0;
  const float ANGLE_MAX = 180.0;
  float servo_center_angle = 90.0; // 舵机实际中位

  // 对应舵机 PWM 范围（单位：微秒）
  // 根据产品规格，通常数字舵机 PWM: 500~2500us 对应 0~180°
  const int PWM_MIN = 500;  
  const int PWM_MAX = 2500;

  // 如果目标角度和舵机角度存在偏差，可调整校准参数
  float angleOffset = 0.0;      // 角度偏移，用于零点校准
  float angleScale  = 1.0;      // 线性比例系数，用于非 1:1 映射

  // ---------------- PID 参数 ----------------
  // 位置环
  float servo_P_pos = 2.0;
  float servo_I_pos = 0.0;
  float servo_D_pos = 0.2;

  // 速度环
  float servo_P_vel = 1.2;
  float servo_I_vel = 0.0;
  float servo_D_vel = 0.05;

  // PID 状态变量
  float servo_pos_err_last = 0;// 上次位置误差
  float servo_vel_err_last = 0;// 上次速度误差
  float servo_pos_int = 0;// 位置积分
float servo_vel_int = 0;// 速度积分


//无刷电机控制
  // ======================================================
  //                串级 PID 结构体与函数
  // ======================================================
  
  // 单级 PID 结构体
  typedef struct
  {
    float kp, ki, kd;
    float error, lastError;
    float integral, maxIntegral;
    float output, maxOutput;
  } PID;

  // 串级 PID 结构体
  typedef struct
  {
    PID outer;    // 位置环
    PID inner;    // 速度环
    float output; // 最终 Uq 电压
    float ff_gain;    // 前馈系数 Kff
  } CascadePID;

  // 初始化单级 PID
  void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
  {
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->error = pid->lastError = 0;
    pid->integral = 0;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    pid->output = 0;
  }

  // 一次 PID 计算
  void PID_Calc(PID *pid, float ref, float fdb, float dt)
  {
    pid->lastError = pid->error;
    pid->error = ref - fdb;
    float P = pid->kp * pid->error;
    pid->integral += pid->ki * pid->error * dt;
    // 积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;
    float I = pid->integral;
    float D = 0.0f;
    if (dt > 1e-5f) {
      D = pid->kd * (pid->error - pid->lastError) / dt;
    }
    pid->output = P + I + D;
    // 输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
  }

  // 串级调用
  void PID_CascadeCalc(CascadePID *cp, float posRef, float posFdb, float velFdb, float chassisVel, float dt)
  {
    PID_Calc(&cp->outer, posRef, posFdb, dt);
    PID_Calc(&cp->inner, cp->outer.output, velFdb, dt);
    float ff = cp->ff_gain * chassisVel;
    cp->output = cp->inner.output + ff;
  }

  // ======================================================
  //               SimpleFOC
  // ======================================================

  // ——— SimpleFOC 对象声明 ——//

  // 硬件引脚
  const int yaw_pinPWM_A = 18;
  const int yaw_pinPWM_B = 19;
  const int yaw_pinPWM_C = 12;
  const int yaw_pinEn = 4;

  // FOC 对象
  BLDCMotor yaw_motor = BLDCMotor(7); // 7 极对数
  BLDCDriver3PWM yaw_driver = BLDCDriver3PWM(yaw_pinPWM_A, yaw_pinPWM_B, yaw_pinPWM_C, yaw_pinEn);
  MagneticSensorI2C yaw_sensor = MagneticSensorI2C(AS5600_I2C);

  // 串级 PID 实例
  CascadePID yawPID;


//输出手柄输入
  String xbox_string()
  {
  String str = String(xboxController.xboxNotif.btnY) + "," +
               String(xboxController.xboxNotif.btnX) + "," +
               String(xboxController.xboxNotif.btnB) + "," +
               String(xboxController.xboxNotif.btnA) + "," +
               String(xboxController.xboxNotif.btnLB) + "," +
               String(xboxController.xboxNotif.btnRB) + "," +
               String(xboxController.xboxNotif.btnSelect) + "," +
               String(xboxController.xboxNotif.btnStart) + "," +
               String(xboxController.xboxNotif.btnXbox) + "," +
               String(xboxController.xboxNotif.btnShare) + "," +
               String(xboxController.xboxNotif.btnLS) + "," +
               String(xboxController.xboxNotif.btnRS) + "," +
               String(xboxController.xboxNotif.btnDirUp) + "," +
               String(xboxController.xboxNotif.btnDirRight) + "," +
               String(xboxController.xboxNotif.btnDirDown) + "," +
               String(xboxController.xboxNotif.btnDirLeft) + "," +
               String(xboxController.xboxNotif.joyLHori) + "," +
               String(xboxController.xboxNotif.joyLVert) + "," +
               String(xboxController.xboxNotif.joyRHori) + "," +
               String(xboxController.xboxNotif.joyRVert) + "," +
               String(xboxController.xboxNotif.trigLT) + "," +
               String(v) + "," +
               String(xboxController.xboxNotif.trigRT) + "\n";
  return str;
};


/*********************************************/


// 手柄通信维护
void handleXboxController() {
 
  xboxController.onLoop();

  // 判断是否连接成功
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {

      // 可选功能：振动演示
      // demoVibration();
      // demoVibration_2();
    }
  } else {
    Serial.println("not connected");

    // 如果失败次数太多，自动重启
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }
}

// ========== PCNT 初始化（放在 setup() 中调用） ==========
void pcnt_init_encoders() {
  // 配置右轮 PCNT（PCNT_UNIT_0）
  pcnt_config_t pcnt_cfg_r;
  pcnt_cfg_r.pulse_gpio_num = motor_R_sensorA;
  pcnt_cfg_r.ctrl_gpio_num  = motor_R_sensorB;
  pcnt_cfg_r.channel       = PCNT_CHANNEL_0;
  pcnt_cfg_r.unit          = PCNT_UNIT_0;
  // 在脉冲上升下降时都计数，但方向由 ctrl（B）决定：上升时增/减、下降时增/减
  pcnt_cfg_r.pos_mode = PCNT_COUNT_INC;   // pulse 上升沿时计数 +1
  pcnt_cfg_r.neg_mode = PCNT_COUNT_DEC;   // pulse 下降沿时计数 -1
  pcnt_cfg_r.lctrl_mode = PCNT_MODE_KEEP; // ctrl=low 时保持模式
  pcnt_cfg_r.hctrl_mode = PCNT_MODE_REVERSE; // ctrl=high 时反转极性
  pcnt_cfg_r.counter_h_lim = 32767;
  pcnt_cfg_r.counter_l_lim = -32768;
  pcnt_unit_config(&pcnt_cfg_r);

  // 配置左轮 PCNT（PCNT_UNIT_1）
  pcnt_config_t pcnt_cfg_l;
  pcnt_cfg_l.pulse_gpio_num = motor_L_sensorA;
  pcnt_cfg_l.ctrl_gpio_num  = motor_L_sensorB;
  pcnt_cfg_l.channel       = PCNT_CHANNEL_0;
  pcnt_cfg_l.unit          = PCNT_UNIT_1;
  pcnt_cfg_l.pos_mode = PCNT_COUNT_INC;
  pcnt_cfg_l.neg_mode = PCNT_COUNT_DEC;
  pcnt_cfg_l.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_cfg_l.hctrl_mode = PCNT_MODE_REVERSE;
  pcnt_cfg_l.counter_h_lim = 32767;
  pcnt_cfg_l.counter_l_lim = -32768;
  pcnt_unit_config(&pcnt_cfg_l);

  // 启动计数器
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);

  // 建议：把引脚设置为输入并上拉（可保留）
  pinMode(motor_R_sensorA, INPUT_PULLUP);
  pinMode(motor_R_sensorB, INPUT_PULLUP);
  pinMode(motor_L_sensorA, INPUT_PULLUP);
  pinMode(motor_L_sensorB, INPUT_PULLUP);

  // 初始化时间戳
  RlastEncoderTime = millis();
  LlastEncoderTime = millis();

  // 读取初始计数值到 lastCount，避免第一次大跳
  int16_t cnt;
  pcnt_get_counter_value(PCNT_UNIT_0, &cnt);
  lastRCount = cnt;
  pcnt_get_counter_value(PCNT_UNIT_1, &cnt);
  lastLCount = cnt;
}


// ========== 替代原有 RgetSpeed() / LgetSpeed() 的实现 ==========
float RgetSpeed() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - RlastEncoderTime;
  static float RRPM = 0;
  if (deltaTime >= 5) {  // 每5ms计算一次
    // 更新硬件计数并累计
    int16_t cntR;
    pcnt_get_counter_value(PCNT_UNIT_0, &cntR);
    int32_t delta = (int32_t)cntR - (int32_t)lastRCount;
    lastRCount = cntR;

    // 使用 delta 作为该时间段内的脉冲数（正负）
    // 计算 RPM（注意单位与原来保持一致）
    RRPM = (delta / (float)encoderPPR / gearRatio) * (60000.0 / (float)deltaTime);

    // 清零硬件计数（可选）：不清零也行，因为我们用差值
    // pcnt_counter_clear(PCNT_UNIT_0);

    RlastEncoderTime = currentTime;
  }
  float RWheelSpeed = RRPM * wheelRPMToSpeed;
  return RWheelSpeed;
}

float LgetSpeed() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - LlastEncoderTime;
  static float LRPM = 0;
  if (deltaTime >= 5) {  // 每5ms计算一次
    int16_t cntL;
    pcnt_get_counter_value(PCNT_UNIT_1, &cntL);
    int32_t delta = (int32_t)cntL - (int32_t)lastLCount;
    lastLCount = cntL;

    LRPM = (delta / (float)encoderPPR / gearRatio) * (60000.0 / (float)deltaTime);

    // pcnt_counter_clear(PCNT_UNIT_1);

    LlastEncoderTime = currentTime;
  }
  float LWheelSpeed = LRPM * wheelRPMToSpeed;
  return LWheelSpeed;
}

//PID计算
double motor_calculatePID(motorPID& pid, double targetSpeed, double actualSpeed, double dt) {
  if (dt <= 0) dt = 0.001; // 防护
  if (abs(targetSpeed) < 0.015) targetSpeed = 0;
  double currentError = targetSpeed - actualSpeed;
  pid.integral += currentError * dt;  // 积分按时间累加

  // anti-windup
  const double I_LIMIT = 1000.0;
  if (pid.integral > I_LIMIT) pid.integral = I_LIMIT;
  if (pid.integral < -I_LIMIT) pid.integral = -I_LIMIT;

  double derivative = (currentError - pid.previousError) / dt;
  pid.previousError = currentError;

  double output = mortor_Kp * currentError + mortor_Ki * pid.integral + mortor_Kd * derivative;
  output = constrain(output, -255, 255);
  
  if (abs(output) < 35 && abs(targetSpeed) > 0.1)
    output = copysign(35, output);  // 最小输出保护，防止低速时电机不转动
  return output;
}


//油门模拟（加倒挡）
  void simulateThrottle(float dt) {
  if (!xboxController.isConnected()) return;

  unsigned long currentTime = millis();

  // 读取扳机值（归一化到0~1）
  float k = xboxController.xboxNotif.trigLT / 1023.0;   // 油门（前进）
  float brake = xboxController.xboxNotif.trigRT / 1023.0; // 刹车（右扳机）

  // 判断是否静止（速度非常小）
  bool isStopped = (v < 0.1);

  // 判断右扳机是否超过阈值
  if (brake > rtThreshold) {
    if (rtPressedStartTime == 0) {
      rtPressedStartTime = currentTime;  // 开始计时
    } else {
      // 判断是否持续超过2秒
      if (isStopped && (currentTime - rtPressedStartTime >= rtHoldDuration)) {
        reverseMode = true; // 进入倒车模式
        v = 0;   // 切换瞬间清零速度
      }
    }
  } else {
    // 右扳机松开，清空计时
    rtPressedStartTime = 0;
if (v > -0.1 && reverseMode && k > rtThreshold) {
  reverseMode = false;
}  }   //再次按左扳机，退出倒车

  // 根据是否倒车模式计算加速度和速度
  float k_adj = k * k;
  float rollingResistance = (abs(v) < 3.0) ? 0.5 : 0.338;// 低速滚阻：速度越低，滚阻越大（模拟履带 + 慢起步）
  float a = 0.0;

  if (!reverseMode) {
    // 正常前进模式
    a = 1.4 * k_adj - 0.006 * v * v - rollingResistance - motor_brake_Coeff * brake;// 加速度公式（推力 - 阻力 - 滚阻 - 刹车）
    if (v < 0.5) a *= 0.5;  // 起步迟钝
    v = v + a * dt;
    if (v < 0) v = 0;
    if (v > motor_v_max) v = motor_v_max;
  } else {
    // 倒车模式：加速度用同样公式，但油门变成倒车时的刹车，速度取负值表示倒车
    // 注意此时k（油门）不再代表推力，而是“倒车时的刹车”，即用k替代brake
    // 这里做个简单处理：用k代替刹车，推力为负方向
    a = -1.4 * brake * brake + 0.006 * v * v + rollingResistance + motor_brake_Coeff * k; 
     
    v = v + a * dt;
    if (v < -reverseVmax) v = -reverseVmax;
    
    if (v > 0) v = 0; // 倒车时速度不能正
  }

  // 输出调试信息
  //String output = "ReverseMode=" + String(reverseMode ? "ON" : "OFF") +
  //                ", LT=" + String(k, 3) +
  //                ", RT=" + String(brake, 3) +
  //                ", a=" + String(a, 3) +
  //                ", v=" + String(v, 3) + " m/s";
  //Serial.println(output);
}

//加入转向和自转
  void updateWheelSpeed(float dt) {
  // 获取摇杆水平值
  int rawYaw = xboxController.xboxNotif.joyLHori; // 范围 0~65535
  float yawCenter = 32767.5;
  float deadZone = 4000;  // 死区范围，可调节
  float yaw = (rawYaw - yawCenter) / yawCenter; // 归一化到 [-1, 1]

  // 死区处理
  if (abs(yaw) < (deadZone / yawCenter)) {
    yaw = 0.0;
  }

  // 判断是否进入原地转向模式
  bool isSpinMode = abs(v) < 0.1 && abs(yaw) > 0.2;
  static float spinV = 0.0;
  if (isSpinMode) {
  // 原地转向逻辑：摇杆偏移量决定转速大小
  float k = yaw;           // 作为转速模拟的油门
  float rollingResistance = 0.5;
  float a = 1.4 * k - rollingResistance;
  spinV += a * dt;
  if (spinV > 1.22) spinV = 1.22;

  // 根据 yaw 的符号确定左右履带方向
  float dir = copysign(1.0, yaw);  // +1 或 -1

  Lv = -spinV * dir;
  Rv =  spinV * dir;
  }else {
    spinV = 0.0;
    // 正常差速转向：两侧履带速度按 yaw 做调整
    float turnRatio = constrain(yaw, -1.0, 1.0);
    float turnFactor = 0.5; // 可调差速比例（建议在 0.4~0.7）

    Lv = v * (1.0 - turnFactor * turnRatio); // 左履带
    Rv = v * (1.0 + turnFactor * turnRatio); // 右履带

    // 限速处理，避免某侧履带速度超过 motor_v_max
    float maxV = reverseMode ? reverseVmax : motor_v_max;
    Lv = constrain(Lv, -maxV, maxV);
    Rv = constrain(Rv, -maxV, maxV);
  }
}

//电机正反转驱动
  void RdriveMotor(double speed) {

  // 限幅
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(motor_R_IN1, LOW);   // 正转 motor_R_IN1=0 motor_R_IN2=1
    digitalWrite(motor_R_IN2, HIGH);
    ledcWrite(motor_PWMchannel_R, speed);
  } else if (speed < 0) {
    digitalWrite(motor_R_IN1, HIGH);  // 反转 motor_R_IN1=1 motor_R_IN2=0
    digitalWrite(motor_R_IN2, LOW);
    ledcWrite(motor_PWMchannel_R, -speed);

  } else {
    // 停止
    digitalWrite(motor_R_IN1, LOW);
    digitalWrite(motor_R_IN2, LOW);
    ledcWrite(motor_PWMchannel_R, 0);
  }
}

  void LdriveMotor(double speed) {

  // 限幅
  speed = constrain(speed, -255, 255);

  if (speed < 0) {
    digitalWrite(motor_L_IN1, LOW);   // 反转 motor_L_IN1=0 motor_L_IN2=1
    digitalWrite(motor_L_IN2, HIGH);
    ledcWrite(motor_PWMchannel_L, -speed);
  } else if (speed > 0) {
    digitalWrite(motor_L_IN1, HIGH);  // 轮子正转 motor_L_IN1=1 motor_L_IN2=0
    digitalWrite(motor_L_IN2, LOW);
    ledcWrite(motor_PWMchannel_L, speed);
  } else {
    // 停止
    digitalWrite(motor_L_IN1, LOW);
    digitalWrite(motor_L_IN2, LOW);
    ledcWrite(motor_PWMchannel_L, 0);
  }
}

// A 键按下 → 切换双稳开关 → 如果是 ON 则保存炮闩IMU角度，右摇杆控制 + 灵敏度调节
  void GunPosition(bool A_pressed, float dt)
{
    // 上升沿触发：上次未按，这次按下
    if (A_pressed && !lastAState) {

        // 切换双稳状态
        switchState = !switchState;

        //Serial.print("Switch toggled -> ");
        //Serial.println(switchState ? "ON" : "OFF");

        // 如果现在切换到 "ON"，则保存炮塔IMU角度
        if (switchState) {
            turret_saved_roll  = sensor_turretData->roll;
            turret_saved_pitch = sensor_turretData->pitch;
            turret_saved_yaw_cont   = turret_yaw_cont_now;

            //Serial.println("=== Turret Angle Saved! ===");
            //Serial.print("Roll: ");  Serial.println(turret_saved_roll);
            //Serial.print("Pitch: "); Serial.println(turret_saved_pitch);
            //Serial.print("Yaw: ");   Serial.println(turret_saved_yaw_cont);
        }
    }

    // 记录按键状态（防抖）
    lastAState = A_pressed;

    if (switchState) {
    // -----------------------
    // 0. 灵敏度调节（方向键 ↑ ↓）
    // -----------------------
    static uint32_t lastSensAdjust = 0;
    if (millis() - lastSensAdjust > 200) {
      if (xboxController.xboxNotif.btnDirUp)   turret_Sensitivity += 0.1;
      if (xboxController.xboxNotif.btnDirDown) turret_Sensitivity -= 0.1;
      lastSensAdjust = millis();
    }
    turret_Sensitivity = constrain(turret_Sensitivity, SENS_MIN, SENS_MAX);

    // -----------------------
    // 1. 读取右摇杆
    // -----------------------
    int rawYaw   = xboxController.xboxNotif.joyRHori; // 0~65535
    int rawPitch = xboxController.xboxNotif.joyRVert; // 0~65535
    float center = 32767.5;
    float deadZone = 4000;

    // 归一化到 [-1,1]
    float yawIn   = (rawYaw - center) / center;
    float pitchIn = (rawPitch - center) / center;

    // 死区
    if (abs(rawYaw - center) < deadZone)   yawIn   = 0;
    if (abs(rawPitch - center) < deadZone) pitchIn = 0;

    // 手柄上推 = 画面向上 ⇒ pitch 取反
    pitchIn = -pitchIn;

    // -----------------------
    // 2. 应用灵敏度
    // -----------------------
    yawIn   *= turret_Sensitivity;
    pitchIn *= turret_Sensitivity;

    // -----------------------
    // 3. 计算角速度（线性）
    // -----------------------
    float yawVel   = yawIn   * yaw_Max_Vel;   // °/s
    float pitchVel = pitchIn * pitch_Max_Vel; // °/s

    // -----------------------
    // 4. 积分得到姿态指令
    // -----------------------
    turret_saved_yaw_cont   += yawVel   * dt;
    turret_saved_pitch += pitchVel * dt;

    // 限制俯仰角
    turret_saved_pitch = constrain(turret_saved_pitch, -10.0, 35.0);

    // -----------------------
    // 5. 输出调试信息
    // -----------------------
    /*Serial.print("Saved Yaw=");
    Serial.print(turret_saved_yaw_cont, 2);
    Serial.print(" Pitch=");
    Serial.print(turret_saved_pitch, 2);
    Serial.print(" Sens=");
    Serial.println(turret_Sensitivity, 2);*/
  }
}

// 舵机控制初始化
  void initPitchServo(int pin) {
    pitchServo.attach(pin);  // 连接舵机信号线
}

//舵机控制：双环 PID 角度范围 -10°～30° 中位角度 = 0°
  void ServoPID(float dt)
{
    // -------- 1. 读取 IMU pitch 和 pitch 角速度（X轴）--------
    float pitch_turret  = sensor_turretData->pitch;
    float pitch_chassis = sensor_chassisData->pitch;

    float gyro_chassis = sensor_chassisData->gy;
    float gyro_turret  = sensor_turretData->gy;

    // -------- 2. 位置环：角度误差 --------
    float err_pos = turret_saved_pitch - pitch_turret;
      const float SERVO_I_MAX = 50.0;
    servo_pos_int += err_pos * dt;
    servo_pos_int = constrain(servo_pos_int, -SERVO_I_MAX, SERVO_I_MAX);

    float d_pos = (err_pos - servo_pos_err_last) / dt;
    static float d_pos_lpf = 0;
    d_pos_lpf += (d_pos - d_pos_lpf) * dt / (0.02f + dt);
    servo_pos_err_last = err_pos;

    float u_pos = servo_P_pos * err_pos + servo_I_pos * servo_pos_int + servo_D_pos * d_pos_lpf;

    // -------- 3. 速度环目标 = 抵消车体角速度 + 位置环输出 --------
    float target_vel = -gyro_chassis + u_pos;

    // -------- 4. 速度环：速度误差 --------
    float err_vel = target_vel - gyro_turret;

    servo_vel_int += err_vel * dt;

    // 积分限幅，防止积分风暴
    servo_vel_int = constrain(servo_vel_int, -SERVO_I_MAX, SERVO_I_MAX);


    float d_vel = (err_vel - servo_vel_err_last) / dt;
    static float d_vel_lpf = 0;
    d_vel_lpf += (d_vel - d_vel_lpf) * dt / (0.02f + dt);
    servo_vel_err_last = err_vel;

    float u_vel = servo_P_vel * err_vel + servo_I_vel * servo_vel_int + servo_D_vel * d_vel_lpf;

    // -------- 5. 将速度输出转换为角度输出（内部单位：度）--------
    static float controlAngle = 0.0;   // 0° = 舵机中位
    controlAngle += u_vel * dt;

    // 限制控制范围（你要求：-10° 到 30°）
    controlAngle = constrain(controlAngle, -10.0f, 30.0f);

    // -------- 6. 将 -10~30° 转换为舵机 PWM --------
    // 这里我们假设舵机中位是 PWM 中值(1500us)
    float servoAnglePhysical = controlAngle + servo_center_angle;  
    // 示例：servo_center_angle = 90°

    // 校准参数处理
    float calibratedAngle = (servoAnglePhysical + angleOffset) * angleScale;

    // 限舵机物理范围
    calibratedAngle = constrain(calibratedAngle, ANGLE_MIN, ANGLE_MAX);

    // 转换 PWM
    int pwm = map(calibratedAngle, ANGLE_MIN, ANGLE_MAX, PWM_MIN, PWM_MAX);

    // 如果双稳关闭，控制角度归零
    if(!switchState) 
      controlAngle = 0;
      servo_pos_int = 0;
      servo_vel_int = 0;
      servo_pos_err_last = 0;
    servo_vel_err_last = 0;

    // 输出
    pitchServo.writeMicroseconds(pwm);
}

//yaw 角度连续化函数
float yaw_Continuous_deg(float deg_yaw_turret_now)
{
    // 1. 度 → 弧度
    float rad_yaw_turret_now = deg_yaw_turret_now * DEG_TO_RAD;

    // 2. 第一次调用，直接初始化
    if (!yaw_init_done) {
        yaw_raw_last_rad = rad_yaw_turret_now;
        yaw_cont_rad = rad_yaw_turret_now;
        yaw_init_done = true;

        return deg_yaw_turret_now;
    }

    // 3. 计算相邻两次的差值
    float dyaw = rad_yaw_turret_now - yaw_raw_last_rad;

    // 4. 处理跨 360° 回跳（unwrap 核心）
    if (dyaw > M_PI) {
        dyaw -= 2.0f * M_PI;
    } else if (dyaw < -M_PI) {
        dyaw += 2.0f * M_PI;
    }

    // 5. 累加得到连续角
    yaw_cont_rad += dyaw;
    float yaw_cont_deg = yaw_cont_rad/DEG_TO_RAD; // 转回度
    yaw_raw_last_rad = rad_yaw_turret_now;

    return yaw_cont_deg;
}

/*************************************************/


void setup()
{
  Serial.begin(115200);
  xboxController.begin();

  // 初始化编码器
  pcnt_init_encoders();

  // 初始化电机驱动引脚
  pinMode(motor_R_IN1, OUTPUT);
  pinMode(motor_R_IN2, OUTPUT);

  pinMode(motor_L_IN1, OUTPUT);
  pinMode(motor_L_IN2, OUTPUT);

  // PWM 初始化
  ledcSetup(motor_PWMchannel_R, motor_PWM_FREQ, motor_PWM_RESOLUTION);
  ledcAttachPin(motor_R_PWM, motor_PWMchannel_R);

  ledcSetup(motor_PWMchannel_L, motor_PWM_FREQ, motor_PWM_RESOLUTION);
  ledcAttachPin(motor_L_PWM, motor_PWMchannel_L);
  
  // 初始化双稳
  Serial1.begin(115200, SERIAL_8N1, 5, 2);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);


  // 将全局引用指向 IMU 内部 data
  sensor_chassisData = &imu_chassis.getData();
  sensor_turretData  = &imu_turret.getData();

  // 初始化舵机
  initPitchServo(15); // 舵机信号接在 GPIO15

  // 无刷电机控制初始化

    // 1) 驱动初始化
    yaw_driver.voltage_power_supply = 12.0;
    yaw_driver.init();

    // 2) 传感器初始化
    yaw_sensor.init();
    yaw_motor.linkSensor(&yaw_sensor);

    // 3) 链接驱动与电机
    yaw_motor.linkDriver(&yaw_driver);
    yaw_motor.voltage_limit = 12.0;

    // 4) 选择 torque 模式，不使用库内置 PID
    yaw_motor.controller = MotionControlType::torque;
    // 禁掉库内速度 PI，避免与您的 inner PID 干扰
    yaw_motor.PID_velocity.P = 0;
    yaw_motor.PID_velocity.I = 0;
    yaw_motor.PID_velocity.D = 0;
    yaw_motor.LPF_velocity.Tf = 0.01; // 低通滤波时间常数

    // 5) FOC 初始化
    yaw_motor.init();
    yaw_motor.initFOC();

    // 6) 自己的 PID 参数，需根据电机与负载调参
    // 外环：位置→输出速度 (rad/s)
    PID_Init(&yawPID.outer, 1, 0.0, 1.0, 0.0, 12);
    // 内环：速度→输出 Uq 电压 (V)
    PID_Init(&yawPID.inner, 0.1, 0.01, 0.00, 1000, 6.0);

}



void loop()
{
  /************** ① 最高优先级：FOC（每次 loop 都跑） **************/
    yaw_motor.loopFOC();   // ❗不要加 if，不要用 millis，不要 delay
    yaw_motor.move();

    /************** ② 当前时间（微秒级） **************/
    static uint32_t lastMicros = micros();
  uint32_t nowMicros = micros();


  /************** ③ IMU 固定频率更新（500Hz） **************/
    static uint32_t lastIMU = 0;
    if (nowMicros - lastIMU >= 2000) {   // 2000us = 500Hz
      lastIMU = nowMicros;

      imu_chassis.update();
      imu_turret.update();

      // yaw 连续化建议放在 IMU 更新后
      turret_yaw_cont_now = yaw_Continuous_deg(sensor_turretData->yaw);
  }


  /************** ④ 用户输入 / 状态机（50Hz） **************/
    static uint32_t lastUI = 0;
    if (nowMicros - lastUI >= 20000) {    // 20ms = 50Hz
      float dt_ui = (nowMicros - lastUI) * 1e-6f;
      lastUI = nowMicros;

      handleXboxController();

      bool A_pressed = xboxController.xboxNotif.btnA;
      GunPosition(A_pressed, dt_ui); 
  }



  /************** ⑤ 控制主循环（5ms = 200Hz） **************/
    static uint32_t lastCtrl = 0;
    if (nowMicros - lastCtrl >= 5000) {
      float dt = (nowMicros - lastCtrl) * 1e-6f;
      lastCtrl = nowMicros;

      /***** 行走部分 *****/
      if (xboxController.isConnected() &&!xboxController.isWaitingForFirstNotification()) {
        simulateThrottle(dt);
        updateWheelSpeed(dt);

        motor_R_currentSpeed = RgetSpeed();
        motor_ROutput = motor_calculatePID(pidR, Rv, motor_R_currentSpeed, dt);
        RdriveMotor(motor_ROutput);

        motor_L_currentSpeed = LgetSpeed();
        motor_LOutput = motor_calculatePID(pidL, Lv, motor_L_currentSpeed, dt);
        LdriveMotor(motor_LOutput);
      }
      else {
        v = 0; Lv = 0; Rv = 0;
        RdriveMotor(0);
        LdriveMotor(0);

        pidR.integral = pidL.integral = 0;
        pidR.previousError = pidL.previousError = 0;
      }


      /***** 双稳控制 *****/
      if (switchState) {
        ServoPID(dt);

        float rad_yaw_now   = turret_yaw_cont_now * DEG_TO_RAD;
        float rad_yaw_ref   = turret_saved_yaw_cont * DEG_TO_RAD;
        float rad_yaw_gyro  = sensor_turretData->gz * DEG_TO_RAD;
        float rad_yaw_chassis_gyro = -sensor_chassisData->gz * DEG_TO_RAD;
            
        static float rad_yaw_gyro_lpf = 0;
        const float gyroTf = 0.01f; // 10ms
        rad_yaw_gyro_lpf += (rad_yaw_gyro - rad_yaw_gyro_lpf) * dt / (gyroTf + dt);

        PID_CascadeCalc(
          &yawPID,
          rad_yaw_ref,
          rad_yaw_now,
          rad_yaw_gyro_lpf,
          rad_yaw_chassis_gyro,
          dt
        );

        yaw_motor.target = constrain(
          yawPID.output,
          -yaw_motor.voltage_limit,
          yaw_motor.voltage_limit
        );

      }
    }


  /************** ⑥ 调试输出（低频，不阻塞） **************/
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();

      Serial.print("R:");
      Serial.print(motor_R_currentSpeed);
      Serial.print(" L:");
      Serial.print(motor_L_currentSpeed);
      Serial.print(" yaw:");
      Serial.println(turret_yaw_cont_now);
    }
}
