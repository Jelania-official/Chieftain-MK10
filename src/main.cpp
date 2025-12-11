#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Ticker.h>
#include <math.h>
#include <HardwareSerial.h>
#define PI 3.1415926
#include "AS201.h"
#include <Servo.h>



// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
xboxController("28:ea:0b:d9:0b:9f");


//速度曲线
  float v = 0.0;
  float Lv = 0.0;
  float Rv = 0.0;
  unsigned long prevTime = 0;
  const float vmax = 14.5; // 主战坦克极限速度
const float brakeCoeff = 1.5;   // 刹车强度系数（建议在 1.2 ~ 2.0）


// PID参数
  const double Kp = 2.0;
  const double Ki = 0.1;
  const double Kd = 0.5;
  double ROutput = 0;
double LOutput = 0;


// 行走部分驱动引脚（输出）
  #define AIN1 25
  #define AIN2 26
  #define PWMA 33

  #define BIN1 27
  #define BIN2 14
#define PWMB 32


/***************** 编码器参数 *****************/
  #define R1 18    // 右轮编码器引脚A
  #define R2 19    // 右轮编码器引脚B
  volatile long Rcounter = 0; // 右轮脉冲计数
  float RcurrentSpeed = 0;

  #define L1 21    // 左轮编码器引脚A
  #define L2 22    // 左轮编码器引脚B
  volatile long Lcounter = 0; // 右轮脉冲计数
  float LcurrentSpeed = 0;

  unsigned long RlastEncoderTime = 0;
  unsigned long LlastEncoderTime = 0;
  const int encoderPPR = 7;    // 编码器每转脉冲数
  const int gearRatio = 59;   // 减速比
const float wheelRPMToSpeed = 13.3 / 540.0;  // ≈ 0.02463 m/s per RPM


// PWM 通道、频率、分辨率
  #define CHANNEL_A 0  // 控制 PWMA（右轮）
  #define CHANNEL_B 1  // 控制 PWMB（左轮）
  #define PWM_FREQ 10000       // 10kHz，适合电机
#define PWM_RESOLUTION 8     // 8位，0~255


//加入倒车
  bool reverseMode = false;           // 是否倒车模式
  unsigned long rtPressedStartTime = 0; // 右扳机按下开始时间（毫秒）
  const float rtThreshold = 0.2;      // 右扳机按下阈值（0~1之间）
  const unsigned long rtHoldDuration = 2000; // 持续按下时间，单位ms（2秒）
const float reverseVmax = 6.0;  // 倒车限速6 m/s


struct PIDController {
  double previousError = 0.0;
  double integral = 0.0;
};
  PIDController pidR;
PIDController pidL;


//双稳
  AS201 imu_chassis(16, 17, Serial2);  // 车体 IMU
  AS201 imu_turret(26, 27, Serial1);     // 炮塔 IMU

  SensorData* chassisData;  
  SensorData* turretData;  

  //打开双稳获取当前姿态
  bool switchState = false;   // A键双稳开关
  bool lastAState = false;    // 防止长按重复触发

  float turret_saved_roll  = 0;
  float turret_saved_pitch = 0;
  float turret_saved_yaw   = 0;

  // 灵敏度（倍数）
  float turretSensitivity = 1.0;

  // 灵敏度范围
  const float SENS_MIN = 0.3;
  const float SENS_MAX = 2.0;

  // 最大角速度（°/s）
  const float yawMaxVel   = 60.0;
const float pitchMaxVel = 40.0;


//舵机
  Servo pitchServo;  // 舵机对象
  // 输入角度范围（单位：度）
  const float ANGLE_MIN = 0.0;
  const float ANGLE_MAX = 180.0;

  // 对应舵机 PWM 范围（单位：微秒）
  // 根据产品规格，通常数字舵机 PWM: 1000~2000us 对应 0~180°
  const int PWM_MIN = 1000;  
  const int PWM_MAX = 2000;

  // 如果目标角度和舵机角度存在偏差，可调整校准参数
  float angleOffset = 0.0;      // 角度偏移，用于零点校准
float angleScale  = 1.0;      // 线性比例系数，用于非 1:1 映射



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

// 编码器中断服务函数
  void IRAM_ATTR RencoderISR() {
  bool a = digitalRead(R1);
  bool b = digitalRead(R2);
  if (a == b) Rcounter++;
  else Rcounter--;
  }

  void IRAM_ATTR LencoderISR() {
  bool a = digitalRead(L1);
  bool b = digitalRead(L2);
  if (a == b) Lcounter--;
  else Lcounter++;
}

// 获取真实转速(m/s)
float RgetSpeed() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - RlastEncoderTime;
  static float RRPM= 0;
  if(deltaTime >= 50) {  // 每50ms计算一次转速
    RRPM = (Rcounter / (float)encoderPPR / gearRatio) * (60000.0 / deltaTime);
    Rcounter = 0;
    RlastEncoderTime = currentTime;
  }
  float RlinearSpeed = RRPM * wheelRPMToSpeed;
  return RlinearSpeed;
}

float LgetSpeed() {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - LlastEncoderTime;
    static float LRPM= 0;
    if(deltaTime >= 50) {  // 每50ms计算一次转速
      LRPM = (Lcounter / (float)encoderPPR / gearRatio) * (60000.0 / deltaTime);
      Lcounter = 0;
      LlastEncoderTime = currentTime;
    }
    float LlinearSpeed = LRPM * wheelRPMToSpeed;
    return LlinearSpeed;
}

//PID计算
double calculatePID(PIDController& pid, double targetSpeed, double actualSpeed, double dt) {
  if (dt <= 0) dt = 0.001; // 防护
  double currentError = targetSpeed - actualSpeed;
  pid.integral += currentError * dt;  // 积分按时间累加

  // anti-windup
  const double I_LIMIT = 1000.0;
  if (pid.integral > I_LIMIT) pid.integral = I_LIMIT;
  if (pid.integral < -I_LIMIT) pid.integral = -I_LIMIT;

  double derivative = (currentError - pid.previousError) / dt;
  pid.previousError = currentError;

  double output = Kp * currentError + Ki * pid.integral + Kd * derivative;
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
      }
    }
  } else {
    // 右扳机松开，清空计时
    rtPressedStartTime = 0;
if (v > -0.1 && reverseMode && k > rtThreshold) {
  reverseMode = false;
}  }   //再次按左扳机，退出倒车

  // 根据是否倒车模式计算加速度和速度
  float k_adj = pow(k, 2.0);
  float rollingResistance = (abs(v) < 3.0) ? 0.5 : 0.338;// 低速滚阻：速度越低，滚阻越大（模拟履带 + 慢起步）
  float a = 0.0;

  if (!reverseMode) {
    // 正常前进模式
    a = 1.4 * k_adj - 0.006 * v * v - rollingResistance - brakeCoeff * brake;// 加速度公式（推力 - 阻力 - 滚阻 - 刹车）
    if (v < 0.5) a *= 0.5;  // 起步迟钝
    v = v + a * dt;
    if (v < 0) v = 0;
    if (v > vmax) v = vmax;
  } else {
    // 倒车模式：加速度用同样公式，但油门变成倒车时的刹车，速度取负值表示倒车
    // 注意此时k（油门）不再代表推力，而是“倒车时的刹车”，即用k替代brake
    // 这里做个简单处理：用k代替刹车，推力为负方向
    a = -1.4 * pow(brake, 2.0) + 0.006 * v * v + rollingResistance + brakeCoeff * k; 
     
    v = v + a * dt;
    if (v < -reverseVmax) v = -reverseVmax;
    
    if (v > 0) v = 0; // 倒车时速度不能正
  }

  // 输出调试信息
  String output = "ReverseMode=" + String(reverseMode ? "ON" : "OFF") +
                  ", LT=" + String(k, 3) +
                  ", RT=" + String(brake, 3) +
                  ", a=" + String(a, 3) +
                  ", v=" + String(v, 3) + " m/s";
  Serial.println(output);
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

    // 限速处理，避免某侧履带速度超过 vmax
    float maxV = reverseMode ? reverseVmax : vmax;
    Lv = constrain(Lv, -maxV, maxV);
    Rv = constrain(Rv, -maxV, maxV);
  }
}

//电机正反转驱动
void RdriveMotor(double speed) {

  // 限幅
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(AIN1, LOW);   // 正转 AIN1=0 AIN2=1
    digitalWrite(AIN2, HIGH);
    ledcWrite(CHANNEL_A, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, HIGH);  // 反转 AIN1=1 AIN2=0
    digitalWrite(AIN2, LOW);
    ledcWrite(CHANNEL_A, -speed);

  } else {
    // 停止
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(CHANNEL_A, 0);
  }
}

void LdriveMotor(double speed) {

  // 限幅
  speed = constrain(speed, -255, 255);

  if (speed < 0) {
    digitalWrite(BIN1, LOW);   // 反转 BIN1=0 BIN2=1
    digitalWrite(BIN2, HIGH);
    ledcWrite(CHANNEL_B, speed);
  } else if (speed > 0) {
    digitalWrite(BIN1, HIGH);  // 轮子正转 BIN1=1 BIN2=0
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_B, -speed);
  } else {
    // 停止
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_B, 0);
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
            turret_saved_roll  = turretData->roll;
            turret_saved_pitch = turretData->pitch;
            turret_saved_yaw   = turretData->yaw;

            //Serial.println("=== Turret Angle Saved! ===");
            //Serial.print("Roll: ");  Serial.println(turret_saved_roll);
            //Serial.print("Pitch: "); Serial.println(turret_saved_pitch);
            //Serial.print("Yaw: ");   Serial.println(turret_saved_yaw);
        }
    }

    // 记录按键状态（防抖）
    lastAState = A_pressed;

    if (switchState) {
    // -----------------------
    // 0. 灵敏度调节（方向键 ↑ ↓）
    // -----------------------
    if (xboxController.xboxNotif.btnDirUp)   turretSensitivity += 0.1;
    if (xboxController.xboxNotif.btnDirDown) turretSensitivity -= 0.1;
    turretSensitivity = constrain(turretSensitivity, SENS_MIN, SENS_MAX);

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
    yawIn   *= turretSensitivity;
    pitchIn *= turretSensitivity;

    // -----------------------
    // 3. 计算角速度（线性）
    // -----------------------
    float yawVel   = yawIn   * yawMaxVel;   // °/s
    float pitchVel = pitchIn * pitchMaxVel; // °/s

    // -----------------------
    // 4. 积分得到姿态指令
    // -----------------------
    turret_saved_yaw   += yawVel   * dt;
    turret_saved_pitch += pitchVel * dt;

    // 限制俯仰角
    turret_saved_pitch = constrain(turret_saved_pitch, -10.0, 35.0);

    // -----------------------
    // 5. 输出调试信息
    // -----------------------
    /*Serial.print("Saved Yaw=");
    Serial.print(turret_saved_yaw, 2);
    Serial.print(" Pitch=");
    Serial.print(turret_saved_pitch, 2);
    Serial.print(" Sens=");
    Serial.println(turretSensitivity, 2);*/
  }
}

// 舵机控制初始化
void initPitchServo(int pin) {
    pitchServo.attach(pin);  // 连接舵机信号线
}

// 舵机控制函数 输入：目标角度 targetAngle 输出：PWM 控制舵机
void setPitchAngle(float targetAngle) {
    // 1. 校准和比例调整
    float calibratedAngle = (targetAngle + angleOffset) * angleScale;

    // 2. 限制角度在舵机可行范围内
    if (calibratedAngle < ANGLE_MIN) calibratedAngle = ANGLE_MIN;
    if (calibratedAngle > ANGLE_MAX) calibratedAngle = ANGLE_MAX;

    // 3. 映射到 PWM 输出
    int pwm = map(calibratedAngle, ANGLE_MIN, ANGLE_MAX, PWM_MIN, PWM_MAX);

    // 4. 写入舵机
    pitchServo.writeMicroseconds(pwm);
}


/*************************************************/


void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Client");
  xboxController.begin();

  // 初始化编码器
  pinMode(R1, INPUT_PULLUP);
  pinMode(R2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(R1), RencoderISR, RISING);
    
  pinMode(L1, INPUT_PULLUP);
  pinMode(L2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L1), LencoderISR, RISING);


  // 初始化电机驱动引脚
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // PWM 初始化
  ledcSetup(CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, CHANNEL_A);

  ledcSetup(CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMB, CHANNEL_B);
  
  // 初始化双稳
  Serial1.begin(115200, SERIAL_8N1, 26, 27);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);


  // 将全局引用指向 IMU 内部 data
  chassisData = &imu_chassis.getData();
  turretData  = &imu_turret.getData();

  // 初始化舵机
  initPitchServo(13); // 舵机信号接在 GPIO13

}



void loop()
{
 //  管理手柄通信和按键数据
  handleXboxController();

  
  //计算时间
  unsigned long now = millis();
  static unsigned long lastLoop = 0;
  
    // 更新双稳 IMU 数据更新
    imu_chassis.update();
    imu_turret.update();
  
  if (now - lastLoop < 50) return;  // 每 50ms 执行一次逻辑
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;


 if (xboxController.isConnected() && !xboxController.isWaitingForFirstNotification()) {
    //模拟油门
    simulateThrottle(dt);


    // 更新左右轮速度目标（转向）
    updateWheelSpeed(dt);


    //PID 
    RcurrentSpeed = RgetSpeed();
    ROutput = calculatePID(pidR, Rv, RcurrentSpeed, dt);
    RdriveMotor(ROutput);

    LcurrentSpeed = LgetSpeed();
    LOutput = calculatePID(pidL, Lv, LcurrentSpeed, dt);
    LdriveMotor(LOutput);
  }

  else {
    // 未连接时强制停止所有电机
    v = 0; Lv = 0; Rv = 0;
    RdriveMotor(0);
    LdriveMotor(0);
    
    // 重置PID积分项，避免积分饱和
    pidR.integral = 0;
    pidL.integral = 0;
    pidR.previousError = 0;
    pidL.previousError = 0;
  }

  // 调试信息
    static unsigned long lastSerial = 0;
    if (millis() - lastSerial >= 200) {
      lastSerial = millis();
      Serial.print("右轮转速: ");
      Serial.print(RcurrentSpeed);
      Serial.print("RPID输出: ");
      Serial.println(ROutput);

      Serial.print("左轮转速: ");
      Serial.print(LcurrentSpeed);
      Serial.print("LPID输出: ");
      Serial.println(LOutput);
    }

  // 双稳控制
    bool A_pressed = xboxController.xboxNotif.btnA;  // 获取手柄 A 按键状态
    GunPosition(A_pressed, dt);

    // 在系统里随时使用开关状态 switchState
    if (switchState) {
      // 模式 ON：例如 PID 保持炮塔角度
    } 
    else {
      // 模式 OFF：例如自由控制
    }

}
