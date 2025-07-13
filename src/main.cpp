#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Ticker.h>
#define PI 3.1415926


// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("28:ea:0b:d9:0b:9f");


//速度曲线
float Rv = 0.0;
float Lv = 0.0;

unsigned long prevTime = 0;
const float vmax = 14.5; // 主战坦克极限速度
const float brakeCoeff = 1.5;   // 刹车强度系数（建议在 1.2 ~ 2.0）


// PID参数
const double Kp = 2.0;
const double Ki = 0.1;
const double Kd = 0.5;


// 行走部分驱动引脚（输出）
#define AIN1 25
#define AIN2 26
#define PWMA 33
#define STBY 32  // 启动控制引脚

#define BIN1 25
#define BIN2 26
#define PWMB 33


/***************** 编码器参数 *****************/
#define R1 18    // 右轮编码器引脚A
#define R2 19    // 右轮编码器引脚B
volatile long Rcounter = 0; // 右轮脉冲计数
float RcurrentSpeed = 0;      // 当前转速(RPM)

#define L1 18    // 右轮编码器引脚A
#define L2 19    // 右轮编码器引脚B
volatile long Lcounter = 0; // 右轮脉冲计数
float LcurrentSpeed = 0;      // 当前转速(RPM)

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
const float rtThreshold = 0.5;      // 右扳机按下阈值（0~1之间）
const unsigned long rtHoldDuration = 2000; // 持续按下时间，单位ms（2秒）
const float reverseVmax = 6.0;  // 倒车限速6 m/s


struct PIDController {
  double previousError = 0.0;
  double integral = 0.0;
};
PIDController pidR;
PIDController pidL;


/*********************************************/


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
               String(Rv) + "," +
               String(xboxController.xboxNotif.trigRT) + "\n";
  return str;
};


// 手柄通信维护
void handleXboxController() {
 
  xboxController.onLoop();

  // 判断是否连接成功
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {
      Serial.print(xbox_string());

      // 可选功能：振动演示
      // demoVibration();
      // demoVibration_2();
    }
  } else {
    Serial.println("not connected");
    RdriveMotor(0);//停止电机
    LdriveMotor(0);

    // 如果失败次数太多，自动重启
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }
}


// 编码器中断服务函数
void IRAM_ATTR RencoderISR() {
  if (digitalRead(R2) == HIGH) {
    Rcounter++;  // 顺时针
  } else {
    Rcounter--;  // 逆时针
  }
}

void IRAM_ATTR LencoderISR() {
  if (digitalRead(L2) == HIGH) {
    Lcounter++;  // 顺时针
  } else {
    Lcounter--;  // 逆时针
  }
}


// 获取真实转速(m/s)
float RgetSpeed() {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - RlastEncoderTime;
    
    if(deltaTime >= 100) {  // 每100ms计算一次转速
        RcurrentSpeed = (Rcounter / (float)encoderPPR / gearRatio) * (60000.0 / deltaTime);
        Rcounter = 0;
        RlastEncoderTime = currentTime;
    }
    float RlinearSpeed = RcurrentSpeed * wheelRPMToSpeed;
    return RlinearSpeed;
}

float LgetSpeed() {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - LlastEncoderTime;
    
    if(deltaTime >= 100) {  // 每100ms计算一次转速
        LcurrentSpeed = (Lcounter / (float)encoderPPR / gearRatio) * (60000.0 / deltaTime);
        Lcounter = 0;
        LlastEncoderTime = currentTime;
    }
    float LlinearSpeed = LcurrentSpeed * wheelRPMToSpeed;
    return LlinearSpeed;
}


//PID计算
double calculatePID(PIDController& pid, double targetSpeed, double actualSpeed) {
  double currentError = targetSpeed - actualSpeed;
  pid.integral += currentError;

  // 限制积分项
  if (pid.integral > 1000) pid.integral = 1000;
  if (pid.integral < -1000) pid.integral = -1000;

  double derivative = currentError - pid.previousError;
  pid.previousError = currentError;

  double output = Kp * currentError + Ki * pid.integral + Kd * derivative;

  // 限幅
  if (output > 255) output = 255;
  if (output < -255) output = -255;

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
  bool isStopped = (Rv < 0.1);

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
    // 右扳机松开，清空计时，退出倒车模式
    rtPressedStartTime = 0;
    reverseMode = false;
  }

  // 根据是否倒车模式计算加速度和速度
  float k_adj = pow(k, 2.0);
  float rollingResistance = (abs(Rv) < 3.0) ? 0.5 : 0.338;// 低速滚阻：速度越低，滚阻越大（模拟履带 + 慢起步）
  float a = 0.0;

  if (!reverseMode) {
    // 正常前进模式
    a = 1.4 * k_adj - 0.006 * Rv * Rv - rollingResistance - brakeCoeff * brake;// 加速度公式（推力 - 阻力 - 滚阻 - 刹车）
    if (Rv < 0.5) a *= 0.5;  // 起步迟钝
    Rv = Rv + a * dt;
    if (Rv < 0) Rv = 0;
    if (Rv > vmax) Rv = vmax;
  } else {
    // 倒车模式：加速度用同样公式，但油门变成倒车时的刹车，速度取负值表示倒车
    // 注意此时k（油门）不再代表推力，而是“倒车时的刹车”，即用k替代brake
    // 这里做个简单处理：用k代替刹车，推力为负方向
    a = 1.4 * pow(brake, 2.0) - 0.006 * Rv * Rv - rollingResistance - brakeCoeff * k; 
    if (Rv > -reverseVmax) {
      Rv = Rv + a * dt;
      if (Rv < -reverseVmax) Rv = -reverseVmax;
    }
    if (Rv > 0) Rv = 0; // 倒车时速度不能正
  }

  // 输出调试信息
  String output = "ReverseMode=" + String(reverseMode ? "ON" : "OFF") +
                  ", LT=" + String(k, 3) +
                  ", RT=" + String(brake, 3) +
                  ", a=" + String(a, 3) +
                  ", Rv=" + String(Rv, 3) + " m/s";
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
  bool isSpinMode = abs(Rv) < 0.1 && abs(yaw) > 0.2;

  if (isSpinMode) {
  // 原地转向逻辑：摇杆偏移量决定转速大小
  float k = yaw;           // 作为转速模拟的油门
  float rollingResistance = 0.5;
  float a = 1.4 * k - rollingResistance;

  static float spinV = 0.0;

  spinV += a * dt;
  if (spinV > 1.22) spinV = 1.22;

  // 根据 yaw 的符号确定左右履带方向
  float dir = copysign(1.0, yaw);  // +1 或 -1

  Lv = -spinV * dir;
  Rv =  spinV * dir;
  }else {
    // 正常差速转向：两侧履带速度按 yaw 做调整
    float turnRatio = constrain(yaw, -1.0, 1.0);
    float turnFactor = 0.5; // 可调差速比例（建议在 0.4~0.7）

    Lv = Rv * (1.0 - turnFactor * turnRatio); // 左履带
    Rv = Rv * (1.0 + turnFactor * turnRatio); // 右履带

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

  if (speed > 0) {
    digitalWrite(BIN1, LOW);   // 正转 BIN1=0 BIN2=1
    digitalWrite(BIN2, HIGH);
    ledcWrite(CHANNEL_B, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, HIGH);  // 反转 BIN1=1 BIN2=0
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_B, -speed);
  } else {
    // 停止
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_B, 0);
  }
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
   pinMode(STBY, OUTPUT);

   pinMode(BIN1, OUTPUT);
   pinMode(BIN2, OUTPUT);

   digitalWrite(STBY, HIGH); // 使能芯片

   // PWM 初始化
   ledcSetup(CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
   ledcAttachPin(PWMA, CHANNEL_A);

   ledcSetup(CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
   ledcAttachPin(PWMB, CHANNEL_B);

    
}



void loop()
{
 //  管理手柄通信和按键数据
  handleXboxController();


  //计算时间
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  if (dt <= 0) dt = 0.05;
  prevTime = currentTime;


 //模拟油门
  simulateThrottle(dt);


 // 更新左右轮速度目标（转向）
  updateWheelSpeed(dt);


 //PID 
  float RcurrentSpeed = RgetSpeed();
  double RpwmOutput = calculatePID(pidR, Rv, RcurrentSpeed);
  RdriveMotor(RpwmOutput);

  float LcurrentSpeed = LgetSpeed();
  double LpwmOutput = calculatePID(pidL, Lv, LcurrentSpeed);
  LdriveMotor(LpwmOutput);


 // 调试信息
 Serial.print("右轮转速: ");
 Serial.print(RcurrentSpeed);
 Serial.print(" RPM | RPID输出: ");
 Serial.println(RpwmOutput);

 Serial.print("左轮转速: ");
 Serial.print(LcurrentSpeed);
 Serial.print(" RPM | LPID输出: ");
 Serial.println(LpwmOutput);


  delay(50);  // 每50ms更新一次
}
