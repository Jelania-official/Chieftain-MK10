#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Ticker.h>
#define PI 3.1415926


// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("28:ea:0b:d9:0b:9f");


//速度曲线
float v = 0.0;
unsigned long prevTime = 0;
const float vmax = 13.3; // 主战坦克极限速度
const float brakeCoeff = 1.5;   // 刹车强度系数（建议在 1.2 ~ 2.0）


// PID参数
const double Kp = 2.0;
const double Ki = 0.1;
const double Kd = 0.5;

// 定义PID变量
double currentError = 0;
double integral = 0;
double derivative = 0;
double previousError = 0;


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


// PWM 通道、频率、分辨率
#define CHANNEL_A 0  // 控制 PWMA（右轮）
#define CHANNEL_B 1  // 控制 PWMB（左轮）
#define PWM_FREQ 10000       // 10kHz，适合电机
#define PWM_RESOLUTION 8     // 8位，0~255

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
               String(v) + "," +
               String(xboxController.xboxNotif.trigRT) + "\n";
  return str;
};


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


// 获取真实转速(RPM)
float RgetSpeed() {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - RlastEncoderTime;
    
    if(deltaTime >= 100) {  // 每100ms计算一次转速
        RcurrentSpeed = (Rcounter / (float)encoderPPR / gearRatio) * (60000.0 / deltaTime);
        Rcounter = 0;
        RlastEncoderTime = currentTime;
    }
    
    return RcurrentSpeed;
}

float LgetSpeed() {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - LlastEncoderTime;
    
    if(deltaTime >= 100) {  // 每100ms计算一次转速
        LcurrentSpeed = (Lcounter / (float)encoderPPR / gearRatio) * (60000.0 / deltaTime);
        Lcounter = 0;
        LlastEncoderTime = currentTime;
    }
    
    return LcurrentSpeed;
}



//PID计算
double calculatePID(double targetSpeed, double actualSpeed) {
    currentError = targetSpeed - actualSpeed;

    integral += currentError;
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;

    double derivative = currentError - previousError;
    previousError = currentError;

    double output = Kp * currentError + Ki * integral + Kd * derivative;

    // 输出限幅（-255~255，对应PWM范围）
    if (output > 255) output = 255;
    if (output < -255) output = -255;

    return output;
}


//模拟油门
void simulateThrottle() {
  if (!xboxController.isConnected()) return;

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  if (dt <= 0) dt = 0.05;
  prevTime = currentTime;

  // 输入控制量
  float k = xboxController.xboxNotif.trigLT / 1023.0;   // 推进
  float brake = xboxController.xboxNotif.trigRT / 1023.0; // 刹车

  // 推力为非线性：油门深度越大，推力非线性上升
  float k_adj = pow(k, 2.0);

  // 低速滚阻：速度越低，滚阻越大（模拟履带 + 慢起步）
  float rollingResistance = (v < 4.0) ? 0.5 : 0.338;

  // 加速度公式（推力 - 阻力 - 滚阻 - 刹车）
  float a = 1.4 * k_adj - 0.006 * v * v - rollingResistance - brakeCoeff * brake;

  // 更新速度
  v = v + a * dt;

  // 限制最低速度为0
  if (v < 0) v = 0;

  // 限制最高速度
  if (v > vmax) v = vmax;

  // 打印输出
  String output = "k=" + String(k, 3) +
                  ", brake=" + String(brake, 3) +
                  ", a=" + String(a, 3) +
                  ", v=" + String(v, 3) + " m/s";
  Serial.println(output);
}


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

    // 如果失败次数太多，自动重启
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }
}


//电机正反转驱动
void RdriveMotor(double speed) {
  // 启动 TB6612（必须 STBY=1 才能工作）
  digitalWrite(STBY, HIGH);

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
  // 启动 TB6612（必须 STBY=1 才能工作）
  digitalWrite(STBY, HIGH);

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


 //模拟油门
  simulateThrottle();



 //PID 
  float RcurrentSpeed = RgetSpeed();
  double RpwmOutput = calculatePID(Rv, RcurrentSpeed);
  RdriveMotor(RpwmOutput);

  float LcurrentSpeed = LgetSpeed();
  double LpwmOutput = calculatePID(Lv, LcurrentSpeed);
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

