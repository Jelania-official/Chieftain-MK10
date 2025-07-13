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


// PWM输出引脚
#define pwmPin 33

/***************** 编码器参数 *****************/
#define R1 18    // 右轮编码器引脚A
#define R2 19    // 右轮编码器引脚B
#define L1 18    // 右轮编码器引脚A
#define L2 19    // 右轮编码器引脚B
volatile long Rcounter = 0; // 右轮脉冲计数
unsigned long lastEncoderTime = 0;
float currentSpeed = 0;      // 当前转速(RPM)
const int encoderPPR = 7;    // 编码器每转脉冲数
const int gearRatio = 59;   // 减速比

void IRAM_ATTR encoderISR();


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
void IRAM_ATTR encoderISR() {
  if (digitalRead(R2) == HIGH) {
    Rcounter++;  // 顺时针
  } else {
    Rcounter--;  // 逆时针
  }
}



// 获取真实转速(RPM)
float getSpeed() {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastEncoderTime;
    
    if(deltaTime >= 100) {  // 每100ms计算一次转速
        currentSpeed = (Rcounter / (float)encoderPPR / gearRatio) * (60000.0 / deltaTime);
        Rcounter = 0;
        lastEncoderTime = currentTime;
    }
    
    return currentSpeed;
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

    // 输出限幅（0~255，对应PWM范围）
    if (output > 255) output = 255;
    if (output < 0) output = 0;

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



void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Client");
  xboxController.begin();

    // 初始化编码器
   pinMode(R1, INPUT_PULLUP);
   pinMode(R2, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(R1), encoderISR, RISING);
    
    // 初始化PWM引脚
   pinMode(pwmPin, OUTPUT);
    
}



void loop()
{
 //  管理手柄通信和按键数据
 handleXboxController();



 //模拟油门
  simulateThrottle();



 //PID 
  float currentSpeed = getSpeed();
 double pwmOutput = calculatePID(v, currentSpeed);
 analogWrite(pwmPin, pwmOutput);


 // 调试信息
 Serial.print("转速: ");
 Serial.print(currentSpeed);
 Serial.print(" RPM | PID输出: ");
 Serial.println(pwmOutput);



  delay(50);  // 每50ms更新一次
}