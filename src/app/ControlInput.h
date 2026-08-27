#pragma once

// TankRobot 从 Xbox 手柄读取后统一填到这个结构里。
// 这样底盘和炮塔不用直接依赖手柄库，只关心已经归一化后的控制量。
struct ControlInput {
    float triggerL = 0.0f; // LT，0.0~1.0，用作倒车/刹车输入
    float triggerR = 0.0f; // RT，0.0~1.0，用作前进油门输入
    float joyLX = 0.0f;    // 左摇杆 X，-1.0~1.0，用作底盘转向
    float joyRX = 0.0f;    // 右摇杆 X，-1.0~1.0，用作炮塔 yaw 手动输入
    float joyRY = 0.0f;    // 右摇杆 Y，-1.0~1.0，用作炮管 pitch 手动输入
    bool aPressed = false; // A 键，作为炮塔稳定模式开关
    bool bPressed = false; // B 键，锁存整车急停
    bool yPressed = false; // Y 键，解除手柄急停
};
