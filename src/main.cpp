#include <Arduino.h>
#include "app/TankRobot.h"

// ==========================================
// 全局实例与 FreeRTOS 双核入口
// ==========================================
TankRobot robot;
TaskHandle_t FOC_TaskHandle;
// Core 0 的任务函数：只负责无刷电机的 FOC 算法
void FocTask(void *pvParameters) {
    for (;;) {
        robot.runFOC_Only(); 
        vTaskDelay(pdMS_TO_TICKS(1)); // 给 CPU0 idle/watchdog 留出时间，炮塔未就绪时也不会空转重启
    }
}

// 默认在 Core 1 上运行的 setup
void setup() { 
    robot.setup(); 
    
    // 将 FOC 任务绑定到 Core 0
    xTaskCreatePinnedToCore(
        FocTask,       // 任务函数
        "FOC_Task",    // 任务名称
        8192,          // 堆栈大小 (分配8K给FOC防溢出)
        NULL,          // 任务参数
        5,             // 优先级 (数字越大优先级越高，设为5确保FOC优先执行)
        &FOC_TaskHandle, // 任务句柄
        0              // 核心编号：Core 0
    );
}

// 默认在 Core 1 上运行的 loop
void loop() { 
    // 处理底盘、IMU、蓝牙、PID和舵机
    robot.loop_without_FOC(); 
}
