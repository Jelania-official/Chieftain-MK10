#include <Arduino.h>
#include "app/TankRobot.h"

// ==========================================
// Arduino/FreeRTOS 入口
// ==========================================
// main.cpp 只保留程序入口和任务绑定；整车逻辑都在 app/TankRobot 中。
TankRobot robot;
TaskHandle_t FOC_TaskHandle;

// Core 0 高频任务：只负责 yaw 无刷电机的 FOC loopFOC/move。
// 让 FOC 独占一个核心，可以减少主循环里蓝牙、IMU、底盘计算造成的抖动。
void FocTask(void *pvParameters) {
    for (;;) {
        robot.runFOC_Only();
        vTaskDelay(pdMS_TO_TICKS(1)); // 给 idle/watchdog 留时间，炮塔未就绪时也不会空转重启。
    }
}

void setup() {
    robot.setup();

    // 把 FOC 任务固定到 Core 0；Arduino loop 默认运行在 Core 1。
    xTaskCreatePinnedToCore(
        FocTask,
        "FOC_Task",
        8192,
        NULL,
        5,
        &FOC_TaskHandle,
        0
    );
}

void loop() {
    // 处理蓝牙、IMU、底盘、炮塔稳定和电池保护。
    robot.loop_without_FOC();
}
