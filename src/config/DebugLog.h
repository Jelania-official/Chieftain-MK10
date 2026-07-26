#pragma once

#include <Arduino.h>

// ==========================================
// 0. 调试输出控制中心
// ==========================================
// GLOBAL_DEBUG 是编译期开关；设为 0 时 LOG/LOG_ALWAYS 都不会生成串口输出代码。
// currentChannel 是运行期频道选择；改 DebugLog.cpp 里的默认值即可切换观察范围。
#define GLOBAL_DEBUG 1

// 普通 LOG 使用频道过滤：
// - CHASSIS_ONLY：底盘调试
// - TURRET_ONLY：炮塔调试
// - IMU_RAW：IMU 原始/中间量调试
// - ALL：输出所有频道
// - NONE：关闭普通频道日志
enum DebugChannel { NONE, CHASSIS_ONLY, TURRET_ONLY, IMU_RAW, ALL };
extern DebugChannel currentChannel;

#if GLOBAL_DEBUG
  // LOG 受 currentChannel 过滤；LOG_ALWAYS 只受 GLOBAL_DEBUG 控制，适合错误和保护提示。
  #define LOG(ch, fmt, ...) if(currentChannel == ch || currentChannel == ALL) Serial.printf(fmt, ##__VA_ARGS__)
  #define LOG_ALWAYS(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define LOG(ch, fmt, ...)
  #define LOG_ALWAYS(fmt, ...)
#endif
