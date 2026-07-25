#pragma once

#include <Arduino.h>

// ==========================================
// 0. 调试控制中心 (Debug Control Center)
// ==========================================
#define GLOBAL_DEBUG 1 

enum DebugChannel { NONE, CHASSIS_ONLY, TURRET_ONLY, IMU_RAW, ALL };
extern DebugChannel currentChannel;

#if GLOBAL_DEBUG
  #define LOG(ch, fmt, ...) if(currentChannel == ch || currentChannel == ALL) Serial.printf(fmt, ##__VA_ARGS__)
  #define LOG_ALWAYS(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define LOG(ch, fmt, ...)
  #define LOG_ALWAYS(fmt, ...)
#endif

