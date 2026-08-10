// PlatformIO 默认从 src 目录寻找 Arduino 入口文件。
// 真实的 AS5600/FOC 调试固件集中放在 tools/as5600_foc_debug_tool/firmware，
// 这里保留一个很薄的入口壳，避免把工具源码散在主工程 src 里。
#include <BluetoothSerial.h> // 让PlatformIO依赖扫描识别ESP32框架内置蓝牙库。
#include "../tools/as5600_foc_debug_tool/firmware/as5600_foc_debug_main.cpp"
