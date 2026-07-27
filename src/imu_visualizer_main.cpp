// PlatformIO 默认从 src 目录寻找 Arduino 入口文件。
// 真实的 IMU 调试固件集中放在 tools/imu_debug_tool/firmware，
// 这里保留一个很薄的入口壳，避免把工具源码散在主工程 src 里。
#include "../tools/imu_debug_tool/firmware/imu_visualizer_main.cpp"
