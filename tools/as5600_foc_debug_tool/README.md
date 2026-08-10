# AS5600 / SimpleFOC / 无刷电机调试工具

这是炮塔 yaw 轴的独立台架工具。它复用整车工程的真实引脚、SimpleFOC 版本、电机极对数和堵转参数，但使用适合 AS5600 差分速度的独立保守 PID 默认值。它不会启动履带、舵机或两颗 MPU6050，也没有拆分或改写 `TankTurret`。

工具由两部分组成：

- ESP32 独立固件：`firmware/as5600_foc_debug_main.cpp`
- PC 图形工具：`as5600_foc_debug_tool.py`

固件同时提供 USB 串口和 ESP32 经典蓝牙 SPP，两条链路使用完全相同的握手、命令和遥测协议。蓝牙名称沿用整车调试模式的 `ChieftainMK10-Debug`，便于复用 Windows 已配对的虚拟 COM 口。

## 主要功能

- 未执行 FOC 对齐时，先以 20 Hz 显示 AS5600 原始计数与绝对角度。
- 手动执行 SimpleFOC 传感器方向和零电角对齐，并显示对齐结果。
- 普通上电阶段不初始化 SimpleFOC PWM/电机对象；只有明确点击“FOC对齐”后才初始化驱动并允许电机产生预期的对齐运动。
- 直接 q 轴电压点动，按钮松开即归零，单次最长 3 秒。
- 速度闭环和位置串级闭环控制，目标速度上限为 25 °/s，位置范围为相对零点 ±90°。
- 在线修改外环 `KP/KD` 和内环 `KP/KI/KD`；参数只保存在 RAM，重启后恢复本工具的保守默认值，不会改写整车 `RobotConfig.h`。
- 调试速度由 100 Hz AS5600 原始角跨零差分并经约 20 ms 低通得到，再按 SimpleFOC 对齐识别的 `sensor_direction` 统一方向；不直接使用高频单计数差分。
- 实时显示位置、位置误差、速度、q 轴电压、动力电压、I²C 错误计数和实际 FOC 循环频率。
- 自动执行 ±15° 位置阶跃，估算上升时间、超调、末端误差和峰值电压。
- 自动执行 ±5/10/15/22.5 °/s 扫速，统计平均速度、误差、标准差和平均电压。
- 将完整遥测导出为 UTF-8 CSV。
- 捕获“炮塔正前”对应的 AS5600 原始角，并生成 `TURRET_FRONT_SENSOR_OFFSET`、`TURRET_SENSOR_SIGN` 配置片段；圆盘同步显示后甲板 ±15°/±10°区域。

商家已经把磁铁与编码器安装成套，本工具不增加 AS5600 `STATUS` 磁场强弱检测；当前 `sensor_ok` 只表示 I²C 原始角读取成功且数据持续更新。

## 调度频率

频率按整车源码对齐：

- `motor.loopFOC()` / `motor.move()`：ESP32 Core 0，每 1 ms 调度一次，界面显示实测频率。
- 串级 PID 与安全判断：Core 1，200 Hz。
- AS5600 独立原始角检查：Core 0，100 Hz。
- PC 遥测：20 Hz。
- PC 心跳与活动目标刷新：50 Hz；连续 300 ms 收不到命令会锁存超时故障。

## 编译与烧录

在项目根目录执行：

```powershell
C:\Users\zj\.platformio\penv\Scripts\platformio.exe run -e as5600_foc_debug -t upload
```

调试完恢复整车固件：

```powershell
C:\Users\zj\.platformio\penv\Scripts\platformio.exe run -e esp32dev -t upload
```

默认 PlatformIO 环境仍是 `esp32dev`。只有显式选择 `as5600_foc_debug` 才会烧录本工具固件。`src/as5600_foc_debug_main.cpp` 只是 PlatformIO 入口壳，实际逻辑集中在本目录，没有把整车文件拆碎。

## PC 工具启动

推荐双击：

```text
start_as5600_foc_debug.cmd
```

启动脚本会检查 Python 3 和依赖，缺少时自动安装。也可以手动执行：

```powershell
python -m pip install -r tools\as5600_foc_debug_tool\requirements.txt
python tools\as5600_foc_debug_tool\as5600_foc_debug_tool.py
```

连接方式：

- USB：选择 ESP32 的 USB COM 口，或点击“自动连接”。打开 USB 串口通常会让 ESP32 复位一次。
- 蓝牙：先在 Windows 中配对 `ChieftainMK10-Debug`，再选择系统创建的“蓝牙链接上的标准串行”COM 口。Windows 可能同时创建传入/传出两个端口，只有能完成工具握手的端口可用，“自动连接”会逐一识别。
- 同一时刻只运行一个 PC 调试工具实例，避免 USB 和蓝牙客户端同时发送控制命令。

## 首次台架流程

1. 让炮塔悬空或拆掉容易碰撞的结构，确认至少能自由转动 ±20°。
2. 建议直流电源先限流到 0.6～0.8 A；保留随手切断 12 V 的能力。
3. 先用 USB 连接 ESP32，再接 12 V 动力电源。工具会自动识别运行本固件的串口。
4. 上电默认只读取 AS5600，所有三相输出保持为零。观察原始角随手动旋转是否连续。
5. 电压上限保持默认 1.0 V，点击“确认安全并执行 FOC 对齐”。对齐期间电机会短暂运动。
6. 先用 0.3～0.5 V 的正、负向点动确认相位与编码器反馈正常；直接电压模式都不能平顺转动时，不要进入速度/位置闭环。
7. 再试 ±2 °/s 速度模式和 ±5°位置模式；裸转子默认使用外环 KP=0.4、内环 KP=0.01，其余 I/D 为零。确认方向、噪声、电流和温升后再逐步增加参数和目标。
8. 最后运行自动阶跃/扫速测试并导出 CSV。自动扫速需要连续转动，必须确认线缆不会缠绕。

不要一开始把电压上限设为 6 V。该电机额定电流 0.5 A、最大电流 2 A，当前硬件没有相电流采样；6 V 只是整车控制上限，不等于可长期持续输出的安全电压。

## 安全逻辑

- Esc、红色“紧急停止”、断开或关闭窗口都会发送停止命令并锁存故障。
- PC 窗口在输出期间失去焦点会立即把输出归零。
- 直接电压点动超过 3 秒会锁存故障。
- AS5600 读取失败/陈旧、动力电压不在 6.0～16.5 V、PC 心跳超过 300 ms 都会归零并锁存故障。
- 高输出、存在运动需求且 500 ms 内机械角移动不足 0.5°时，复用整车参数触发堵转保护。
- 清除故障不会自动恢复输出；需要重新选择模式并启动。

SimpleFOC Mini 的 EN 在当前网表中由 `U3-5` 接 3.3 V，因此 PC 的“停止”是 PWM/q 轴电压归零，不是物理断开驱动板电源。紧急情况仍应切断 12 V。

独立固件已经把 `driver.init()`、`motor.init()` 延后到操作者确认“FOC对齐”时执行，可以消除由普通启动阶段PWM初始化造成的额外跳动。但 EN 被硬件固定为高电平，ESP32上电复位到 `setup()` 开始执行之前，三路PWM引脚仍有一个软件无法控制的短暂窗口；尤其 `GPIO5` 还是ESP32启动相关引脚。因此如果电机在接通12 V或ESP32复位瞬间仍轻跳，完整硬件修复应当把驱动EN改为默认下拉并由GPIO在初始化完成后拉高，或增加可靠的上电延时使能，不能只依赖固件。

## 方位与后甲板说明

FOC 对齐得到的是“电角度换相基准”，工具的“当前位置设为控制零点”得到的是本次测试的相对机械零点，两者都不代表车体正前。后甲板避让使用 AS5600 的绝对原始角，再结合：

```cpp
TURRET_FRONT_SENSOR_OFFSET
TURRET_SENSOR_SIGN
```

换算炮塔相对车体方位。因此正前偏移在装车后捕获一次才有意义；更换电机、编码器、齿轮啮合关系或重新安装炮塔后需要重新确认。

## 串口协议摘要

115200 波特率，ASCII 行协议，版本为 1。PC 用 `HELLO,1` 识别固件。主要命令为 `ALIGN`、`MODE`、`SET`、`RUN`、`LIMIT`、`PID`、`ZERO`、`CLEAR`、`STOP` 和 `HB`。

遥测格式：

```text
TEL,time_ms,state,mode,foc_ready,sensor_ok,battery_v,raw_count,raw_deg,
    position_deg,target_position_deg,target_velocity_dps,actual_velocity_dps,
    voltage_v,voltage_limit_v,position_error_deg,stall_latched,i2c_errors,foc_hz
```
