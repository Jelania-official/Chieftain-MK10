# USB IMU 姿态可视化验证工具

这个工具用于单独验证车体 MPU6050 的姿态估计效果，不启用蓝牙、不运行整车控制逻辑，也不会动底盘和炮塔电机。

## 固件部分

独立固件真实代码在 `tools/imu_debug_tool/firmware/imu_visualizer_main.cpp`，使用主工程里的车体 IMU 引脚：

- SDA：`Config::I2C_IMU_SDA`
- SCL：`Config::I2C_IMU_SCL`
- MPU6050 地址：`0x68`

编译/烧录这个独立固件：

```powershell
C:\Users\zj\.platformio\penv\Scripts\pio.exe run -e imu_visualizer -t upload
```

正常整车固件仍然使用 `esp32dev` 环境：

```powershell
C:\Users\zj\.platformio\penv\Scripts\pio.exe run -e esp32dev -t upload
```

`platformio.ini` 里已经做了 `build_src_filter`，所以两个入口不会互相冲突。

注意：PlatformIO 默认只从 `src` 目录找 Arduino 入口文件，所以项目里保留了一个很薄的 `src/imu_visualizer_main.cpp`。它只负责 `#include` 这个工具文件夹里的真实固件代码，不写实际逻辑。

## PC 可视化部分

首次使用先安装依赖：

```powershell
python -m pip install -r tools\imu_debug_tool\requirements.txt
```

运行可视化工具：

```powershell
python tools\imu_debug_tool\imu_visualizer.py
```

也可以直接双击 `tools/imu_debug_tool/start_imu_visualizer.cmd`。启动脚本会先检查 Python 依赖，如果缺少 `pyserial`、`PySide6`、`pyqtgraph` 或 `PyOpenGL`，会自动调用 `pip install -r requirements.txt` 安装。

如果你的 Windows 环境能正常处理中文文件名，也可以双击 `启动IMU可视化.bat`；推荐优先用英文名的 `start_imu_visualizer.cmd`，避免 `cmd` 对中文路径/编码解析出问题。

使用流程：

1. ESP32 上电后先保持 IMU 静止，固件会自动校准陀螺仪零偏。
2. 打开 PC 工具，选择 ESP32 对应的 USB 串口。
3. 点击“连接”。
4. 点击“姿态归零”可以把当前姿态作为新的零点。
5. 点击“重新校准”会重新做陀螺仪零偏校准，校准时仍然要保持静止。

## 串口协议

固件与 PC 可视化工具统一使用 115200 波特率，以 50Hz 输出 CSV：

```text
IMU,timeMs,dt,ax,ay,az,gxDeg,gyDeg,gzDeg,qw,qx,qy,qz,roll,pitch,yaw,temp
```

其中：

- `ax/ay/az`：加速度，单位 m/s²
- `gxDeg/gyDeg/gzDeg`：角速度，单位 °/s
- `qw/qx/qy/qz`：姿态四元数
- `roll/pitch/yaw`：欧拉角，单位 °
- `temp`：MPU6050 温度

## 3D 姿态显示

PC 工具左侧包含一个简单 3D 车体模型：

- 车体长轴表示前后方向。
- 两侧黑色块表示履带方向，方便观察 roll。
- 前方短杆表示炮管/车头参考方向，方便观察 yaw。

3D 视图直接使用固件输出的四元数驱动，比只看欧拉角更直观。如果 3D 区域提示缺少 OpenGL 依赖，重新运行依赖安装命令即可：

```powershell
python -m pip install -r tools\imu_debug_tool\requirements.txt
```
