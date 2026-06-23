# ICM45686 EIS App - Demo 与压测说明文档

本文档用于记录 `icm45686-eis-app` 中两个应用层 Demo 的编译、运行方法、测试目标、日志解读和性能评估方法。当前工程基于已经调通的 `/dev/icm45686` 字符设备方案，不依赖 IIO 子系统；Demo 的目标是验证 **IMU 数据读取、应用层环形缓冲、时间戳窗口查询、陀螺仪积分和 EIS 像素偏移计算** 这条链路是否稳定。

---

## 目录

1. [环境准备与通用编译要求](#环境准备与通用编译要求)
2. [Demo 01: ICM45686 基础 IMU 数据读取测试](#demo-01-icm45686-基础-imu-数据读取测试)
3. [Demo 02: EIS 防抖偏移计算与 CPU 压测](#demo-02-eis-防抖偏移计算与-cpu-压测)
4. [性能指标排查指南](#性能指标排查指南)
5. [常见异常与定位方法](#常见异常与定位方法)
6. [新增 Demo 文档模板](#新增-demo-文档模板)

---

## 环境准备与通用编译要求

运行 Demo 前需要满足以下条件：

* **硬件平台**：RK3588 / RK3588 ELF2 等 Rockchip ARM64 平台。
* **IMU 模块**：ICM45686，已通过 SPI 接入，基础数据读取正常。
* **内核驱动**：已加载 `inv_imu_driver.ko` 和 `icm45686_spi.ko`。
* **设备节点**：`/dev/icm45686` 存在。
* **编译工具链**：`aarch64-buildroot-linux-gnu-gcc/g++`。
* **C++ 标准**：C++14，需支持 `<thread>`、`<atomic>`、`<chrono>`、`<mutex>` 等。

运行前检查：

```bash
ls -la /dev/icm45686
lsmod | grep icm45686
dmesg | grep -iE "icm|45686|imu|spi4|who"
```

正常驱动日志应包含：

```bash
ICM45686 initialized successfully
icm45686_spi spi4.0: ICM45686 SPI driver loaded successfully
```

### 编译

项目根目录提供一键编译脚本：

```bash
cd icm45686-eis-app
./build.sh
```

编译完成后，可执行文件安装到：

```text
install/icm45686_app
install/icm45686_eis_demo
```

如果需要手动清理并重新编译：

```bash
rm -rf build install
./build.sh
```

### 部署到板端

```bash
scp install/icm45686_app root@192.168.0.232:/usr/bin/
scp install/icm45686_eis_demo root@192.168.0.232:/usr/bin/
```

---

## Demo 01: ICM45686 基础 IMU 数据读取测试

**源文件**：`src/icm45686_app.c`

### 1. 演示目标

验证 `/dev/icm45686` 字符设备是否能够被用户态正常访问，并持续读取：

* 三轴加速度，单位 `m/s²`；
* 三轴陀螺仪，单位 `rad/s`；
* 温度，单位 `℃`；
* AHRS 姿态角 `Yaw / Pitch / Roll`。

该 Demo 主要用于确认内核驱动、SPI 通信、寄存器配置、字节序解析和基础物理量换算是否正确。

### 2. 运行方法

```bash
/usr/bin/icm45686_app
```

### 3. 预期输出

```text
ICM45686 IMU Application
========================
Device opened successfully
AHRS initialized
Sensor configured

Data output format:
Count | Accel (m/s²) XYZ | Gyro (rad/s) XYZ | Temp (°C) | Yaw Pitch Roll (°)
------|-------------------|-------------------|-----------|------------------
    0 |  -9.71  -0.25   1.56 |   0.00   0.02  -0.00 |   29.06 |   0.00  -0.00   0.00
```

### 4. 数据判断标准

静置状态下不要只看某一个轴是否等于 `9.8`，因为模块放置方向不同，重力会分布到不同轴上。应看三轴合成模长：

```text
|a| = sqrt(ax² + ay² + az²) ≈ 9.8 m/s²
```

正常表现：

* 加速度三轴数据稳定，模长接近 `9.8 m/s²`；
* 陀螺仪三轴接近 `0 rad/s`；
* 温度稳定，不应大幅跳变；
* Pitch / Roll 能够随姿态变化，Yaw 因无磁力计会有长期漂移。

### 5. 常见异常

| 现象 | 可能原因 | 处理方法 |
|---|---|---|
| `/dev/icm45686` 不存在 | 驱动 probe 失败 | 查看 `dmesg` 中 WHO_AM_I / reset / SPI 日志 |
| 加速度全为 0 | 寄存器地址或传感器未使能 | 检查 `PWR_MGMT0`、`ACCEL_CONFIG0`、`GYRO_CONFIG0` |
| 温度在 25/41/57/73℃ 跳变 | 字节序错误 | ICM45686 默认 Little Endian，应按低字节在前解析 |
| Yaw 漂移 | 六轴 IMU 无绝对航向参考 | 属于正常现象，EIS 主要使用短时间 gyro 积分 |

---

## Demo 02: EIS 防抖偏移计算与 CPU 压测

**源文件**：`src/eis_demo.cpp`，配合 `src/imu_eis.cpp` / `include/imu_eis.hpp`

### 1. 演示目标

验证当前应用层 EIS 防抖接口是否能稳定完成：

* 后台线程按固定频率读取 IMU；
* 每条 IMU 数据打 `CLOCK_MONOTONIC` 时间戳；
* 数据写入应用层 `ImuRingBuffer`；
* 模拟 30 FPS 视频帧时间戳；
* 根据目标帧时间戳查询前后窗口内 IMU 样本；
* 对陀螺仪角速度进行积分；
* 将角度变化换算为 `offsetX / offsetY` 像素补偿量；
* 统计成功率、耗时和 CPU 占用。

### 2. 运行方法

默认运行：

```bash
/usr/bin/icm45686_eis_demo
```

推荐显式参数运行：

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20

# 工业震动起步配置：200Hz IMU、30FPS相机、±500DPS、±4G、offset限幅120px、启用平滑和2秒零偏标定
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 200 30 20 1 1 120 0 -1 1 0 0.25 2000
```

参数含义：

```text
/dev/icm45686  设备节点
30             运行 30 秒
1200 1200      相机焦距 focalX / focalY，单位 pixel
100            IMU 读取频率，单位 Hz
30             模拟视频帧率，单位 FPS
20             EIS 时间窗口半径，单位 ms

可选扩展参数：
gyroRange      0/1/2/3，对应 ±250/±500/±1000/±2000DPS
accelRange     0/1/2/3，对应 ±2G/±4G/±8G/±16G
maxOffset      最大像素补偿量，需要小于视觉链路裁剪余量
timeOffsetMs   相机帧时间戳与IMU时间戳的修正偏移，可为负
signX signY    offset输出方向符号，用于适配安装方向
swapXY         是否交换X/Y轴映射，0为不交换，1为交换
smoothingAlpha 0表示关闭平滑，0.2~0.5表示启用一阶低通平滑
biasCalibMs    0表示关闭零偏标定，例如2000表示启动时静置标定2秒
```

> 对 100Hz IMU，采样间隔约为 10ms。`halfWindowMs=20` 表示总窗口 40ms，通常可以取到 3~5 条 IMU 样本，足够做陀螺仪积分。不要使用 `5ms` 作为默认窗口，否则窗口内通常只有 1 条样本，EIS 计算会失败。

### 3. 线程架构说明

```text
Main Thread
  │
  ├── 打开 /dev/icm45686
  ├── 设置 accel / gyro 量程
  ├── 启动 Icm45686Reader 后台线程
  │      └── readLoop(): read/ioctl 读取 IMU → 打时间戳 → push 到 ImuRingBuffer
  │
  └── 模拟 30FPS 帧循环
         ├── targetTimestamp = now - halfWindowMs
         ├── EisStabilizer::calculate_eis_offset()
         ├── 统计 success / offset / cost / CPU
         └── 每秒打印一次心跳日志
```

### 4. 预期终端输出

```text
========================================
ICM45686 EIS Demo
  Device       : /dev/icm45686
  Runtime      : 30 s
  Sample Rate  : 100.00 Hz
  Frame Rate   : 30.00 Hz
  Focal        : 1200.00 / 1200.00 pixel
  Half Window  : 20 ms
  Window Width : 40 ms
========================================
IMU reader started. Waiting for ring buffer warm-up...
[EIS Demo] frames=31 success=31 failed_eis=0 success_rate=100.00% buffered=130 total_imu=130 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.020 ms max_cost=0.071 ms cpu=3.00%
           latest_imu: accel=(-9.71 -0.25 1.56 | norm=9.84) gyro=(0.0070 0.0240 -0.0020 | norm=0.0251) temp=29.06 used_samples=4
```



### 5. 字段解释

| 字段 | 含义 | 正常标准 |
|---|---|---|
| `frames` | 已模拟处理的视频帧数量 | 30 秒约 900 帧 |
| `success` | EIS offset 计算成功帧数 | 应接近 frames |
| `failed_eis` | EIS 计算失败帧数 | 应为 0 或极少 |
| `success_rate` | EIS 计算成功率 | 推荐接近 100% |
| `buffered` | ring buffer 当前缓存 IMU 样本数 | 稳定增长到容量上限 |
| `total_imu` | 总 IMU 读取条数 | 30 秒约 3000 条 |
| `failed_imu` | IMU 读取失败次数 | 应为 0 |
| `latest_offset` | 最新一帧像素补偿量 | 静止时接近 0，转动时应变化 |
| `max_abs_offset` | 测试期间最大像素补偿量 | 动态测试时应大于 0 |
| `nonzero` | 出现非零 offset 的帧数 | 动态测试时应增加 |
| `avg_cost` | 平均单帧 EIS 计算耗时 | 通常小于 1ms |
| `max_cost` | 最大单帧计算耗时 | 通常小于 1ms |
| `cpu` | 当前进程 CPU 占用 | 实测约 2%~7% |
| `used_samples` | 本次积分使用的 IMU 样本数 | 100Hz + 20ms 半窗口通常约 4 条 |

### 6. 静态测试方法

让模块保持静止，运行：

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20
```

静态测试成功标准：

```text
success_rate ≈ 100%
failed_eis = 0
used_samples ≈ 3~5
gyro norm 接近 0
accel norm 接近 9.8~10.0
latest_offset ≈ (0,0)
max_abs_offset ≈ (0,0)
```

静止时 `offset=(0,0)` 是正确结果，因为没有角运动，不需要像素补偿。

### 7. 动态防抖功能测试方法

运行 Demo 后，轻轻绕 X/Y 轴转动或手动模拟抖动 ICM45686 模块。

动态测试成功标准：

```text
gyro norm 明显变大
latest_offset 从 (0,0) 变成非零
max_abs_offset 逐步增大
nonzero 持续增加
success_rate 仍接近 100%
```

如果动态转动时 `gyro norm` 增大但 `offset` 仍然一直为 0，可能原因是：

1. 焦距 `focalX/focalY` 设置偏小；
2. 转动幅度太小，积分角度低于 0.5 像素后被取整为 0；
3. 轴向映射或符号方向需要按实际安装方式调整；
4. 陀螺仪单位或换算比例仍需复核。

### 8. 实测基准数据

当前静态测试结果：

```text
root@elf2-buildroot:~# /usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20
========================================
ICM45686 EIS Demo
  Device       : /dev/icm45686
  Runtime      : 30 s
  Sample Rate  : 100.00 Hz
  Frame Rate   : 30.00 Hz
  Focal        : 1200.00 / 1200.00 pixel
  Half Window  : 20 ms
  Window Width : 40 ms
========================================
Test method:
  - Keep IMU static first: success should increase, offset should be near zero.
  - Then gently rotate/shake IMU: latest_offset and max_abs_offset should change.
========================================
IMU reader started. Waiting for ring buffer warm-up...
[EIS Demo] frames=31 success=31 failed_eis=0 success_rate=100.00% buffered=158 total_imu=158 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.014 ms max_cost=0.071 ms cpu=5.00%
           latest_imu: accel=(-1.09 -0.37 9.98 | norm=10.04) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.37 used_samples=4
[EIS Demo] frames=61 success=61 failed_eis=0 success_rate=100.00% buffered=258 total_imu=258 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.014 ms max_cost=0.071 ms cpu=5.00%
           latest_imu: accel=(-1.09 -0.37 9.99 | norm=10.05) gyro=(0.0020 -0.0010 0.0000 | norm=0.0022) temp=27.56 used_samples=4
[EIS Demo] frames=92 success=92 failed_eis=0 success_rate=100.00% buffered=362 total_imu=362 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.015 ms max_cost=0.071 ms cpu=4.84%
           latest_imu: accel=(-1.08 -0.37 9.97 | norm=10.04) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.56 used_samples=4
[EIS Demo] frames=122 success=122 failed_eis=0 success_rate=100.00% buffered=462 total_imu=462 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.016 ms max_cost=0.071 ms cpu=7.00%
           latest_imu: accel=(-1.09 -0.38 9.98 | norm=10.05) gyro=(0.0020 -0.0020 0.0000 | norm=0.0028) temp=27.43 used_samples=4
[EIS Demo] frames=153 success=153 failed_eis=0 success_rate=100.00% buffered=565 total_imu=565 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.017 ms max_cost=0.071 ms cpu=5.81%
           latest_imu: accel=(-1.08 -0.37 9.98 | norm=10.04) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.56 used_samples=4
[EIS Demo] frames=184 success=184 failed_eis=0 success_rate=100.00% buffered=668 total_imu=668 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.018 ms max_cost=0.071 ms cpu=4.84%
           latest_imu: accel=(-1.09 -0.37 9.98 | norm=10.04) gyro=(0.0020 -0.0010 0.0000 | norm=0.0022) temp=27.50 used_samples=4
[EIS Demo] frames=214 success=214 failed_eis=0 success_rate=100.00% buffered=768 total_imu=768 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.019 ms max_cost=0.071 ms cpu=3.00%
           latest_imu: accel=(-1.08 -0.37 9.97 | norm=10.04) gyro=(0.0020 -0.0010 0.0000 | norm=0.0022) temp=27.62 used_samples=4
[EIS Demo] frames=245 success=245 failed_eis=0 success_rate=100.00% buffered=872 total_imu=872 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.020 ms max_cost=0.071 ms cpu=3.87%
           latest_imu: accel=(-1.09 -0.37 9.98 | norm=10.04) gyro=(0.0020 -0.0010 0.0000 | norm=0.0022) temp=27.56 used_samples=4
[EIS Demo] frames=275 success=275 failed_eis=0 success_rate=100.00% buffered=972 total_imu=972 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.021 ms max_cost=0.071 ms cpu=4.00%
           latest_imu: accel=(-1.09 -0.37 9.97 | norm=10.04) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.50 used_samples=4
[EIS Demo] frames=305 success=305 failed_eis=0 success_rate=100.00% buffered=1072 total_imu=1072 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.021 ms max_cost=0.071 ms cpu=4.00%
           latest_imu: accel=(-1.09 -0.37 9.98 | norm=10.05) gyro=(0.0030 -0.0020 0.0000 | norm=0.0036) temp=27.56 used_samples=4
[EIS Demo] frames=336 success=336 failed_eis=0 success_rate=100.00% buffered=1175 total_imu=1175 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.022 ms max_cost=0.071 ms cpu=5.81%
           latest_imu: accel=(-1.08 -0.37 9.98 | norm=10.05) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.62 used_samples=4
[EIS Demo] frames=366 success=366 failed_eis=0 success_rate=100.00% buffered=1275 total_imu=1275 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.023 ms max_cost=0.071 ms cpu=5.00%
           latest_imu: accel=(-1.08 -0.37 9.98 | norm=10.05) gyro=(0.0020 0.0000 0.0000 | norm=0.0020) temp=27.25 used_samples=4
[EIS Demo] frames=396 success=396 failed_eis=0 success_rate=100.00% buffered=1375 total_imu=1375 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.024 ms max_cost=0.071 ms cpu=7.00%
           latest_imu: accel=(-1.09 -0.36 9.98 | norm=10.05) gyro=(0.0020 -0.0010 0.0010 | norm=0.0024) temp=27.56 used_samples=4
[EIS Demo] frames=427 success=427 failed_eis=0 success_rate=100.00% buffered=1478 total_imu=1478 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.025 ms max_cost=0.071 ms cpu=5.81%
           latest_imu: accel=(-1.08 -0.37 9.98 | norm=10.04) gyro=(0.0020 -0.0010 0.0000 | norm=0.0022) temp=27.56 used_samples=4
[EIS Demo] frames=458 success=458 failed_eis=0 success_rate=100.00% buffered=1582 total_imu=1582 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.026 ms max_cost=0.071 ms cpu=5.81%
           latest_imu: accel=(-1.09 -0.37 9.97 | norm=10.04) gyro=(0.0020 0.0000 0.0000 | norm=0.0020) temp=27.25 used_samples=4
[EIS Demo] frames=489 success=489 failed_eis=0 success_rate=100.00% buffered=1685 total_imu=1685 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.027 ms max_cost=0.071 ms cpu=6.78%
           latest_imu: accel=(-1.09 -0.36 9.97 | norm=10.04) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.56 used_samples=4
[EIS Demo] frames=519 success=519 failed_eis=0 success_rate=100.00% buffered=1785 total_imu=1785 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.028 ms max_cost=0.071 ms cpu=6.00%
           latest_imu: accel=(-1.07 -0.35 10.00 | norm=10.06) gyro=(0.0010 -0.0020 0.0000 | norm=0.0022) temp=27.37 used_samples=4
[EIS Demo] frames=549 success=549 failed_eis=0 success_rate=100.00% buffered=1885 total_imu=1885 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.029 ms max_cost=0.071 ms cpu=6.00%
           latest_imu: accel=(-1.09 -0.37 9.98 | norm=10.04) gyro=(0.0050 -0.0010 0.0000 | norm=0.0051) temp=27.56 used_samples=4
[EIS Demo] frames=580 success=580 failed_eis=0 success_rate=100.00% buffered=1988 total_imu=1988 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.030 ms max_cost=0.071 ms cpu=4.84%
           latest_imu: accel=(-1.09 -0.36 9.98 | norm=10.04) gyro=(0.0000 0.0000 0.0000 | norm=0.0000) temp=27.68 used_samples=4
[EIS Demo] frames=611 success=611 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2092 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.031 ms max_cost=0.071 ms cpu=5.81%
           latest_imu: accel=(-1.08 -0.37 9.94 | norm=10.00) gyro=(0.0000 0.0000 0.0000 | norm=0.0000) temp=27.56 used_samples=4
[EIS Demo] frames=641 success=641 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2192 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.032 ms max_cost=0.071 ms cpu=6.00%
           latest_imu: accel=(-1.09 -0.36 9.98 | norm=10.05) gyro=(0.0000 0.0000 0.0000 | norm=0.0000) temp=27.50 used_samples=4
[EIS Demo] frames=672 success=672 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2295 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.032 ms max_cost=0.071 ms cpu=5.81%
           latest_imu: accel=(-1.09 -0.37 9.96 | norm=10.03) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.50 used_samples=4
[EIS Demo] frames=702 success=702 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2395 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.033 ms max_cost=0.071 ms cpu=4.00%
           latest_imu: accel=(-1.09 -0.36 10.02 | norm=10.08) gyro=(0.0020 0.0000 0.0000 | norm=0.0020) temp=27.50 used_samples=4
[EIS Demo] frames=733 success=733 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2498 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.034 ms max_cost=0.071 ms cpu=3.87%
           latest_imu: accel=(-1.10 -0.38 9.89 | norm=9.96) gyro=(0.0060 -0.0010 0.0000 | norm=0.0061) temp=27.43 used_samples=4
[EIS Demo] frames=763 success=763 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2598 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.034 ms max_cost=0.071 ms cpu=4.00%
           latest_imu: accel=(-1.09 -0.37 9.98 | norm=10.04) gyro=(0.0010 -0.0010 0.0000 | norm=0.0014) temp=27.37 used_samples=4
[EIS Demo] frames=794 success=794 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2702 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.035 ms max_cost=0.071 ms cpu=4.84%
           latest_imu: accel=(-1.08 -0.36 10.01 | norm=10.08) gyro=(0.0030 -0.0040 0.0000 | norm=0.0050) temp=27.37 used_samples=4
[EIS Demo] frames=824 success=824 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2802 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.035 ms max_cost=0.071 ms cpu=3.00%
           latest_imu: accel=(-1.07 -0.36 10.00 | norm=10.07) gyro=(0.0000 -0.0040 0.0000 | norm=0.0040) temp=27.56 used_samples=4
[EIS Demo] frames=855 success=855 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=2905 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.036 ms max_cost=0.071 ms cpu=3.87%
           latest_imu: accel=(-1.09 -0.37 9.94 | norm=10.01) gyro=(0.0000 0.0000 0.0000 | norm=0.0000) temp=27.56 used_samples=4
[EIS Demo] frames=886 success=886 failed_eis=0 success_rate=100.00% buffered=2048 total_imu=3008 failed_imu=0 latest_offset=(0,0) max_abs_offset=(0,0) nonzero=0 avg_cost=0.036 ms max_cost=0.071 ms cpu=3.87%
           latest_imu: accel=(-1.06 -0.37 9.97 | norm=10.03) gyro=(0.0030 -0.0030 0.0000 | norm=0.0042) temp=27.43 used_samples=4
========================================
Demo finished. frames=900 success=900 failed_eis=0 success_rate=100.00% total_imu=3059 failed_imu=0 max_abs_offset=(0,0)
========================================
```


```text
测试条件：IMU 100Hz，模拟视频帧 30FPS，halfWindowMs=20ms，运行 30秒
帧数：900
EIS 成功帧数：900
EIS 失败帧数：0
成功率：100.00%
IMU 样本数：约 3059 条
IMU 失败读取：0
used_samples：约 4 条
单帧平均计算耗时：约 0.013~0.036 ms
单帧最大计算耗时：约 0.071 ms
进程 CPU 占用：约 2%~7%，平均约 4%
```

结论：当前 Demo 已证明 `IMU读取 → ring buffer → 时间窗口查询 → gyro积分 → offset计算` 链路稳定跑通，并且 CPU 开销较低。

---


### 9. 双相机参数化测试

当前 `icm45686_eis_demo` 已支持扩展参数，可以模拟不同帧率相机的 EIS 配置。双相机系统中，建议使用一份 `ImuConfig` 和两份 `EisCameraConfig`：

```text
IMU 全局配置：
sampleHz、gyroRange、accelRange、gyroBias

15FPS 相机配置：
frameRate=15、halfWindowMs=20~30、timeOffsetMs单独标定、smoothingAlpha可稍大

30FPS 相机配置：
frameRate=30、halfWindowMs=15~20、timeOffsetMs单独标定、smoothingAlpha可稍小
```

#### 15FPS 相机起步测试

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 200 15 25 1 1 120 0 -1 1 0 0.35 2000
```

#### 30FPS 相机起步测试

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 200 30 20 1 1 120 0 -1 1 0 0.25 2000
```

这两个命令的主要区别是：

```text
15FPS：halfWindowMs=25，smoothingAlpha=0.35
30FPS：halfWindowMs=20，smoothingAlpha=0.25
```

15FPS 帧间隔更长，可以使用稍大的积分窗口；30FPS 帧间隔更短，窗口不宜过大，否则可能带来补偿延迟。详细参数选取方法见 `docs/PARAMETER-SELECTION.md`。

---

## 性能指标排查指南

### 1. 进程 CPU 占用

```bash
top -p $(pidof icm45686_eis_demo)
```

或观察 Demo 自带 `cpu=` 字段。

正常表现：

```text
100Hz IMU + 30FPS EIS 计算下，CPU 约 2%~7%
```

### 2. 线程 CPU 负载

```bash
top -H -p $(pidof icm45686_eis_demo)
```

应重点关注：

* 主线程是否稳定；
* IMU 读取线程是否过高；
* 是否存在 busy loop 导致某线程 CPU 异常升高。

### 3. 文件描述符泄漏

```bash
watch -n 2 'ls /proc/$(pidof icm45686_eis_demo)/fd | wc -l'
```

正常表现：运行期间 fd 数量应稳定，不应持续增长。

### 4. IMU 数据稳定性

关注心跳中的：

```text
accel norm
gyro norm
temp
failed_imu
```

正常表现：

* 静止时 `accel norm ≈ 9.8~10.0`；
* 静止时 `gyro norm ≈ 0`；
* `temp` 缓慢变化，不大幅跳变；
* `failed_imu=0`。

---

## 常见异常与定位方法

| 异常 | 现象 | 原因 | 处理方法 |
|---|---|---|---|
| EIS 失败 | `success=0`，`used_samples=1` | halfWindow 太小 | 使用 `halfWindowMs=20` 或更大 |
| offset 静止为 0 | `latest_offset=(0,0)` | 静止状态无角运动 | 正常现象，做动态转动测试 |
| 动态 offset 仍为 0 | gyro 有变化但 offset 不变 | 焦距过小/转动过小/轴向映射问题 | 增大 focal，轻微加大转动，检查 axis sign |
| CPU 过高 | `cpu` 明显高于 10% | 读取线程频率过高或 busy loop | 降低 sampleHz，检查 sleep 逻辑 |
| IMU 失败读取 | `failed_imu` 增长 | 驱动或设备节点异常 | 查看 `dmesg`，检查驱动加载 |
| `/dev/icm45686` 不存在 | open 失败 | 驱动 probe 失败 | 先运行基础驱动检查命令 |

---

## 新增 Demo 文档模板

新增 Demo 时，请按以下格式追加到本文档：

```text
## Demo XX: Demo 名称

**源文件**：src/xxx.cpp

### 1. 演示目标
说明该 Demo 验证什么能力。

### 2. 线程架构说明
说明主线程、后台线程、消费者线程之间的数据流。

### 3. 运行方法
给出默认命令和带参数命令。

### 4. 预期输出
贴出关键日志。

### 5. 性能指标
说明 CPU、耗时、成功率、失败数等指标。

### 6. 常见异常
说明如何排查。
```
