# ICM45686 EIS App — 学习指南

## 目标

本文档面向项目组其他成员，用于快速理解 `icm45686-eis-app` 这一部分的背景、代码结构、设计原因、调试经验和继续学习路线。学习完成后，应能够说清楚：

1. 这个模块做了什么；
2. 为什么当前采用 `/dev/icm45686` 字符设备方案，而不是 IIO；
3. 应用层如何读取 IMU 并缓存；
4. EIS 防抖 offset 是怎么计算出来的；
5. 如何运行 Demo 并判断结果是否成功；
6. 之前踩过哪些坑，遇到类似问题如何排查。

---

## 第一层：能说清“做了什么”

### 一句话概括

> `icm45686-eis-app` 是 RK3588 平台上的 ICM45686 IMU 应用层防抖接口工程：它通过 `/dev/icm45686` 字符设备读取 IMU 数据，在用户态建立时间戳环形缓冲区，并根据视频帧目标时间窗口内的陀螺仪积分结果计算 `offsetX/offsetY` 像素级电子防抖补偿量。

### 架构图（要能画出来）

```text
ICM45686 IMU
    │ SPI4
    ▼
RK3588 SPI Controller
    │
    ▼
icm45686_spi.ko + inv_imu_driver.ko
    │
    ▼
/dev/icm45686
    │
    ▼
Icm45686Reader
    │  后台线程 100Hz 读取 + CLOCK_MONOTONIC 时间戳
    ▼
ImuRingBuffer
    │  保存最近 IMU 历史数据
    ▼
EisStabilizer
    │  按目标帧时间戳取窗口 → gyro 积分 → 角度转像素
    ▼
offsetX / offsetY
```

### 关键代码（需要熟悉）

```cpp
Icm45686Reader reader(2048);
EisStabilizer stabilizer;

reader.openDevice("/dev/icm45686");
reader.setAccelRange(0);   // ±2G
reader.setGyroRange(0);    // ±250DPS
reader.start(100.0f);      // 100Hz 后台读取

stabilizer.bindReader(&reader);
stabilizer.setMaxOffset(200);

uint64_t targetTimestampNs = imu_get_time_ns() - 20ULL * 1000000ULL;
int32_t offsetX = 0;
int32_t offsetY = 0;

bool ok = stabilizer.calculate_eis_offset(
    1200.0f, 1200.0f,
    targetTimestampNs,
    20,
    offsetX, offsetY
);
```

要能解释：

* `reader.start(100.0f)` 为什么是 100Hz；
* `targetTimestampNs = now - 20ms` 为什么往前偏移；
* `halfWindowMs=20` 为什么比 `5` 合理；
* 静止时 `offset=(0,0)` 为什么是正确结果。

---

## 第二层：能解释“为什么这么设计”

### 决策 1：为什么当前使用字符设备 `/dev/icm45686`，而不是 IIO

| 对比维度 | IIO 标准方案 | 当前字符设备方案 |
|---|---|---|
| 设备节点 | `/dev/iio:deviceX` | `/dev/icm45686` |
| 内核框架 | IIO buffer + trigger + kfifo | 自定义 SPI 字符设备 |
| 用户态读取 | sysfs enable + epoll + read | open/read/ioctl 或后台线程周期读取 |
| 时间戳 | 可在中断下半部写入 | 当前在用户态读取完成后打时间戳 |
| 工作量 | 需要重构驱动 | 当前已调通，能快速完成 Demo |
| 当前阶段适合性 | 产品化更标准 | Demo 与功能验证更合适 |

当前项目已经完成：

```text
SPI接线 → 设备树 → WHO_AM_I → /dev/icm45686 → IMU数据正常 → EIS计算成功
```

因此现阶段优先基于已调通的字符设备完成应用层接口。只有后续明确要求中断触发、IIO kfifo、DMA 或 epoll 时，再重构为 IIO。

### 决策 2：为什么需要应用层环形缓冲区

防抖不是只看最新一条 IMU 数据，而是要看某一帧曝光时间附近的一段 IMU 数据。假设视频帧中心时间为 `T`，就需要查询：

```text
[T - halfWindow, T + halfWindow]
```

因此必须保存最近一段历史数据。`ImuRingBuffer` 的作用就是在应用层模拟一个“历史 IMU kfifo”：

```text
IMU读取线程不断 push 最新样本
EIS计算线程按时间戳查询某个窗口内样本
```

### 决策 3：为什么 `halfWindowMs` 默认用 20ms

100Hz IMU 的采样间隔约为：

```text
1000ms / 100 = 10ms
```

如果 `halfWindowMs=5ms`，总窗口只有 10ms，很容易只取到 1 条样本。陀螺仪积分至少需要 2 条样本，因此会出现：

```text
success=0
used_samples=1
avg_cost=0.000 ms
```

改成 `halfWindowMs=20ms` 后，总窗口为 40ms，通常能取到 3~5 条样本，EIS 计算成功率可达到 100%。

### 决策 4：为什么目标时间戳要用 `now - halfWindow`

如果直接用：

```cpp
targetTimestampNs = now;
```

那么查询窗口是：

```text
[now - halfWindow, now + halfWindow]
```

右半部分落到了未来，ring buffer 中没有未来 IMU 数据，容易样本不足。

因此 Demo 中使用：

```cpp
targetTimestampNs = now - halfWindow;
```

此时查询窗口变成：

```text
[now - 2 * halfWindow, now]
```

窗口完整落在历史数据中，实时计算更稳定。

### 决策 5：为什么静止时 offset 为 0 是正确结果

EIS offset 来自陀螺仪角速度积分：

```text
角速度 ≈ 0  →  积分角度 ≈ 0  →  像素偏移 ≈ 0
```

因此静止测试中看到：

```text
latest_offset=(0,0)
max_abs_offset=(0,0)
```

是正确结果，不代表算法失败。动态测试时需要轻轻转动 IMU，观察 `gyro norm`、`latest_offset`、`max_abs_offset` 和 `nonzero` 是否变化。

---

## 第三层：能讲清 EIS 算法

### 1. 输入数据

每条 IMU 数据包含：

```text
timestampNs
accelX/Y/Z，单位 m/s²
gyroX/Y/Z，单位 rad/s
temperature，单位 ℃
```

EIS 主要使用陀螺仪 `gyroX / gyroY / gyroZ`，加速度数据主要用于验证 IMU 是否正常、后续可用于姿态辅助。

### 2. 查窗口

```cpp
startTimeNs = targetTimestampNs - halfWindowMs * 1e6;
endTimeNs   = targetTimestampNs + halfWindowMs * 1e6;
reader->getSamplesBetween(startTimeNs, endTimeNs, samples);
```

如果 `samples.size() < 2`，无法积分，返回失败。

### 3. 梯形积分

```text
θx += 0.5 × (gyroX[i-1] + gyroX[i]) × Δt
θy += 0.5 × (gyroY[i-1] + gyroY[i]) × Δt
θz += 0.5 × (gyroZ[i-1] + gyroZ[i]) × Δt
```

### 4. 角度转像素

当前 Demo 使用小角度近似：

```text
offsetX ≈ focalX × θy
offsetY ≈ focalY × θx
```

真实项目中要根据相机坐标系和 IMU 安装方向调整轴向和符号。

### 5. 限幅

```text
-200 <= offsetX <= 200
-200 <= offsetY <= 200
```

防止异常数据导致画面补偿过大。

---

## 第四层：能跑 Demo 并判断是否成功

### 1. 基础 IMU 测试

```bash
/usr/bin/icm45686_app
```

判断标准：

```text
accel norm ≈ 9.8~10.0
gyro ≈ 0
temp 稳定
Yaw/Pitch/Roll 有三个角输出
```

### 2. EIS 静态测试

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20
```

成功标准：

```text
frames=900
success=900
failed_eis=0
success_rate=100.00%
used_samples≈4
failed_imu=0
cpu≈2%~7%
offset=(0,0)
```

### 3. EIS 动态测试

运行 Demo 时轻轻转动模块。

成功标准：

```text
gyro norm 变大
latest_offset 出现非零
max_abs_offset 变大
nonzero 增加
success_rate 仍接近 100%
```

---

## 第五层：调试 bug 和教训

### Bug 1：驱动加载但没有 `/dev/icm45686`

**现象**：

```bash
ls: cannot access '/dev/icm45686': No such file or directory
Failed to get reset GPIO
```

**原因**：reset GPIO 强制申请失败，probe 没走到字符设备创建。

**解决**：将 reset GPIO 改为 optional；未连接 reset 时跳过硬件复位。

### Bug 2：设备树 GPIO 编号写错

**现象**：GPIO 写成：

```dts
interrupts = <45 IRQ_TYPE_LEVEL_HIGH>;
reset-gpios = <&gpio1 44 GPIO_ACTIVE_LOW>;
```

**原因**：`<&gpio1 ...>` 后面的编号应是 bank 内偏移，不是全局编号。

**解决**：使用 Rockchip 宏：

```dts
interrupts = <RK_PB5 IRQ_TYPE_LEVEL_HIGH>;
reset-gpios = <&gpio1 RK_PB4 GPIO_ACTIVE_LOW>;
```

### Bug 3：WHO_AM_I 读到 0x00 / 0xff

**现象**：

```bash
Invalid WHO_AM_I value: 0x00 (expected 0x68)
```

**原因**：原代码使用旧芯片 WHO_AM_I 地址和值。

**解决**：ICM45686 正确为：

```c
#define ICM45686_REG_WHO_AM_I 0x72
#define ICM45686_WHO_AM_I_VAL 0xE9
```

### Bug 4：数据全是 0

**原因**：只改了 WHO_AM_I，其余数据寄存器和配置寄存器仍是旧地址。

**解决**：使用 ICM45686 正确寄存器：

```text
ACCEL_DATA 从 0x00 开始
GYRO_DATA 从 0x06 开始
TEMP_DATA 从 0x0C 开始
PWR_MGMT0 = 0x10
ACCEL_CONFIG0 = 0x1B
GYRO_CONFIG0 = 0x1C
```

### Bug 5：静止数据乱跳，温度离散跳变

**原因**：ICM45686 默认 Little Endian，但代码按 Big Endian 解析。

**解决**：按低字节在前组包：

```c
raw = (int16_t)(buf[0] | (buf[1] << 8));
```

### Bug 6：EIS Demo `success=0`

**原因**：`halfWindowMs=5` 时窗口太小，100Hz 下通常只有 1 条样本。

**解决**：改为 `halfWindowMs=20`，目标时间戳使用 `now - halfWindow`。

### Bug 7：交叉编译 file in wrong format

**现象**：

```bash
Relocations in generic ELF (EM: 62)
file in wrong format
```

**原因**：C 文件用 aarch64-gcc 编译，C++ 文件却被主机 g++ 编译，或者旧 `.o` 没清理。

**解决**：同时设置 `CC` 和 `CXX`，并清理旧目标文件：

```bash
rm -f *.o icm45686_app icm45686_eis_demo
```

---

## 第六层：推荐学习路线

### 阶段 1：先理解 IMU 数据

学习内容：

* 加速度计单位和重力加速度；
* 陀螺仪单位 `rad/s`；
* 静止时三轴加速度模长为什么接近 9.8；
* 六轴 IMU 为什么 yaw 会漂移。

建议阅读代码：

```text
src/icm45686_app.c
src/icm45686_user.c
include/icm45686_user.h
```

### 阶段 2：理解用户态读取与线程

学习内容：

* C++ `std::thread`；
* `std::atomic<bool>`；
* `std::mutex`；
* `CLOCK_MONOTONIC`；
* ring buffer。

建议阅读代码：

```text
include/imu_eis.hpp
src/imu_eis.cpp
```

重点看：

```text
Icm45686Reader::start()
Icm45686Reader::readLoop()
ImuRingBuffer::push()
ImuRingBuffer::getSamplesBetween()
```

### 阶段 3：理解 EIS offset 计算

学习内容：

* 陀螺仪积分；
* 时间戳窗口；
* 小角度近似；
* 焦距与像素偏移关系；
* offset 限幅。

重点看：

```text
EisStabilizer::calculate_eis_offset()
EisStabilizer::integrateGyro()
```

### 阶段 4：理解工程化测试

学习内容：

* 交叉编译；
* CMake；
* CPU 监控；
* Demo 日志字段；
* 静态测试与动态测试区别。

建议运行：

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20
```

---

## 面试 / 汇报话术

可以这样概括：

> 我负责把 ICM45686 IMU 通过 SPI 接入 RK3588，并在应用层实现了一个 EIS 防抖 offset 计算接口。底层通过自定义字符设备 `/dev/icm45686` 提供 IMU 数据，应用层用 C++ 封装了读取线程、时间戳环形缓冲区和防抖计算类。EIS 算法通过目标帧曝光时间附近的陀螺仪数据做梯形积分，然后根据相机焦距将角度变化转换为像素偏移。当前 Demo 在 100Hz IMU、30FPS 模拟帧率、20ms 半窗口配置下，30 秒处理 900 帧，成功率 100%，CPU 约 2%~7%，单帧计算耗时小于 0.1ms，证明链路已经稳定跑通。

---

## 最终掌握标准

看完并实操后，应能回答以下问题：

1. `/dev/icm45686` 是哪里来的？
2. 为什么当前不是 IIO 方案？
3. 为什么 EIS 需要 ring buffer？
4. 为什么 `halfWindowMs=5` 会失败？
5. 为什么 `targetTimestampNs` 要用 `now - halfWindow`？
6. 静止时 offset 为 0 是否正常？
7. 如何判断 IMU 数据是否正常？
8. 如何判断 EIS 动态防抖是否有效？
9. CPU 利用率怎么看？
10. 后续如何接入真实相机和 RGA 平移？
