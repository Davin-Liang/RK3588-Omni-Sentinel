# ICM45686 EIS App — 技术实现文档

## 1. 概述

`icm45686-eis-app` 是运行在 RK3588 平台上的 ICM45686 IMU 应用层组件。它基于已经调通的 ICM45686 SPI 内核驱动，通过 `/dev/icm45686` 字符设备读取 IMU 数据，并在用户态完成时间戳标记、环形缓冲、时间窗口查询、陀螺仪积分和 EIS 像素偏移计算。

当前工程定位不是 IIO 驱动，也不直接实现图像裁剪或 RGA 平移，而是提供一个清晰的 **IMU → EIS offset** 应用层接口。后续相机/视觉模块可直接调用 `calculate_eis_offset()` 获取 `offsetX / offsetY`，再接入图像防抖链路。

---

## 2. 架构总览

```text
                      ICM45686 IMU Module
                              │
                              │ SPI4
                              ▼
                    RK3588 SPI Controller
                              │
                              │ device tree: icm45686@0
                              ▼
          ┌────────────────────────────────────┐
          │  Kernel Space                      │
          │                                    │
          │  inv_imu_driver.ko                 │
          │    ├─ WHO_AM_I / register config   │
          │    ├─ raw accel/gyro/temp read     │
          │    └─ unit conversion              │
          │                                    │
          │  icm45686_spi.ko                   │
          │    ├─ SPI read/write               │
          │    ├─ char device registration     │
          │    └─ /dev/icm45686                │
          └────────────────────────────────────┘
                              │
                              │ read/ioctl
                              ▼
          ┌────────────────────────────────────┐
          │  User Space                        │
          │                                    │
          │  Icm45686Reader                    │
          │    ├─ open /dev/icm45686           │
          │    ├─ background read thread       │
          │    ├─ CLOCK_MONOTONIC timestamp    │
          │    └─ push to ImuRingBuffer        │
          │                                    │
          │  ImuRingBuffer                     │
          │    ├─ keep recent IMU samples      │
          │    └─ query by time window         │
          │                                    │
          │  EisStabilizer                     │
          │    ├─ gyro trapezoid integration   │
          │    ├─ angle → pixel offset         │
          │    └─ offsetX / offsetY            │
          └────────────────────────────────────┘
                              │
                              ▼
                    Camera / RGA / EIS pipeline
```

### 设计边界

| 层级 | 当前工程职责 | 不负责内容 |
|---|---|---|
| 内核驱动 | SPI 通信、寄存器配置、字符设备、基础数据读取 | IIO buffer、硬件中断 kfifo、DMA 触发采样 |
| 应用层读取 | 打开 `/dev/icm45686`、周期读取、时间戳、缓存 | 真实相机帧时间戳获取 |
| EIS 接口 | 根据目标时间窗口计算 `offsetX/offsetY` | 图像裁剪、RGA 平移、视频编码 |
| Demo | 验证算法链路和 CPU 开销 | 最终产品级防抖画面效果 |

---

## 3. 模块划分

| 文件 | 职责 |
|---|---|
| `include/icm45686_user.h` | C 语言用户态访问接口声明，封装 ioctl 命令和数据结构 |
| `src/icm45686_user.c` | C 语言设备访问实现，负责 open/read/ioctl/close |
| `include/imu_ahrs.h` | AHRS 姿态解算接口声明 |
| `src/imu_ahrs.c` | Madgwick 姿态解算实现，用于基础 IMU Demo |
| `include/imu_eis.hpp` | C++ 应用层 EIS 接口声明，定义 `ImuSample`、`ImuRingBuffer`、`Icm45686Reader`、`EisStabilizer` |
| `src/imu_eis.cpp` | C++ 应用层 EIS 接口实现 |
| `src/icm45686_app.c` | 基础 IMU 数据读取 Demo |
| `src/eis_demo.cpp` | EIS offset 计算与 CPU 压测 Demo |
| `CMakeLists.txt` | CMake 构建配置 |
| `build.sh` | 交叉编译脚本 |

---

## 4. 核心数据结构

### 4.1 ImuSample

`ImuSample` 是应用层统一 IMU 样本结构。每次从 `/dev/icm45686` 读取数据后，都会补充一个单调时间戳。

```cpp
struct ImuSample {
    uint64_t timestampNs;  // CLOCK_MONOTONIC，单位 ns

    float accelX;          // m/s²
    float accelY;
    float accelZ;

    float gyroX;           // rad/s
    float gyroY;
    float gyroZ;

    float temperature;     // ℃
};
```

设计要点：

* 时间戳使用 `CLOCK_MONOTONIC`，避免系统时间调整影响；
* 加速度、角速度、温度均在用户态使用 float 表示；
* 该结构是 EIS 算法与底层驱动解耦的核心数据单元。

### 4.2 ImuRingBuffer

`ImuRingBuffer` 保存最近一段时间的 IMU 样本，供 EIS 按时间窗口查询。

```cpp
class ImuRingBuffer {
public:
    explicit ImuRingBuffer(size_t maxSamples = 512);
    void push(const ImuSample& sample);
    void clear();
    size_t size() const;
    bool latest(ImuSample& sample) const;
    bool getSamplesBetween(uint64_t startTimeNs,
                           uint64_t endTimeNs,
                           std::vector<ImuSample>& samples) const;
};
```

实现方式：

* 内部使用 `std::deque<ImuSample>`；
* 使用 `std::mutex` 保护并发访问；
* 写入超过容量后从队首删除旧样本；
* 当前 Demo 默认使用 2048 条缓存，100Hz 下约可保存 20 秒历史数据。

### 4.3 Icm45686Reader

`Icm45686Reader` 负责连接内核字符设备和应用层 ring buffer。

```cpp
class Icm45686Reader {
public:
    bool openDevice(const std::string& devPath = "/dev/icm45686");
    void closeDevice();

    bool setAccelRange(uint8_t range);
    bool setGyroRange(uint8_t range);

    bool readSample(ImuSample& sample);

    bool start(float sampleHz = 100.0f);
    void stop();

    bool getLatestSample(ImuSample& sample) const;
    bool getSamplesBetween(uint64_t startTimeNs,
                           uint64_t endTimeNs,
                           std::vector<ImuSample>& samples) const;
};
```

核心逻辑：

```text
openDevice()
  └── icm45686_open("/dev/icm45686")

start(100Hz)
  └── 创建后台线程 readLoop()

readLoop()
  ├── icm45686_read_data(fd, &data)
  ├── imu_get_time_ns()
  ├── 转换为 ImuSample
  └── ringBuffer.push(sample)
```

### 4.4 EisStabilizer

`EisStabilizer` 根据目标帧曝光时间附近的 IMU 数据计算像素补偿量。

```cpp
class EisStabilizer {
public:
    bool bindReader(Icm45686Reader* reader);

    bool calculate_eis_offset(float focalX,
                              float focalY,
                              uint64_t targetTimestampNs,
                              uint32_t halfWindowMs,
                              int32_t& offsetX,
                              int32_t& offsetY);

    double lastCostMs() const;
    size_t lastUsedSamples() const;

    void setAxisSign(float signX, float signY);
    void setMaxOffset(int32_t maxOffsetPixel);
};
```

参数含义：

| 参数 | 含义 |
|---|---|
| `focalX` | 水平方向焦距，单位 pixel |
| `focalY` | 垂直方向焦距，单位 pixel |
| `targetTimestampNs` | 当前视频帧曝光中心时间戳，单位 ns |
| `halfWindowMs` | 查询 IMU 数据的时间窗口半径，单位 ms |
| `offsetX` | 输出水平像素补偿量 |
| `offsetY` | 输出垂直像素补偿量 |

---

## 5. 线程模型

### 5.1 线程清单

| 线程 | 所在模块 | 职责 | 生命周期 |
|---|---|---|---|
| Main Thread | `eis_demo.cpp` | 初始化、模拟帧循环、调用 EIS、打印性能日志 | 进程启动到退出 |
| IMU Reader Thread | `Icm45686Reader::readLoop()` | 按 `sampleHz` 周期读取 IMU，写入 ring buffer | `reader.start()` 创建，`reader.stop()` 停止 |

### 5.2 线程生命周期

```text
main()
  │
  ├─ reader.openDevice("/dev/icm45686")
  ├─ reader.setAccelRange(0)
  ├─ reader.setGyroRange(0)
  ├─ reader.start(100Hz)
  │      │
  │      └─ readLoop thread
  │           ├─ readSample()
  │           ├─ timestamp
  │           ├─ ringBuffer.push()
  │           └─ sleep until next sample
  │
  ├─ stabilizer.bindReader(&reader)
  ├─ frame loop at 30FPS
  │      ├─ targetTimestamp = now - halfWindowMs
  │      ├─ calculate_eis_offset()
  │      └─ print heartbeat every 1s
  │
  └─ reader.stop(); reader.closeDevice()
```

### 5.3 为什么不用 epoll

当前 `/dev/icm45686` 是自定义字符设备，`read/ioctl` 属于主动读取传感器寄存器，并不是 IIO buffer 那种“数据就绪后 fd 可读”的模型。因此第一版应用层 Demo 不使用 `epoll_wait()`，而采用后台线程周期读取。

如果未来内核驱动加入：

```text
硬件中断 → wait_queue → poll() → 用户态 epoll_wait()
```

则可以将 `Icm45686Reader::readLoop()` 改为 epoll 驱动模型。当前阶段没有必要重构驱动。

---

## 6. 核心数据流

### 6.1 IMU 读取流程

```text
Icm45686Reader::readLoop()
  │
  ├─ icm45686_read_data(fd, &data)
  │    └─ ioctl ICM45686_IOC_READ_DATA
  │
  ├─ imu_get_time_ns()
  │    └─ clock_gettime(CLOCK_MONOTONIC)
  │
  ├─ 填充 ImuSample
  │    ├─ accelX/Y/Z
  │    ├─ gyroX/Y/Z
  │    ├─ temperature
  │    └─ timestampNs
  │
  └─ ImuRingBuffer::push(sample)
```

### 6.2 EIS offset 计算流程

```text
EisStabilizer::calculate_eis_offset()
  │
  ├─ startTime = targetTimestampNs - halfWindowMs
  ├─ endTime   = targetTimestampNs + halfWindowMs
  │
  ├─ reader->getSamplesBetween(startTime, endTime, samples)
  │    └─ 从 ring buffer 中取出时间窗口内 IMU 样本
  │
  ├─ 样本数检查
  │    └─ 少于 2 条则无法积分，返回 false
  │
  ├─ integrateGyro(samples, thetaX, thetaY, thetaZ)
  │    └─ 对 gyro 做梯形积分，得到角度变化 rad
  │
  ├─ 小角度近似转换为像素偏移
  │    ├─ offsetX = focalX × thetaY × signX
  │    └─ offsetY = focalY × thetaX × signY
  │
  ├─ clamp 到 ±maxOffsetPixel
  └─ 返回 true
```

### 6.3 时间戳窗口设计

Demo 中使用：

```cpp
targetTimestampNs = nowNs - halfWindowMs * 1000000ULL;
```

原因：`calculate_eis_offset()` 查询窗口为：

```text
[targetTimestampNs - halfWindowMs, targetTimestampNs + halfWindowMs]
```

如果直接使用 `nowNs`，窗口右半部分会落到未来，ring buffer 中还没有未来 IMU 数据，容易导致样本不足。将目标时间戳往前移动 `halfWindowMs` 后，实际查询窗口变为：

```text
[now - 2 * halfWindowMs, now]
```

整个窗口都落在历史数据中，更适合实时 Demo 测试。

---

## 7. EIS 算法说明

### 7.1 陀螺仪积分

陀螺仪输出单位为 `rad/s`，两个相邻样本之间使用梯形积分：

```text
Δθx += 0.5 × (gyroX[i-1] + gyroX[i]) × Δt
Δθy += 0.5 × (gyroY[i-1] + gyroY[i]) × Δt
Δθz += 0.5 × (gyroZ[i-1] + gyroZ[i]) × Δt
```

其中：

```text
Δt = (timestamp[i] - timestamp[i-1]) / 1e9
```

### 7.2 角度到像素偏移

当前 Demo 使用小角度近似：

```text
offsetX ≈ focalX × Δθy
offsetY ≈ focalY × Δθx
```

这是 Demo 级映射关系，真实产品中需要根据：

* 相机坐标系；
* IMU 坐标系；
* IMU 与相机安装方向；
* 图像是否裁剪 / 是否旋转；
* 光学中心和相机内参；

进一步标定轴向和符号。

### 7.3 offset 限幅

`setMaxOffset(200)` 将输出限制在：

```text
-200 <= offsetX <= 200
-200 <= offsetY <= 200
```

防止异常瞬时陀螺仪数据导致画面补偿过大。

---

## 8. 编译与部署实现

### 8.1 CMake 目标

当前 CMake 构建两个可执行文件：

| 目标 | 源文件 | 说明 |
|---|---|---|
| `icm45686_app` | `src/icm45686_app.c`、`src/icm45686_user.c`、`src/imu_ahrs.c` | 基础 IMU 数据 Demo |
| `icm45686_eis_demo` | `src/eis_demo.cpp`、`src/imu_eis.cpp`、`src/icm45686_user.c` | EIS 防抖接口 Demo |

### 8.2 build.sh

`build.sh` 设置交叉编译器：

```bash
TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
GCC_COMPILER=$TOOL_CHAIN/bin/aarch64-buildroot-linux-gnu
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++
```

随后执行：

```bash
mkdir -p build/
cd build/
cmake ..
make -j4
make install
```

安装产物位于 `install/`。

---

## 9. 已知限制与后续扩展

### 9.1 已知限制

1. 当前不是 IIO 驱动方案，没有 `/dev/iio:deviceX`、IIO kfifo、sysfs buffer enable。
2. 当前时间戳在用户态读取完成后打点，不是硬件中断触发时间戳。
3. 当前 offset 只完成计算，未直接作用到真实图像帧。
4. 当前轴向映射是 Demo 默认映射，真实安装后需要标定。
5. 当前 yaw 无磁力计校正，长期漂移是正常现象。

### 9.2 后续扩展

1. 与相机模块对齐真实曝光中心时间戳；
2. 根据相机内参替换 Demo 中的 `focalX/focalY`；
3. 接入 RGA 图像平移或裁剪模块，形成完整防抖闭环；
4. 加入陀螺仪零偏在线估计；
5. 增加动态防抖 benchmark；
6. 若项目要求更标准的 Linux 传感器框架，可升级为 IIO + 中断 + kfifo + epoll。
