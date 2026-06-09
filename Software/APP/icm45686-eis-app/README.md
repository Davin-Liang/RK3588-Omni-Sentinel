# ICM45686-EIS-App

**ICM45686-EIS-App** 是一个面向瑞芯微（Rockchip）RK3588 平台的 IMU 数据读取与电子防抖（EIS, Electronic Image Stabilization）应用层工程。

本工程基于已经调通的 **ICM45686 SPI 字符设备驱动**，通过 `/dev/icm45686` 在用户态读取加速度计、陀螺仪与温度数据，并在应用层完成 **IMU 时间戳标记、环形缓冲、时间窗口检索、陀螺仪积分与像素级防抖补偿量计算**。工程同时提供基础 IMU 数据测试程序和 EIS 防抖接口测试 Demo，可作为后续接入真实相机帧时间戳、视频裁剪/平移补偿和完整电子防抖链路的应用层底座。

当前方案采用的是 **自定义 SPI 字符设备 + 用户态 C/C++ 接口**，而不是 Linux IIO 子系统。也就是说，本工程不依赖 `/dev/iio:deviceX`、IIO kfifo、DMA 中断触发等机制，而是在当前已经验证通过的 `/dev/icm45686` 驱动接口基础上完成防抖应用层封装。

---

## ✨ 核心架构与特性

* **基于已调通 SPI 字符设备方案**：底层 ICM45686 已通过 RK3588 SPI4 接入，内核驱动成功创建 `/dev/icm45686`，应用层可直接通过 `open/read/ioctl` 获取 IMU 数据。
* **完整 IMU 用户态封装**：`icm45686_user.c` 对 `/dev/icm45686` 的打开、关闭、数据读取、加速度计量程设置和陀螺仪量程设置进行了 C 接口封装，便于 C/C++ 程序复用。
* **后台采样线程**：`Icm45686Reader` 以固定频率（默认 100 Hz）循环读取 IMU 数据，并为每个样本附加 `CLOCK_MONOTONIC` 时间戳。
* **应用层环形缓冲区**：`ImuRingBuffer` 保存最近一段 IMU 历史数据，支持按时间区间查询，为视频帧曝光时间附近的运动估计提供数据基础。
* **EIS 像素补偿接口**：`EisStabilizer::calculate_eis_offset()` 根据目标帧时间戳，从 IMU 历史窗口中取样，对陀螺仪角速度做积分，并将角度变化映射为屏幕像素偏移量 `offsetX / offsetY`。
* **基础 AHRS 姿态测试**：`icm45686_app` 支持输出加速度、陀螺仪、温度以及 yaw / pitch / roll 三个姿态角，便于验证 IMU 数据是否正常。
* **防抖 Demo 可观测指标完整**：`icm45686_eis_demo` 输出 `success_rate`、`used_samples`、`latest_offset`、`max_abs_offset`、`nonzero`、`avg_cost`、`max_cost`、`cpu` 等指标，便于验证防抖计算链路和性能开销。
* **CMake 工程化管理**：项目按照 `docs / include / src` 结构组织，提供 `CMakeLists.txt` 和 `build.sh`，支持在 RK3588 Buildroot 工具链下交叉编译和安装。

当前应用层链路如下：

```text
ICM45686 IMU 模块
    ↓ SPI4
RK3588 SPI 控制器
    ↓ 内核驱动 icm45686_spi.ko + inv_imu_driver.ko
/dev/icm45686
    ↓ 用户态 C 接口 icm45686_user.c
Icm45686Reader 后台读取线程
    ↓
ImuRingBuffer 应用层环形缓冲区
    ↓
EisStabilizer::calculate_eis_offset()
    ↓
icm45686_eis_demo 输出防抖 offset 与性能指标
```

---

## 📁 目录结构

```text
icm45686-eis-app/
├── docs/                         # 文档目录
│   ├── BUG_RECORD.md              # 调试问题与解决记录
│   ├── DEMO-INSTRUCTIONS.md       # EIS Demo 基础说明 
│   ├── IMPLEMENTATION.md          # 计算流程和技术架构说明
│   └── LEARNING_GUIDE.md          # 面向项目成员的学习路线
├── include/                      # 头文件目录
│   ├── icm45686_user.h              # /dev/icm45686 用户态 C 接口
│   ├── imu_ahrs.h                   # AHRS 姿态解算接口
│   └── imu_eis.hpp                  # IMU 读取、环形缓冲与 EIS C++ 接口
├── src/                          # 源码目录
│   ├── icm45686_user.c              # 字符设备 open/read/ioctl 封装
│   ├── imu_ahrs.c                   # Madgwick AHRS 姿态解算算法
│   ├── imu_eis.cpp                  # IMU 读取线程、环形缓冲区、EIS offset 计算
│   ├── icm45686_app.c               # 基础 IMU 数据与姿态角测试 Demo
│   └── eis_demo.cpp                 # EIS 防抖接口测试 Demo

├── CMakeLists.txt                # CMake 构建配置
├── build.sh                      # 一键交叉编译脚本
└── README.md                     # 项目说明文档
```

---

## 🛠️ 环境依赖

在编译和运行本工程之前，请确保具备以下环境：

1. **硬件平台**：RK3588 / RK3588S 等 ARM64 Rockchip 平台。
2. **IMU 模块**：ICM45686，已通过 SPI 接入 RK3588。
3. **内核驱动**：已加载 `inv_imu_driver.ko` 和 `icm45686_spi.ko`，并且已生成 `/dev/icm45686`。
4. **CMake**：建议 `>= 3.4.1`。
5. **交叉编译工具链**：如 `aarch64-buildroot-linux-gnu`。
6. **C/C++ 标准库与线程库**：工程使用 C++14、`std::thread`、`std::mutex`、`std::vector` 等标准组件。

默认工具链路径为：

```bash
/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
```

如果你的工具链不在该路径，需要修改 `build.sh` 中的：

```bash
TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
```

---

## 🚀 编译指南

本工程支持 CMake + build.sh 一键交叉编译。项目根目录下执行：

```bash
chmod +x build.sh
./build.sh
```

`build.sh` 会自动完成以下步骤：

```bash
mkdir -p build/
cd build/
cmake ..
make -j4
make install
```

编译完成后，可执行程序会安装到：

```text
install/icm45686_app
install/icm45686_eis_demo
```

如果需要手动构建，也可以执行：

```bash
mkdir -p build
cd build
cmake ..
make -j4
make install
```

---

## 📦 部署到 RK3588 板端

将编译出的程序复制到板端：

```bash
scp install/icm45686_app root@192.168.0.232:/usr/bin/
scp install/icm45686_eis_demo root@192.168.0.232:/usr/bin/
```

运行前请确认驱动已经加载：

```bash
insmod /usr/lib/modules/inv_imu_driver.ko
insmod /usr/lib/modules/icm45686_spi.ko
ls -la /dev/icm45686
```

正常情况下应看到：

```text
/dev/icm45686
```

如果设备节点不存在，请先检查内核驱动、设备树、SPI 接线和 `WHO_AM_I` 读取状态，相关排查记录可参考 `BUG_RECORD.md`。

---

## 📖 快速上手与接口示例

### 1. 基础 IMU 数据测试

运行：

```bash
/usr/bin/icm45686_app
```

该 Demo 用于验证：

* `/dev/icm45686` 是否可以正常打开；
* 加速度计、陀螺仪和温度是否正常输出；
* yaw / pitch / roll 三个姿态角是否可以持续更新。

静止状态下，合理输出应满足：

```text
accel norm ≈ 9.8 m/s²
gyro ≈ 0 rad/s
temp 稳定在室温附近
```

### 2. EIS 防抖接口测试

默认运行：

```bash
/usr/bin/icm45686_eis_demo
```

推荐显式指定参数：

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20
```

参数含义：

```text
/dev/icm45686  IMU 字符设备节点
30             Demo 运行时间，单位秒
1200 1200      focalX / focalY，相机焦距，单位 pixel
100            IMU 应用层读取频率，单位 Hz
30             模拟视频帧率，单位 FPS
20             EIS 积分半窗口，单位 ms
```

静止测试时，预期现象：

```text
success_rate=100.00%
failed_eis=0
used_samples≈4
latest_offset=(0,0)
accel norm≈9.8~10.1
gyro norm≈0
```

轻微转动或抖动 IMU 时，预期现象：

```text
gyro norm 变大
latest_offset 出现非零
max_abs_offset 增大
nonzero 增加
```

### 3. C++ 接口示例

`EisStabilizer` 的核心接口如下：

```cpp
bool calculate_eis_offset(float focalX,
                          float focalY,
                          uint64_t targetTimestampNs,
                          uint32_t halfWindowMs,
                          int32_t& offsetX,
                          int32_t& offsetY);
```

典型使用方式：

```cpp
#include "imu_eis.hpp"

int main() {
    Icm45686Reader reader(2048);
    EisStabilizer stabilizer;

    if (!reader.openDevice("/dev/icm45686")) return -1;

    reader.setAccelRange(0); // ±2G
    reader.setGyroRange(0);  // ±250DPS

    if (!reader.start(100.0f)) return -1;

    stabilizer.bindReader(&reader);
    stabilizer.setMaxOffset(200);

    uint64_t nowNs = imu_get_time_ns();
    uint32_t halfWindowMs = 20;
    uint64_t targetTimestampNs = nowNs - (uint64_t)halfWindowMs * 1000000ULL;

    int32_t offsetX = 0;
    int32_t offsetY = 0;

    bool ok = stabilizer.calculate_eis_offset(1200.0f,
                                              1200.0f,
                                              targetTimestampNs,
                                              halfWindowMs,
                                              offsetX,
                                              offsetY);

    reader.stop();
    reader.closeDevice();

    return ok ? 0 : -1;
}
```

当前 offset 计算采用小角度近似：

```text
offsetX ≈ focalX × ΔthetaY
offsetY ≈ focalY × ΔthetaX
```

实际产品中需要根据 IMU 安装方向、相机坐标系和画面裁剪策略进一步修正轴向和符号。

---

## 📊 实测基准数据（Benchmarks）

以下数据基于当前 RK3588 + ICM45686 模块的静态测试结果，测试配置如下：

```text
IMU Sample Rate : 100 Hz
Frame Rate      : 30 FPS
Half Window     : 20 ms
Window Width    : 40 ms
Focal           : 1200 / 1200 pixel
Runtime         : 30 s
```

实测结果：

* **帧处理数量**：30 秒内处理 `900` 帧，符合 30 FPS 设置。
* **EIS 计算成功率**：`success=900`，`failed_eis=0`，成功率 `100.00%`。
* **IMU 读取稳定性**：约读取 `3059` 条 IMU 数据，`failed_imu=0`，实际采样频率约 `102 Hz`。
* **时间窗口取样**：每帧 EIS 计算使用约 `4` 条 IMU 样本，说明 20 ms 半窗口可满足 100 Hz IMU 的积分取样需求。
* **计算耗时**：平均单帧 EIS 计算耗时约 `0.013~0.036 ms`，最大耗时约 `0.071 ms`。
* **CPU 利用率**：进程 CPU 占用约 `2%~7%`，两轮测试平均约 `4%` 左右。
* **静态 offset 结果**：静止状态下 `latest_offset=(0,0)`、`max_abs_offset=(0,0)`，符合无角运动时无需补偿的预期。
* **IMU 静态数据**：加速度模长约 `10.0 m/s²`，陀螺仪模长接近 `0 rad/s`，温度稳定在约 `27℃`。

该结果说明：

```text
IMU 读取 → 时间戳标记 → 环形缓冲区 → 时间窗口查询 → 陀螺仪积分 → EIS 像素偏移计算
```

这一整条应用层链路已经稳定跑通，并且 CPU 开销较低。

> 注意：上述数据主要验证静态场景下的算法链路和性能开销。若要验证动态防抖效果，需要在 Demo 运行过程中轻微转动或抖动 IMU，观察 `latest_offset`、`max_abs_offset` 和 `nonzero` 是否随角速度变化。

---

## ⚠️ 避坑与注意事项

1. **本工程不是 IIO 驱动方案**：当前工程使用 `/dev/icm45686` 字符设备。如果项目后续强制要求 `/dev/iio:deviceX`、`epoll_wait()`、IIO kfifo 或硬件中断触发，需要重构内核驱动。
2. **静止时 offset 为 0 是正确现象**：静止状态下陀螺仪接近 0，积分角度接近 0，因此 `latest_offset=(0,0)` 并不代表防抖算法失败。
3. **防抖功能必须动态测试**：要验证 offset 是否有效，应在 Demo 运行期间轻轻绕 X/Y 轴转动 IMU，观察 `gyro norm` 与 `offsetX / offsetY` 是否同步变化。
4. **`halfWindowMs` 不能过小**：100 Hz IMU 的采样间隔约 10 ms，若 `halfWindowMs=5`，窗口通常只取到 1 条样本，无法进行陀螺仪积分。推荐使用 `15~30 ms`，当前默认值为 `20 ms`。
5. **焦距参数影响 offset 大小**：`focalX / focalY` 越大，同样角度变化对应的像素偏移越大。Demo 中的 `1200 / 1200 pixel` 是测试值，真实项目应使用相机内参。
6. **Yaw 无绝对参考会漂移**：当前 ICM45686 是六轴 IMU，没有磁力计参与，yaw 长期漂移是正常现象。EIS 主要依赖短时间窗口内的陀螺仪积分，不依赖长期绝对 yaw。
7. **坐标轴符号需要按安装方向校准**：当前 `offsetX / offsetY` 的轴向映射是 Demo 级默认映射，真实接入图像防抖时需要根据 IMU 与相机的安装方向修正符号和轴向。
8. **运行前必须加载内核驱动**：如果 `/dev/icm45686` 不存在，应用程序会打开失败。请先确认 `inv_imu_driver.ko` 和 `icm45686_spi.ko` 已加载成功。
9. **不要混用旧目标文件**：切换交叉编译器或修改 CMake/Makefile 后，建议删除 `build/` 目录或执行 clean，避免 x86 与 aarch64 目标文件混用。

---

## 📚 文档索引

* `docs/DEMO-INSTRUCTIONS.md`：Demo 编译、运行、静态/动态防抖测试、CPU 与性能指标说明。
* `docs/IMPLEMENTATION.md`：当前 SPI 字符设备方案、应用层线程模型、ring buffer、EIS offset 计算流程和技术架构说明。
* `docs/LEARNING_GUIDE.md`：面向项目成员的学习路线、设计决策、常见 bug、调试经验和汇报话术。
* `docs/BUG_RECORD.md`：记录从设备树、GPIO、WHO_AM_I、寄存器地址、字节序到 EIS 窗口参数的完整问题排查过程。

---

## 🔧 后续扩展方向

1. 接入真实相机帧时间戳，将 `targetTimestampNs` 替换为真实曝光中心时间；
2. 根据相机内参替换 Demo 中的 `focalX / focalY`；
3. 根据 IMU 与相机实际安装方向标定 offset 轴向和符号；
4. 加入陀螺仪零偏估计，降低静态窗口内的积分误差；
5. 将 `offsetX / offsetY` 接入图像裁剪、平移或 RGA 图像处理模块，形成完整电子防抖闭环；
6. 如需更严格实时性，可将当前字符设备方案升级为 IIO + 中断 + kfifo 方案。
