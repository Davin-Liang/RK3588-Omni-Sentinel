# SentinelLslidarer

**SentinelLslidarer** 是一个专为瑞芯微（Rockchip）边缘计算平台（如 RK3588）设计的、脱离 ROS 环境的镭神（LSLIDAR）N10Plus 单线激光雷达驱动库。

本库基于 C++14 编写，从原厂 ROS2 驱动中提取核心串口通信与协议解码逻辑，重构为无 ROS 依赖的独立静态库。内置 **SWCR（单写单读）无锁环形缓冲区** 与 **预分配内存池**，配合 `CLOCK_MONOTONIC` 时间戳，为相机-雷达时间同步融合（Sensor Fusion）提供高效、线程安全的点云帧检索接口。

---

## ✨ 核心架构与特性

* **完全脱离 ROS**：从 `lslidar_driver` 中剥离 `rclcpp`、`sensor_msgs`、`pcl_conversions` 等全部 ROS 依赖，仅保留 POSIX 串口 I/O 与 N10Plus 协议解码核心逻辑，零外部运行时开销。
* **SWCR 无锁环形缓冲区**：基于 `std::atomic<uint32_t>` 读写指针 + per-slot sequence 号的单写单读模型。写入端（雷达线程）与读取端（应用线程）无互斥锁竞争，无 Core Dump 风险。
* **预分配内存池**：连续分配 `N × 540 × 12 字节 ≈ 65 KB` 的点云存储空间，64 字节对齐，全程不产生堆碎片与动态分配。
* **时间戳融合接口**：`get_closest_frame(cameraTsNs, outFrame)` 以 `CLOCK_MONOTONIC` 纳秒级时间戳为索引，在环形缓冲区中线性扫描返回最近一帧点云，适配相机-雷达异构传感器融合管线。
* **原生 N10Plus 协议栈**：
  * 串口 460800 baud / 8N1
  * `0xA5 0x5A` 包头同步 + CRC 字节累加和校验
  * 双回波解码（每角度 2 回波 × 3 字节，含反射强度）
  * 方位角插值与 36000 项 sin/cos LUT 加速笛卡尔坐标转换
  * 距离滤波（0.15m - 50.0m）与角度裁剪（90° - 240° 盲区屏蔽）
* **独立读取线程**：`reader_loop_` 持续从串口阻塞读取、解码、检测圈边界（0° 跨越），累积完整 360° 扫描圈后提交至环形缓冲区，完全异步于应用线程。

---

## 🛠️ 环境依赖

在编译本工程之前，请确保环境中包含以下组件：

1. **CMake** (>= 3.4.1)
2. **交叉编译工具链** (如 `aarch64-buildroot-linux-gnu`)
3. **POSIX Threads** (`libpthread`，通常已随工具链内置)
4. **Linux 内核头文件** (提供 `termios.h`、`fcntl.h`、`posix_memalign` 等 POSIX 串口与内存接口)

> 本库 **不需要** ROS2、PCL、Boost、yaml-cpp 等任何外部依赖。

---

## 🚀 编译指南

### 1. 配置交叉编译器路径

**Bash**

```bash
export PATH=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot/bin:$PATH
```

### 2. 一键编译

使用项目提供的 `build.sh`，或手动执行：

**Bash**

```bash
mkdir -p build/
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
```

### 3. 编译产物

| 产物 | 路径 |
|------|------|
| 静态库 | `install/lib/libsentinel_lslidarer_lib.a` |
| 头文件 | `install/include/sentinel_lslidarer.h` |
| Demo 可执行程序 | `install/sentinel_lslidarer_demo` |
| udev 安装脚本 | `install/lidar_udev.sh` |

### 4. 设备准备（udev）

首次使用前，在目标板上执行 udev 脚本，将雷达串口固定映射为 `/dev/sentinel_lidar`：

**Bash**

```bash
sudo bash lidar_udev.sh
# 重新插拔雷达 USB，设备即固定为 /dev/sentinel_lidar
```

---

## 📖 快速上手

`SentinelLslidarer` 的标准生命周期为：**加载配置 -> 启动雷达 -> 获取最近帧 -> 停止雷达**。

**C++**

```cpp
#include "sentinel_lslidarer.h"

int main() {
    // 1. 创建实例并加载默认 N10Plus 配置
    SentinelLslidarer lidar;
    LidarConfig config;
    lidar.load_config(config);

    // 2. 启动雷达（打开串口、启动读取线程）
    if (!lidar.start()) {
        std::fprintf(stderr, "Failed to start lidar.\n");
        return -1;
    }

    // 3. 预分配帧接收缓冲区
    uint32_t maxPoints = lidar.max_points_per_frame();
    LidarFrame frame;
    frame.points = new LidarPoint[maxPoints];

    // 4. 以相机时间戳为索引，查找最近一帧点云
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t cameraTsNs = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
                        + static_cast<uint64_t>(ts.tv_nsec);

    if (lidar.get_closest_frame(cameraTsNs, frame)) {
        // 使用 frame.points[0 .. frame.pointsCount-1]
        for (uint32_t i = 0; i < frame.pointsCount; ++i) {
            float x = frame.points[i].x;
            float y = frame.points[i].y;
            float intensity = frame.points[i].intensity;
            // ... 业务逻辑
        }
    }

    // 5. 清理
    delete[] frame.points;
    lidar.stop();
    return 0;
}
```

### 运行 Demo

**Bash**

```bash
./sentinel_lslidarer_demo
```

Demo 将持续打印每帧时间戳、点数、首尾点坐标与强度、帧龄，按 `Ctrl+C` 安全退出。

---

## 📊 关键性能指标

以下数据基于 RK3588 平台，N10Plus 单线雷达 10Hz 扫频工况实测：

* **内存占用**：环形缓冲区（默认 10 帧）仅占用约 **65 KB** 物理内存，全程零动态分配，进程总 RES 仅 **2.2 MB**。
* **CPU 负载**：
  * **Reader Thread**：**6.0%**（RK3588 A55 小核），承担串口阻塞读取、CRC 校验、双回波解码、sin/cos LUT 查表与笛卡尔坐标转换。
  * **Main Thread**：**1.3%**，仅负责周期性查询与终端 printf 输出，空闲时休眠。
* **帧率稳定性**：**10 Hz**，圈边界检测与时间戳插值算法与原厂驱动一致。
* **查询帧龄 (age)**：稳态 **2-12 ms**，远低于扫频周期 (100ms)，满足实时融合对低延迟的要求。
* **线程安全**：SWCR 模型下，读写线程零互斥、零竞争。
* **Fd 防漏**：进程全局 Fd 数严格恒定在 **4 个**，无需担心资源泄漏。

> 详细测试数据与截图参见 [docs/DEMO-INSTRUCTIONS.md](docs/DEMO-INSTRUCTIONS.md)。

---

## ⚠️ 避坑与注意事项

1. **时间戳一致性**：本库使用 `CLOCK_MONOTONIC` 作为时间戳来源。调用 `get_closest_frame` 时传入的 `cameraTsNs` 必须同为 `CLOCK_MONOTONIC` 时间域，否则查找结果无意义。
2. **缓冲区预分配**：调用者需提前分配 `outFrame.points` 缓冲区，大小至少为 `max_points_per_frame()`（= 540）。若分配不足，`copy_slot` 将自动截断，不会越界写入，但会丢失尾部点云数据。
3. **单消费者约定**：`get_closest_frame` 设计为单一消费者线程调用。若需要多消费者并发读取，请在外部自行加互斥锁保护。
4. **串口权限**：首次使用需执行 `lidar_udev.sh` 安装 udev 规则，否则每次插拔设备名可能变化。默认设备路径为 `/dev/sentinel_lidar`。
5. **首圈丢弃**：雷达启动后，读取线程会自动跳过第一个不完整的半圈扫描，仅从完整的 360° 圈开始填充环形缓冲区。`start()` 返回后需等待雷达电机加速到全转速（约 2-3 秒）。
6. **雷达型号**：本库当前支持 **N10Plus** 串口模式。如需支持其他型号（M10、M10GPS、M10P、N10、N301），修改 `LidarConfig` 中的协议常量（波特率、包长、位偏移、解码函数）并实现对应 `decode_packet_` 分支即可。
