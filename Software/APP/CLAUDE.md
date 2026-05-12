# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

RK3588-Omni-Sentinel 是一个基于瑞芯微 RK3588 的边缘端多传感器融合平台。所有应用层 C++ 组件均脱离 ROS，以静态库形式存在，通过 DMA-BUF 在 NPU/RGA/V4L2 硬件加速器之间实现零拷贝数据流转。

## 构建系统

本项目不是 ROS 工作空间。各组件独立编译，每个组件目录下有自己的 `build.sh` 和 `CMakeLists.txt`。全部使用 aarch64 交叉编译，目标平台为 RK3588 ARM64 Linux。

### 通用编译流程

每个组件目录下执行：

```bash
# 设置交叉编译器路径（按实际 SDK 路径修改 build.sh 中的 TOOL_CHAIN）
./build.sh

# 或手动执行:
mkdir -p build/ && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
```

产物输出到组件的 `install/` 目录。

### 交叉编译器

- `sentinel-lslidarer` / `sentinel-visioner`: `aarch64-buildroot-linux-gnu`
- `dma-buffer-pool`: `aarch64-linux-gnu` (Linaro GCC 6.3.1)

环境变量 `CROSS_COMPILE_PATH` 可覆盖默认工具链路径（仅 `sentinel-lslidarer` 的 build.sh 支持此环境变量）。

### 编译单个组件

```bash
cd sentinel-visioner && ./build.sh     # 视觉管线
cd sentinel-lslidarer && ./build.sh    # 激光雷达驱动
cd dma-buffer-pool && ./build.sh       # DMA 内存池
```

注意 `sentinel-visioner` 通过相对路径 `../dma-buffer-pool` 和 `../3rdparty/librga` 引用依赖，需保持目录树结构不变。

## 架构

### 组件层级

```
sentinel-visioner (相机视觉管线 + RGA 硬件加速)
  ├── dma-buffer-pool (DMA 内存池，O(1) 空闲链表分配)
  │     └── 3rdparty/allocator (DMA/DRM 底层分配器，OBJECT 库)
  │           ├── 3rdparty/librga (RGA 2D 加速库)
  │           └── 3rdparty/libdrm (DRM 头文件 + .so)
  ├── 3rdparty/librga (直接依赖)
  ├── 3rdparty/opencv (OpenCV 3.4.5, 预编译 aarch64)
  └── 3rdparty/rknpu2 (RKNN NPU 运行时，CMake 已就绪但未完全接入)

sentinel-lslidarer (激光雷达驱动，完全独立)
  └── Threads::Threads (唯一外部依赖)
```

### sentinel-lslidarer — 镭神 N10Plus 单线雷达驱动

三层结构：`SerialPort` → `RingBuffer(lock-free SWCR)` → `SentinelLslidarer`

- **SerialPort**: POSIX 串口（460800 baud, 8N1），阻塞读取，内存扫描 `0xA5 0x5A` 包头
- **RingBuffer**: SWCR 无锁环形缓冲区（`std::atomic` + memory_order 屏障），预分配 10 帧 × 540 点 ≈ 65KB
- **SentinelLslidarer**: 拥有 reader 线程，持续解码 N10Plus 108 字节固定包协议（双回波、CRC 校验），检测 360° 圈边界后提交完整帧。以 `CLOCK_MONOTONIC` 纳秒时间戳为索引，通过 `get_closest_frame(cameraTsNs, outFrame)` 线性扫描返回与相机时间戳最近的点云帧

唯一公共头文件: `include/sentinel_lslidarer.h`，API 类: `SentinelLslidarer`

### sentinel-visioner — 多路视觉流水线

"一分三"零拷贝扇出：一路 1080P V4L2 输入，经 RGA 硬件裂变为三路独立数据流：
1. RGB888 640×640 NPU 推理小图（带 Letterbox 灰边 + EIS 防抖偏移）
2. NV12 1280×720 OSD 叠加底图
3. NV12 1920×1080 原始推流副本

- **CameraContext**: 每路相机状态（包括 3 个 DmaBufferPool、2 个 ThreadSafeQueue、epoll fd 等）
- **捕获线程**: epoll 监听 V4L2 `VIDIOC_DQBUF`，只传递 dmaFd（无 mmap），连续 3 次 RGA 调度
- **消费者线程**: 通过 `wait_get_npuOSD()` / `wait_get_orig_copy_buffer()` 阻塞拉取，条件变量休眠（空闲 CPU 0.0%）。**必须调用对应的 `release_*()` 归还 DMA 缓冲区**
- **ThreadSafeQueue**: 泛型阻塞队列模板（`include/ThreadSafeQueue.h`），`std::mutex` + `std::condition_variable`

唯一公共头文件: `include/sentinel-visioner.h`，API 类: `SentinelVisioner`

### dma-buffer-pool — DMA 内存池

- O(1) 空闲链表（Free List）分配/归还模型
- `DmaBuffer_t` 包含 dmaFd、virtAddr、timestampUs、链表 next 指针
- "花名册"（`allBuffers_` vector）确保析构时零泄漏回收
- 通过 `3rdparty/allocator/dma_alloc.h` 调用底层 `dma_buf_alloc()`（DMA heap ioctl + mmap）

### 3rdparty/ 第三方库存放目录

| 库 | 内容 |
|---|---|
| `librga` | 瑞芯微 RGA 2D 加速，提供 `librga.a` + IM2D API 头文件 |
| `rknpu2` | RKNN NPU 运行时，提供 `librknnrt.so` |
| `opencv` | OpenCV 3.4.5 预编译 aarch64 完整包 |
| `libdrm` | DRM 头文件 + `libdrm.so`（allocator 通过 dlopen 使用） |
| `allocator` | DMA/DRM 底层分配器，CMake OBJECT 库（非 .a） |

## 关键约定

- 所有组件 C++14 标准，非 ROS，零外部运行时依赖
- 交叉编译器指向 aarch64，在 x86 开发机上运行编译
- 设备名硬编码：雷达 `/dev/sentinel_lidar`，相机 `/dev/video11`
- DMA 缓冲区遵循严格的"获取-使用-归还"生命周期，未归还将导致内核态内存枯竭和丢帧
- 时间戳统一使用 `CLOCK_MONOTONIC`，不同组件间通过此时间域实现传感器融合对齐
- 各开发者在自己的分支上工作，勿交叉修改他人负责的组件
