# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此代码仓库中工作时提供指导。

## 项目概述

这是 RK3588 边缘计算平台的 Omni-Sentinel 项目软件。这是一个集成了摄像头、激光雷达、IMU 和 NVMe SSD 存储组件的多传感器数据采集和处理系统。项目专注于使用硬件加速（RGA、DMA 缓冲区）实现高性能的零拷贝数据处理。

## 架构概述

### 核心组件

1. **SentinelVisioner** (`APP/sentinel-visioner/`)
   - 高性能多路 MIPI 摄像头采集库
   - 使用 V4L2 的 DMA-BUF 导出模式实现零拷贝操作
   - 集成 RGA（2D 图形加速）进行硬件加速的 NV12->RGB888 转换
   - 为下游消费者（NPU 推理、流媒体）提供线程安全队列系统
   - 关键类：`SentinelVisioner`、`DmaBufferPool`、`ThreadSafeQueue`

2. **DMA Buffer Pool** (`APP/dma-buffer-pool/`)
   - DMA 缓冲区的自定义内存管理
   - 防止高帧率下的内存碎片化
   - 零拷贝管道的重要组成部分

3. **NVMe Data Manager** (`APP/NVMe-SSD/`)
   - 管理高速 NVMe SSD 存储
   - 支持多种数据类型：VIDEO_FRONT（前视）、VIDEO_REAR（后视）、LIDAR（激光雷达）、IMU
   - 基于头部的数据打包和线程安全队列系统
   - 关键类：`NVMeDataManager`、带时间戳的数据结构

4. **第三方依赖**
   - **librga**：瑞芯微的 2D 图形加速库
   - **allocator**：自定义 DMA 分配器
   - **libdrm**：GPU 操作的直接渲染管理器
   - **opencv**：用于演示可视化（可选）

## 构建系统

### 交叉编译设置

项目使用 CMake 进行 ARM64（aarch64）交叉编译。所有组件需要使用相同的工具链构建。

1. **设置工具链路径** 在 `build.sh` 中：
   ```bash
   TOOL_CHAIN=/path/to/your/aarch64-buildroot-linux-gnu_sdk-buildroot
   ```

2. **构建命令**：
   ```bash
   cd APP/sentinel-visioner
   chmod +x build.sh
   ./build.sh
   ```

3. **手动构建**（如果需要）：
   ```bash
   mkdir -p build
   cd build
   cmake ..
   make -j4
   make install
   ```

### 关键构建说明

- 所有组件使用 C++14 标准
- 配置了 RPATH 以实现运行时库发现
- 首选静态库以保证部署稳定性
- OpenCV 默认在 CMakeLists.txt 中被注释掉（可选）

## 开发工作流

### 添加新摄像头

1. 包含头文件：`#include "sentinel-visioner.h"`
2. 创建 SentinelVisioner 实例并添加摄像头：
   ```cpp
   SentinelVisioner visioner;
   visioner.add_camera("/dev/videoX", width, height, bufferCount, camNum);
   visioner.camera_stream_ctrl(camNum, true);
   ```
3. 使用 `wait_get_rga_buffer()` 和 `release_rga_buffer()` 启动消费者线程

### 缓冲区管理关键点

1. **必须在使用后归还缓冲区**：`visioner.release_rga_buffer(camNum, buffer)`
2. 不归还会导致 RGA 内存池耗尽和丢帧
3. 直接使用 DMA 文件描述符给 NPU/GPU/编码器硬件

### 数据存储管道

1. 数据流向：摄像头 → RGA（NV12->RGB888）→ 消费者 → NVMe
2. 每种数据类型在 NVMeDataManager 中有独立的队列
3. 时间戳同步对多传感器融合至关重要

## 目录结构

```
Software/
├── APP/
│   ├── sentinel-visioner/     # 摄像头采集和 RGA 处理
│   ├── dma-buffer-pool/        # DMA 内存管理
│   ├── NVMe-SSD/              # 高速存储管理
│   └── 3rdparty/              # 第三方库（RGA、分配器等）
├── Driver/                    # 内核驱动（dm-ringbox）
└── [其他组件...]
```

## 测试

- `sentinel-visioner` 中的 Demo1 显示基本的摄像头采集和 FPS 监控
- 需要以 root 权限运行以访问 `/dev/videoX`
- 查看 FPS 输出来验证性能：`[Camera 0] FPS: 15.23 | 耗时: 1000 ms | 获取帧数: 1523`

## 重要说明

1. **root 权限** 是进行摄像头和 DMA 操作所必需的
2. **零拷贝架构** 意味着没有 CPU 数据拷贝 - 直接使用 DMA 缓冲区工作
3. **线程安全** 由内部处理 - 不要在线程间共享缓冲区
4. **RGA 处理** 输出带字母框填充的 RGB888（通常是 640x640）
5. **内存管理** 至关重要 - 严格遵循缓冲区归还模式

## 常用命令

```bash
# 构建 sentinel-visioner
cd APP/sentinel-visioner && ./build.sh

# 清理构建
rm -rf build/ && mkdir build && cd build && cmake .. && make clean

# 检查安装输出
ls -la install/
```

## claude工作遵循要求
1. **Think Before Coding**
不确定就问，别猜
有多种理解就都列出来，别自己偷偷选
有更简单的方案要主动说
2. **Simplicity First**
只做需求内的事，不搞“未来可能用到的”功能
能50行搞定就别写200行

