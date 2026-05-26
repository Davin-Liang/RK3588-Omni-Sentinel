# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 组件概述

`sentinel-lslidarer` 是脱离 ROS 的镭神 N10Plus 单线激光雷达独立驱动库，C++14 标准。从原厂 ROS2 驱动（lslidar_x10_driver）中提取核心串口通信与协议解码逻辑，重构为无 ROS/PCL/Boost 依赖的静态库。

## 三层架构

```
SerialPort (POSIX 串口, 460800 baud, 8N1, 阻塞读取)
    └── RingBuffer (SWCR 无锁环形缓冲区, 10 帧 × 540 点 ≈ 65KB)
         └── SentinelLslidarer (reader 线程 + N10Plus 协议解码 + get_closest_frame)
```

- **SerialPort**: `open()` 配置 raw mode + VTIME=5 + VMIN=0；`read_packet()` 内存扫描 0xA5 0x5A 包头，固定 108 字节包长
- **RingBuffer**: SWCR 单写单读模型，`std::atomic` + `memory_order_release/acquire` 屏障；`begin_write()` / `commit_write()` / `copy_slot()` 三接口
- **SentinelLslidarer**: 拥有 reader 线程，持续解码 N10Plus 协议（双回波、方位角插值、CRC 校验、sin/cos LUT），检测 360° 圈边界后提交完整帧

## 构建系统

```bash
cd sentinel-lslidarer
./build.sh                    # 支持 CROSS_COMPILE_PATH 环境变量
# 产物: install/lib/libsentinel_lslidarer_lib.a
#       install/include/sentinel_lslidarer.h
#       install/sentinel_lslidarer_demo
```

交叉编译器: `aarch64-buildroot-linux-gnu`（通过 `build.sh` 中的 `TOOL_CHAIN` 变量设置）。

## 关键约定

- 唯一公共头文件: `include/sentinel_lslidarer.h`，API 类: `SentinelLslidarer`
- 时间戳统一使用 `CLOCK_MONOTONIC` 纳秒
- 调用者需预分配 `frame.points` 缓冲区，大小 ≥ `max_points_per_frame()`（= 540）
- `start()` 返回后需等待 2-3 秒（雷达电机加速 + 首圈丢弃）
- 串口默认路径 `/dev/sentinel_lidar`（通过 `lidar_udev.sh` 创建符号链接）
- 零外部运行时依赖，仅需 `libpthread`

## 编码规范

遵循项目根目录 `CLAUDE.md` 中定义的编码规范（C++14、snake_case 方法、PascalCase 类、尾部下划线私有成员等）。

## 文档

- README.md — 项目概览（功能概述、快速上手、性能指标、注意事项）
- docs/IMPLEMENTATION.md — 实现文档（架构、线程模型、数据流、代码入口点）
- docs/LEARNING_GUIDE.md — 学习指南（三层递进：做了什么 / 为什么 / 踩坑）
- docs/DEMO-INSTRUCTIONS.md — 演示程序说明（Demo 01: 环形缓冲区与时间戳检索）
- BUG_RECORD.md — 问题记录（7 条）
