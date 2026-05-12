# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目性质

这是一个**完全脱离 ROS** 的镭神 N10Plus 单线激光雷达驱动静态库。从原厂 ROS2 驱动抽取核心串口通信与协议解码逻辑，零 ROS 运行时依赖。

## 构建

```bash
# 交叉编译 aarch64 (RK3588)
export CROSS_COMPILE_PATH=/path/to/aarch64-buildroot-linux-gnu_sdk-buildroot
./build.sh

# 手动:
mkdir -p build/ && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
```

唯一依赖: `libpthread`（工具链内置）。

产物:
- `install/lib/libsentinel_lslidarer_lib.a` — 静态库
- `install/include/sentinel_lslidarer.h` — 公共头文件
- `install/sentinel_lslidarer_demo` — Demo 可执行程序
- `install/lidar_udev.sh` — udev 规则安装脚本

## 架构

三层结构，自底向上:

**SerialPort** (`src/serial_port.cpp`) — POSIX 串口 I/O
- 打开 `/dev/sentinel_lidar`，460800 baud / 8N1
- 阻塞 `read()` + 内存扫描 `0xA5 0x5A` 包头
- 返回固定 108 字节 N10Plus 数据包

**RingBuffer** (`src/ring_buffer.cpp`) — SWCR 无锁环形缓冲区
- 单写（雷达线程）单读（应用线程）模型
- `std::atomic<uint32_t>` 读写指针 + per-slot sequence 号 + `memory_order_release`/`acquire` 屏障
- 预分配 64 字节对齐内存池（10 帧 × 540 点 ≈ 65 KB）

**SentinelLslidarer** (`src/sentinel_lslidarer.cpp`) — 主驱动类
- 拥有 reader 线程：阻塞读包 → CRC 校验 → 双回波解码 → sin/cos LUT 查表 → 笛卡尔坐标转换 → 圈边界检测 → 提交完整帧
- 提供 `get_closest_frame(cameraTsNs, outFrame)`: 以 `CLOCK_MONOTONIC` 纳秒时间戳线性扫描返回最近帧
- 距离滤波 0.15m–50.0m，角度裁剪 90°–240° 盲区屏蔽

**N10Plus 协议解码** (`src/m10p_protocol.cpp`)
- 36000 项 sin/cos 预计算 LUT
- CRC 字节累加和校验
- 32 点/包（16 角度组 × 2 回波，含反射强度）

## Demo 运行

```bash
# 先在目标板安装 udev 规则（一次）
sudo bash lidar_udev.sh

# 运行
./sentinel_lslidarer_demo
```

## 性能指标 (RK3588, N10Plus 10Hz)

- Reader 线程 CPU: 6.0%, 主线程: 1.3%
- 进程 RES: 2.2 MB, 帧龄: 2-12 ms
- FD 数恒定 4 个，零泄漏
