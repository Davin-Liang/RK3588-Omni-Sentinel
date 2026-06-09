# RingBox Device Mapper 驱动 (dm-ringbox)

## 目录
- [1. 简介](#1-简介)
- [2. 目录结构](#2-目录结构)
- [3. 架构设计](#3-架构设计)
- [4. 编译指南](#4-编译指南)
- [5. 使用方法](#5-使用方法)
- [6. API 文档](#6-api-文档)
- [7. 性能优化说明](#7-性能优化说明)
- [8. 故障排查](#8-故障排查)

---

## 1. 简介

### 1.1 功能概述
本驱动是专为高性能日志/黑匣子存储设计的 Linux 自定义 Device Mapper (DM) 模块。它在内核层实现了一个对应用层完全透明的"物理扇区环形覆盖"逻辑。通过拦截下发的 `struct bio`（块 I/O 请求），动态篡改目标设备和物理扇区地址，实现高速循环写入。

### 1.2 核心特性
- **环形覆盖写入**: 自动实现循环缓冲区，无需应用层管理写入指针
- **零拷贝设计**: 仅修改 bio 元数据，数据直接通过 DMA 传输
- **事件驱动模型**: 无后台线程，空转时零 CPU 消耗
- **NVMe 优化**: 充分利用 NVMe SSD 的高 IOPS 和低延迟特性

### 1.3 适用场景
- AGV/自动驾驶数据记录
- 工业控制黑匣子
- 高频日志存储
- 实时数据缓冲

---

## 2. 目录结构

```
dm-ringbox/
├── include/
│   └── dm-ringbox.h       # 头文件（宏定义、数据结构、ioctl 接口）
├── src/
│   └── dm-ringbox.c       # 驱动源文件（核心实现）
├── Makefile               # 编译配置文件
└── README.md              # 本文档
```

### 2.1 文件说明

| 文件 | 说明 |
|-----|------|
| `include/dm-ringbox.h` | 头文件，包含宏定义、数据结构定义、ioctl 命令定义。用户空间程序也可使用此头文件 |
| `src/dm-ringbox.c` | 驱动核心实现，包含构造/析构函数、map 函数、ioctl 接口等 |
| `Makefile` | 编译配置，支持交叉编译和模块安装 |

---

## 3. 架构设计

### 3.1 模块架构图

```
+-------------------+
|   Application     |  <-- 用户空间应用程序
+-------------------+
         |
         v (标准块 I/O 调用)
+-------------------+
|  /dev/dm-X        |  <-- Device Mapper 逻辑设备
+-------------------+
         |
         v (bio 拦截)
+-------------------+
|  dm-ringbox       |  <-- 本驱动模块 (修改 bio 扇区地址)
+-------------------+
         |
         v (重定向后的 bio)
+-------------------+
|  NVMe Driver      |  <-- NVMe 块设备驱动
+-------------------+
         |
         v (DMA 直接传输)
+-------------------+
|  NVMe SSD         |  <-- 物理存储设备
+-------------------+
```

### 3.2 设计规范契合度

| 规范要求 | 实现方式 | 说明 |
|---------|---------|------|
| 数据零拷贝 | 仅修改 `bio->bi_iter.bi_sector` 和 `bio->bi_bdev` | 数据页 (`bio->bi_io_vec`) 从不拷贝 |
| 事件驱动 | 基于 `.map` 回调，无后台线程 | 空转时完全阻塞，不消耗 CPU |
| 硬件优先 | 数据传输由 NVMe 控制器 + DMA 完成 | CPU 仅处理地址映射逻辑 |
| 内存安全 | `kzalloc`/`kfree` 配对 + `goto` 错误处理 | 遵循内核内存管理最佳实践 |

### 3.3 环形缓冲区映射原理

```
逻辑地址空间 (应用层视角):
+------------------------------------------------------------------+
|  扇区 0      扇区 N-1     扇区 N      扇区 2N-1    扇区 2N   ...   |
+------------------------------------------------------------------+

物理环形缓冲区 (实际存储):
+------------------------------------------+
|  [0] -> [1] -> [2] -> ... -> [N-1] -> [0]|  (循环覆盖)
+------------------------------------------+
        ↑
        物理起始扇区

映射公式:
  physical_sector = physical_start + (logic_sector % ring_capacity)
```

---

## 4. 编译指南

### 4.1 环境要求

| 项目 | 要求 |
|-----|------|
| 开发主机 | Ubuntu 18.04/20.04 x86_64 |
| 目标平台 | RK3568/RK3588 (ARM64) |
| 内核版本 | Linux 5.10+ |
| 交叉编译器 | aarch64-none-linux-gnu-gcc 10.3+ |

### 4.2 配置 SDK 路径

编辑 `Makefile`，修改以下变量：

```makefile
SDK_PATH ?= /home/topeet/Linux_SDK/rk3568_linux_5.10
```

或在命令行指定：

```bash
make SDK_PATH=/your/sdk/path
```

### 4.3 编译命令

```bash
# 进入驱动目录
cd NVMe-SSD/drivers/dm-ringbox

# 编译驱动模块
make

# 安装到 build 目录
make install

# 清理编译产物
make clean

# 查看帮助
make help
```

### 4.4 编译产物

编译成功后，产物位于 `build/drivers/dm-ringbox/` 目录：

```
build/drivers/dm-ringbox/
├── include/
│   └── dm-ringbox.h   # 头文件（供用户空间程序使用）
└── modules/
    ├── dm-ringbox.ko  # 驱动模块文件
    └── Module.symvers # 符号导出文件
```

---

## 5. 使用方法

### 5.1 部署驱动

将编译好的 `dm-ringbox.ko` 复制到目标板：

```bash
# 通过 NFS/SD卡/SSH 等方式传输
scp dm-ringbox.ko root@192.168.1.100:/lib/modules/
```

### 5.2 加载驱动

```bash
# 加载驱动模块
insmod dm-ringbox.ko

# 验证加载成功
dmesg | grep ringbox
# 输出: ringbox: Module loaded successfully
```

### 5.3 创建 Device Mapper 设备

**命令格式:**
```bash
dmsetup create <设备名> --table "<起始扇区> <总扇区数> ringbox <底层设备> <物理起始> <环形大小>"
```

**参数说明:**

| 参数 | 说明 | 示例 |
|-----|------|------|
| 设备名 | 逻辑设备名称 | `agv_blackbox` |
| 起始扇区 | 逻辑设备起始扇区 | `0` |
| 总扇区数 | 逻辑设备总大小（可大于环形区） | `20971520` (10GB) |
| 底层设备 | NVMe 设备路径 | `/dev/nvme0n1` |
| 物理起始 | 环形区物理起始扇区 | `0` |
| 环形大小 | 环形区大小（扇区数） | `2097152` (1GB) |

**完整示例:**

```bash
# 创建一个 10GB 的逻辑设备，实际映射到 1GB 的环形缓冲区
dmsetup create agv_blackbox --table "0 20971520 ringbox /dev/nvme0n1 0 2097152"

# 查看设备状态
dmsetup status agv_blackbox

# 查看设备表
dmsetup table agv_blackbox
```

### 5.4 使用逻辑设备

```bash
# 设备创建后，会在 /dev/mapper/ 下生成对应设备节点
ls -l /dev/mapper/agv_blackbox

# 格式化（可选）
mkfs.ext4 /dev/mapper/agv_blackbox

# 挂载使用
mount /dev/mapper/agv_blackbox /mnt/blackbox

# 或直接作为裸设备写入
dd if=/dev/zero of=/dev/mapper/agv_blackbox bs=1M count=1000
```

### 5.5 删除设备

```bash
# 卸载（如果已挂载）
umount /mnt/blackbox

# 删除 Device Mapper 设备
dmsetup remove agv_blackbox
```

### 5.6 卸载驱动

```bash
# 确保所有 DM 设备已删除
dmsetup ls | grep ringbox

# 卸载驱动模块
rmmod dm_ringbox
```

---

## 6. API 文档

### 6.1 驱动参数

通过 `dmsetup create` 传入的参数：

| 序号 | 参数名 | 类型 | 说明 |
|-----|-------|------|------|
| 1 | device | string | 底层块设备路径 (如 `/dev/nvme0n1`) |
| 2 | physical_start | sector_t | 环形缓冲区物理起始扇区 |
| 3 | ring_capacity | sector_t | 环形缓冲区大小（扇区数，必须 > 0） |

### 6.2 ioctl 接口

驱动提供以下 ioctl 命令，用于用户空间程序查询状态和控制设备：

#### RINGBOX_IOC_GET_STATUS

**功能**: 获取环形缓冲区状态

```c
struct ringbox_status {
    sector_t physical_start;        // 物理起始扇区
    sector_t ring_capacity;         // 环形缓冲区总容量（扇区数）
    sector_t current_write_pos;     // 当前写入位置（扇区）
    uint64_t total_write_ops;       // 累计写操作次数
    uint64_t total_read_ops;        // 累计读操作次数
    uint64_t total_sectors_written; // 累计写入扇区数
};

int fd = open("/dev/mapper/agv_blackbox", O_RDONLY);
struct ringbox_status status;
ioctl(fd, RINGBOX_IOC_GET_STATUS, &status);
```

#### RINGBOX_IOC_RESET_WRITE

**功能**: 重置写入指针和统计信息

```c
ioctl(fd, RINGBOX_IOC_RESET_WRITE, NULL);
```

#### RINGBOX_IOC_SET_START

**功能**: 设置新的物理起始地址

```c
sector_t new_start = 1048576;  // 新起始扇区
ioctl(fd, RINGBOX_IOC_SET_START, &new_start);
```

### 6.3 状态查询

通过 `dmsetup status` 查看设备状态：

```bash
dmsetup status agv_blackbox
# 输出示例:
# agv_blackbox: 0 20971520 ringbox device:nvme0n1 start:0 capacity:2097152 write_pos:12345 write_ops:5678 read_ops:90 sectors:12345678
```

---

## 7. 性能优化说明

### 7.1 零拷贝实现

本驱动严格遵循零拷贝原则：

```
传统方式 (有拷贝):
  用户缓冲区 -> 内核缓冲区 -> 驱动缓冲区 -> DMA -> 磁盘
                        ↑
                     数据拷贝

零拷贝方式 (本驱动):
  用户缓冲区 -> bio -> NVMe驱动 -> DMA -> 磁盘
                                  ↑
                           仅修改扇区地址指针
```

### 7.2 事件驱动优势

| 对比项 | 轮询方式 | 事件驱动 (本驱动) |
|-------|---------|-----------------|
| CPU 占用 | 持续消耗 | 仅 I/O 时消耗 |
| 响应延迟 | 取决于轮询周期 | 立即响应 |
| 空转功耗 | 高 | 几乎为零 |

### 7.3 NVMe 性能建议

1. **使用高队列深度**: NVMe 支持多队列并行，应用层应使用异步 I/O (libaio/io_uring)
2. **避免小 I/O**: 合并小请求，减少请求开销
3. **对齐访问**: 确保写入地址和大小按扇区对齐

---

## 8. 故障排查

### 8.1 常见错误

| 错误信息 | 原因 | 解决方案 |
|---------|------|---------|
| `Invalid argument count` | dmsetup 参数不正确 | 检查参数数量是否为 3 个 |
| `Failed to allocate memory` | 内存分配失败 | 检查系统内存状态 |
| `Failed to acquire target block device` | 设备不存在或权限不足 | 检查设备路径和权限 |
| `Unknown ioctl command` | ioctl 命令不正确 | 检查命令号是否正确 |

### 8.2 调试方法

```bash
# 查看内核日志
dmesg | tail -50

# 查看驱动加载状态
lsmod | grep dm_ringbox

# 查看 Device Mapper 设备列表
dmsetup ls

# 查看块设备信息
lsblk
```

### 8.3 日志级别

驱动使用 `pr_info`/`pr_err`/`pr_warn` 输出日志，可通过以下方式调整内核日志级别：

```bash
# 查看当前日志级别
cat /proc/sys/kernel/printk

# 设置控制台日志级别
echo "8" > /proc/sys/kernel/printk
```

---

## 更新日志

| 版本 | 日期 | 更新内容 |
|-----|------|---------|
| v1.0.0 | 2026-04-15 | 初始版本，实现基本环形缓冲区功能 |

---

## 作者与许可

- **作者**: RK3568 Development Team
- **许可证**: GPL
- **版本**: 1.0.0
