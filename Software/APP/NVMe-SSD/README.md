# NVMe SSD 测试 Demo

**NVMe SSD 测试 Demo** 是用于测试 NVMeDataManager 类功能的演示程序。该程序模拟多传感器数据（摄像头、激光雷达、IMU）的写入，并测量写入性能是否符合要求（每个接口调用耗时 < 1ms，CPU利用率 < 5%）。

---

## ✨ 核心特性

* **多传感器模拟**：模拟摄像头（15 FPS）、激光雷达（10 Hz）、IMU（100 Hz）数据
* **性能测试**：精确测量每个写入操作的耗时
* **实时监控**：显示队列大小和运行时间统计
* **零拷贝优化**：使用 DMA 缓冲区实现高性能数据传输

---

## 🛠️ 环境依赖

在编译本工程之前，请确保环境中包含以下组件：

1. **CMake** (>= 3.4.1)
2. **交叉编译工具链** (如 `aarch64-buildroot-linux-gnu`)
3. **NVMe 驱动**：确保内核已加载 NVMe 驱动
4. **开发权限**：需要 root 权限访问 NVMe 设备

---

## 🚀 编译指南

本工程支持交叉编译模式。项目根目录下提供了一键编译脚本（`build.sh`）。

### 1. 配置交叉编译器路径

在编译脚本中，将 `TOOL_CHAIN` 修改为你本机的实际 SDK 路径：

**Bash**

```bash
# 修改为你实际的交叉编译工具链路径
TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
```

### 2. 执行编译命令

使用以下脚本进行构建、编译并安装（安装目录默认为工程根目录下的 `install/`）：

**Bash**

```bash
set -e

TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
GCC_COMPILER=$TOOL_CHAIN/bin/aarch64-buildroot-linux-gnu

export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

# 创建并进入构建目录
mkdir -p build/
cd build/

# 生成 Makefile
cmake ..

# 多线程编译
make -j4

# 安装动态库与可执行文件
make install
cd -

echo "Build Success! Output is in install/ directory."
```

---

## 📖 使用说明 (快速上手)

### 运行 Demo

```bash
# 需要 root 权限运行
sudo ./nvme_demo
```

### 程序功能说明

1. **自动初始化**：程序启动时会自动初始化 NVMeDataManager
2. **多线程写入**：
   - 摄像头数据：每 67ms 生成一帧（15 FPS）
   - 激光雷达数据：每 100ms 生成一帧（10 Hz）
   - IMU 数据：每 10ms 生成一组（100 Hz）
3. **性能监控**：实时显示每个写入操作的耗时
4. **统计信息**：每 5 秒输出一次队列大小和运行时间

### 测试通过标准

- **每个类接口调用耗时** < 1ms
- **CPU利用率** < 5%

---

## 📊 输出示例

程序运行时将显示类似以下输出：

```
初始化 NVMe 数据管理器...
初始化成功！
开始测试数据写入...
摄像头帧间隔: 67ms (15 FPS)
激光雷达帧间隔: 100ms (10 Hz)
IMU 数据间隔: 10ms (100 Hz)
----------------------------------------
[摄像头] 帧 #1 写入耗时: 0.45ms
[IMU] 数据 #1 写入耗时: 0.12ms
[IMU] 数据 #2 写入耗时: 0.08ms
[IMU] 数据 #3 写入耗时: 0.15ms
[激光雷达] 帧 #1 写入耗时: 0.67ms
[摄像头] 帧 #2 写入耗时: 0.39ms
[IMU] 数据 #4 写入耗时: 0.11ms
[IMU] 数据 #5 写入耗时: 0.09ms
[IMU] 数据 #6 写入耗时: 0.13ms
[IMU] 数据 #7 写入耗时: 0.10ms
[IMU] 数据 #8 写入耗时: 0.14ms
[IMU] 数据 #9 写入耗时: 0.07ms
[IMU] 数据 #10 写入耗时: 0.12ms
[激光雷达] 帧 #2 写入耗时: 0.58ms
[摄像头] 帧 #3 写入耗时: 0.42ms

[统计] 运行时间: 5s, 队列大小: 0
----------------------------------------
```

---

## ⚠️ 注意事项与避坑指南

1. **Root 权限**：访问 NVMe 设备需要 root 权限
2. **驱动加载**：确保 NVMe 驱动已正确加载 (`lsblk` 查看 nvme 设备)
3. **设备路径**：程序默认使用 `/dev/nvme0n1`，如有需要请修改 NVMeDataManager.cpp 中的设备路径
4. **性能要求**：
   - 单次写入耗时必须小于 1ms
   - CPU 使用率必须保持在 5% 以下
5. **磁盘空间**：确保 NVMe SSD 有足够的磁盘空间用于测试

---

## 🔧 自定义配置

### 修改数据生成参数

在 `nvme_demo.cpp` 中可以修改以下参数：

```cpp
// 测试参数
const int camera_interval_ms = 67;  // 摄像头帧间隔
const int lidar_interval_ms = 100;  // 激光雷达帧间隔  
const int imu_interval_ms = 10;     // IMU 数据间隔
```

### 修改数据大小

```cpp
// 模拟生成摄像头图像数据
void generate_camera_frame(std::vector<uint8_t>& frame_data, 
                          int width = 1920,        // 图像宽度
                          int height = 1080)      // 图像高度

// 模拟生成激光雷达点云数据
void generate_lidar_points(std::vector<uint8_t>& points_data, 
                          int point_count = 4096)  // 点云数量
```

---

## 🚨 错误排查

### 常见错误及解决方案

1. **Permission denied**
   ```
   解决：使用 sudo 运行程序
   ```

2. **No such file or directory: '/dev/nvme0n1'**
   ```
   解决：检查 NVMe 驱动是否加载
   $ lsblk | grep nvme
   $ modprobe nvme
   ```

3. **High CPU usage**
   ```
   解决：减少数据生成频率或优化数据结构
   ```

4. **Write timeout**
   ```
   解决：检查 NVMe 设备是否繁忙，增加缓冲区大小
   ```

---

## 📈 性能优化建议

1. **批量写入**：考虑合并多个传感器数据批量写入，减少 I/O 操作
2. **异步处理**：使用更大的队列减少等待时间
3. **缓冲区优化**：根据实际数据大小调整缓冲区大小
4. **内存对齐**：确保数据缓冲区对齐到 512 字节边界
5. **CPU 亲和性**：将写入线程绑定到特定 CPU 核心

---

## 🔴 RK3588 视频导出测试结果

### 测试环境

| 项目 | 规格 |
|------|------|
| 平台 | RK3588 (ELF2) |
| 内核 | Linux 5.10.209 |
| 存储 | NVMe SSD 128GB (裸盘，无分区) |
| DM 驱动 | dm-ringbox v1.0 (环形缓冲区 100MB) |
| 编译器 | aarch64-none-linux-gnu-gcc 10.3 |

### 运行命令

```bash
# ============================================
# 完整实验流程
# ============================================

# 1. 加载驱动
insmod /userdata/Nvme-SSD/dm-ringbox.ko

# 2. 分配环形缓冲区 (100MB = 204800 扇区 × 512B)
LD_PRELOAD=/userdata/Nvme-SSD/lib/librpl_malloc.so \
LD_LIBRARY_PATH=/userdata/Nvme-SSD/lib \
./dmsetup create agv_blackbox --table "0 204800 ringbox /dev/nvme0n1 0 204800"

# 3. 验证设备已创建
ls -la /dev/mapper/agv_blackbox

# 4. 运行视频导出测试
./nvme_benchmark export

# 5. 验证导出视频
ls -lh /tmp/front_camera_clip.mp4 /tmp/rear_camera_clip.mp4

# 6. 移除缓冲区
LD_PRELOAD=/userdata/Nvme-SSD/lib/librpl_malloc.so \
LD_LIBRARY_PATH=/userdata/Nvme-SSD/lib \
./dmsetup remove agv_blackbox

# 7. 卸载驱动
rmmod dm-ringbox
```

### 测试输出

```
==========================================
  export_trigger_video_clip 功能测试
==========================================
图像格式: RGB888 1920x1080 (5.93 MB/帧)
帧率: 15 FPS | 时间窗口: 回溯 10s
测试时长: 35s
颜色策略: 每秒切换 | 前视(红→绿→蓝) | 后视(黄→黑→紫)
摄像头: 前视 + 后视 双路同时写入
----------------------------------------
Step 1/4: CPU 预生成6张纯色图片...
  [0] 红 RGB(255,0,0)
  [1] 绿 RGB(0,255,0)
  [2] 蓝 RGB(0,0,255)
  [3] 黄 RGB(255,255,0)
  [4] 黑 RGB(0,0,0)
  [5] 紫 RGB(255,0,255)

Step 2/4: 循环写入双路模拟视频帧 35s...
[#1   t+0s]  前视=红 后视=黄  ts=3109994140335  17.6ms
[#2   t+0s]  前视=红 后视=黄  ts=3110061268384  11.5ms
[#3   t+0s]  前视=红 后视=黄  ts=3110127500515   9.6ms
[#4   t+0s]  前视=红 后视=黄  ts=3110193956480   9.4ms
[#5   t+0s]  前视=红 后视=黄  ts=3110260324945   9.3ms
[#75  t+4s]  前视=绿 后视=黑  ts=3114921206872   9.5ms
[#150 t+9s]  前视=红 后视=黄  ts=3119917728041   9.4ms
[#225 t+14s] 前视=蓝 后视=紫  ts=3124907722292   9.4ms
[#300 t+19s] 前视=绿 后视=黑  ts=3129895766168   9.6ms
[#375 t+24s] 前视=红 后视=黄  ts=3134883540544   9.4ms
[#450 t+29s] 前视=蓝 后视=紫  ts=3139874399588   9.4ms
[#525 t+34s] 前视=绿 后视=黑  ts=3144866939506   9.5ms
写入完毕: 525 帧 | 前视=525 后视=525 失败=0

Step 3/4: NVMe 写入线程已关闭，准备导出...
Step 4/4: 导出视频片段 (回溯 10.0s)...
  --- 前视摄像头 ---
  触发时间戳: 3142937603687 (帧索引: 495/525)
  触发时颜色: 红
  Exported 151 frames (10.1s) to /tmp/front_camera_clip.mp4
  前视导出: 成功 ✓ | 耗时=6583ms | 文件=/tmp/front_camera_clip.mp4
  --- 后视摄像头 ---
  触发时间戳: 3142937603687 (帧索引: 495/525)
  触发时颜色: 黄
  Exported 151 frames (10.1s) to /tmp/rear_camera_clip.mp4
  后视导出: 成功 ✓ | 耗时=8143ms | 文件=/tmp/rear_camera_clip.mp4
```

### 测试结论

| 指标 | 结果 | 状态 |
|------|------|:--:|
| 写入帧数 | 525 帧 (前视+后视各525) | ✅ |
| 写入失败 | 0 | ✅ |
| 单帧写入耗时 | 9.3~17.6ms (稳态 ~9.4ms) | ✅ |
| 前视导出 | 151 帧, 10.1s, 6583ms | ✅ |
| 后视导出 | 151 帧, 10.1s, 8143ms | ✅ |
| h264_rkmpp 硬件编码 | 正常工作 | ✅ |
| RGB888→NV12→MP4 管线 | 正常 | ✅ |
| 环形缓冲区(DM ringbox) | 正常工作 | ✅ |