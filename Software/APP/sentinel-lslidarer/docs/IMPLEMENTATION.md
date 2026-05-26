# SentinelLslidarer — 技术实现文档

## 1. 概述

SentinelLslidarer 是从镭神原厂 ROS2 驱动（`lslidar_x10_driver`）中提取核心逻辑、重构为无 ROS 依赖的独立静态库。三层架构：`SerialPort`（POSIX 串口阻塞读取）→ `RingBuffer`（SWCR 无锁环形缓冲区）→ `SentinelLslidarer`（reader 线程 + 协议解码 + 时间戳融合接口）。仅依赖 `libpthread`，零外部运行时开销。

---

## 2. 架构总览

```
N10Plus 雷达 (460800 baud, 8N1)
    │
    │ 串口阻塞读取 (VTIME=5, VMIN=0)
    ▼
SerialPort
    │ read_packet(): 内存扫描 0xA5 0x5A 包头 → 108 字节固定包
    ▼
SentinelLslidarer::reader_loop_()
    ├── check_packet_validity_(): CRC 字节累加和校验
    ├── decode_packet_(): 方位角插值 + 双回波解码 (每包 32 点)
    ├── 圈边界检测: azimuth 从 ~360° 跳变到 ~0° → 完整一圈
    ├── is_point_valid_(): 距离滤波 (0.15m-50.0m) + 角度屏蔽
    ├── sin/cos LUT (36000 项, 0.01° 精度): 笛卡尔坐标转换
    └── commit_write(): 提交完整帧到环形缓冲区
         │
         ▼
RingBuffer (SWCR 无锁, 10 帧 × 540 点 ≈ 65KB)
    │
    │ get_closest_frame(cameraTsNs): 线性扫描最近帧
    ▼
应用线程 (lidar-camera-fusion / Demo)
```

**设计原则**:
- 串口使用阻塞 `read()` 而非 epoll/select，雷达持续输出 10Hz 数据，阻塞读取避免 CPU 空转
- 环形缓冲区使用 SWCR 模型，读写线程零互斥锁竞争
- 所有内存预分配（环形缓冲区 + LUT），运行时零堆分配
- `CLOCK_MONOTONIC` 统一时间戳，与相机帧对齐

---

## 3. 模块划分

| 文件 | 职责 |
|------|------|
| `include/sentinel_lslidarer.h` | 公共 API 头文件（`LidarPoint`、`LidarFrame`、`LidarConfig`、`RingBuffer`、`SerialPort`、`SentinelLslidarer`） |
| `src/sentinel_lslidarer.cpp` | 核心实现：生命周期、帧查询、`reader_loop_`（主循环） |
| `src/serial_port.cpp` | POSIX 串口操作：open/close/read_packet（阻塞读取 + 包头同步） |
| `src/ring_buffer.cpp` | SWCR 无锁环形缓冲区：预分配 + begin_write/commit_write/copy_slot |
| `src/m10p_protocol.cpp` | N10Plus 协议实现：LUT 构建、CRC 校验、双回波解码、距离/角度过滤 |
| `src/demo.cpp` | 功能验证 Demo |

---

## 4. 核心数据结构

### 4.1 LidarPoint

```cpp
struct LidarPoint {
    float x;         // X 坐标（米）
    float y;         // Y 坐标（米）
    float intensity; // 反射强度 (0-255)
};
```

### 4.2 LidarFrame

```cpp
struct LidarFrame {
    uint64_t timestampNs; // 帧时间戳（CLOCK_MONOTONIC，纳秒）
    uint32_t pointsCount; // 帧内有效点数
    LidarPoint* points;   // 点数据指针（调用者预分配，≥540 元素）
};
```

### 4.3 LidarConfig

```cpp
struct LidarConfig {
    std::string serialPort = "/dev/sentinel_lidar";
    int baudRate = 460800;
    int n10PlusHz = 10;
    static constexpr int kPacketLength = 108;      // N10Plus 固定包长
    static constexpr int kPacketPointsMax = 16;     // 每组角度数
    static constexpr int kPointsPerSweep = 540;     // 每圈最大点数
    float minRange = 0.15f;                         // 最小有效距离 (m)
    float maxRange = 50.0f;                         // 最大有效距离 (m)
    int angleDisableMin = 9000;                     // 角度屏蔽起始 (0.01° 单位)
    int angleDisableMax = 24000;                    // 角度屏蔽结束
    uint32_t ringBufferSize = 10;                   // 环形缓冲区帧数
};
```

### 4.4 RingBuffer 内部结构

```cpp
class RingBuffer {
    struct Slot {
        std::atomic<uint32_t> sequence{0};  // 写入序号（全局 writeIndex+1）
        uint64_t timestampNs{0};
        uint32_t pointsCount{0};
    };
    uint32_t capacity_;
    Slot* slots_;                    // 槽元数据数组
    LidarPoint* pointsPool_;         // 点云数据池（capacity × maxPointsPerSlot, 64 字节对齐）
    std::atomic<uint32_t> writeIndex_{0};  // 全局写索引（单调递增）
};
```

---

## 5. 线程模型

### 5.1 线程清单

| 线程 | 职责 | 生命周期 | 同步机制 |
|------|------|----------|----------|
| reader_loop_ | 串口阻塞读取 → 包头同步 → CRC 校验 → 双回波解码 → sin/cos LUT 笛卡尔转换 → 圈边界检测 → 提交环形缓冲区 | `start()` 创建，`stop()` 先关串口释放阻塞 read、再 join 销毁 | SWCR 无锁（`std::atomic` + `memory_order_release/acquire`） |
| 应用线程 | 周期性调用 `get_closest_frame()` 查询最近点云帧，线性扫描环形缓冲区 | 调用方管理 | SWCR 无锁读取（`sequence.load(acquire)` 保证数据可见性） |

### 5.2 读取线程生命周期

```
main thread                              reader_loop_
    |                                         |
    ├─ start()                               |
    |   ├─ serialPort_->open()               |
    |   ├─ build_lut_()                      |
    |   ├─ ringBuffer_ = new RingBuffer()    |
    |   ├─ running_ = true                   |
    |   └─ readerThread_ = new thread ───────▶ while(running_)
    |                                             ├─ serialPort_->read_packet()
    |                                             │   阻塞读取，VTIME=5 (0.5s 超时)
    |                                             │
    |                                             ├─ check_packet_validity_()
    |                                             ├─ decode_packet_()
    |                                             ├─ 圈边界检测 (azimuth 跳变)
    |                                             ├─ is_point_valid_() 过滤
    |                                             ├─ sin/cos LUT 笛卡尔转换
    |                                             └─ ringBuffer_->commit_write()
    |
    ├─ stop()                                |  (检测 false，退出循环)
    |   ├─ running_ = false                  |
    |   ├─ serialPort_->close() ─────────────▶ ::read() 被释放 (fd 关闭)
    |   ├─ readerThread_.join() ◀──────────── 线程退出
    |   ├─ serialPort_.reset()
    |   └─ ringBuffer_.reset()
```

**关键细节**:
- `stop()` 中先 `serialPort_->close()` 关闭 fd，释放阻塞在 `::read()` 中的 reader 线程，避免 join 永久等待
- 圈边界检测使用 `abs(diff) > 35000`（0.01° 单位），即方位角跳变超过 350° 判为新圈开始
- 首圈丢弃：`isFirstSweep=true` 时，跳过第一个不完整的半圈，只从圈边界开始累积完整圈
- 诊断输出：每 5000 次重试打印一次统计；连续 5 次超时且无有效包时输出 WARNING

### 5.3 线程安全规则

| 资源 | 规则 |
|------|------|
| `running_` | `std::atomic<bool>`，main thread 写入 false 后 close fd，reader thread 读取 |
| `ringBuffer_->writeIndex_` | reader thread 写入（fetch_add），应用线程读取（load），无竞争 |
| `ringBuffer_->slots_[i].sequence` | reader thread 写入前先写 pointsCount/timestampNs，sequence 作 release 屏障；应用线程 acquire 读取后保证之前的数据可见 |
| `ringBuffer_->pointsPool_` | reader thread 独占写入（无并发写），应用线程通过 copy_slot 读取（memcpy，无竞争） |
| `config_` | 仅在 `load_config()` 中写入（`start()` 前），运行时不修改 |

---

## 6. RingBuffer SWCR 无锁机制

### 6.1 核心原理

SWCR（Single Writer, Single Reader）模型下，一位写者一位读者，不需要互斥锁。通过 `std::atomic` + `memory_order` 屏障保证数据可见性：

```
写者 (reader_loop_)       读者 (get_closest_frame)
    │                           │
    ├─ begin_write()            ├─ write_index() (acquire)
    │   返回槽位指针             │   获取最新 writeIndex
    │                            │
    ├─ 直接写入 pointsPool_     ├─ 遍历 validCount 个槽位
    │   无锁，读者不碰此槽       │   copy_slot(slotIdx, outFrame)
    │                            │   ├─ sequence.load(acquire)
    ├─ commit_write()           │   │   ← 内存屏障，保证槽数据可见
    │   ├─ 写入 pointsCount     │   ├─ 读 timestampNs/pointsCount
    │   ├─ 写入 timestampNs     │   └─ memcpy 点云数据到 outFrame
    │   └─ sequence.store(      │
    │        seq+1, release)    │
    └─ writeIndex_++ (release)  │
```

### 6.2 内存屏障关键点

- **release 语义**：`commit_write` 中 `sequence.store(seq+1, release)` 保证之前对 `pointsCount`、`timestampNs`、`pointsPool_` 的所有写入对后续 acquire 可见
- **acquire 语义**：`copy_slot` 中 `sequence.load(acquire)` 与 release 配对，保证读到 sequence 后槽位数据的完整性
- **为什么不用 seq_cst**：SWCR 场景不需要全局顺序一致性，`release/acquire` 开销更低

---

## 7. 核心数据流

### 7.1 一个 N10Plus 数据包的生命周期

```
SerialPort::read_packet()
  │
  ├─ ::read(fd, chunk, 512)           // 阻塞读取，VTIME=5 (0.5s 字符间超时)
  │
  ├─ 内存扫描 0xA5 0x5A                // 在 chunk 中逐字节扫描包头
  │   找到后截取 108 字节 = 一个完整包
  │
  └─ 返回 108 字节原始数据

reader_loop_() 处理:
  │
  ├─ check_packet_validity_()
  │   ├─ 确认 data[0]==0xA5 && data[1]==0x5A
  │   └─ 前 107 字节累加和 == data[107]（CRC 校验）
  │
  ├─ clock_gettime(CLOCK_MONOTONIC)    // 记录包到达时间戳
  │
  ├─ decode_packet_()
  │   ├─ 读取起始方位角: data[5..6] % 36000
  │   ├─ 读取结束方位角: data[105..106] % 36000
  │   ├─ 计算角度增量: (endAngle - startAngle) / 15
  │   ├─ 遍历 16 组角度 × 2 回波 = 32 点:
  │   │   dataOff = 7 + group*6 + echo*3
  │   │   distance = ((data[off]<<8) | data[off+1]) * 0.001
  │   │   intensity = data[off+2]
  │   │   azimuth = startAngle + angleIncrement * group
  │   └─ 返回解码点数 (32)
  │
  ├─ 圈边界检测
  │   abs(decoded[i].azimuth - lastAzimuth) > 35000 → 新圈开始
  │
  ├─ is_point_valid_()
  │   0.15m < distance < 50.0m && azimuth not in [90°, 240°]
  │
  ├─ 笛卡尔转换
  │   x = distance * cosLut_[azimuth]
  │   y = distance * sinLut_[azimuth]
  │
  ├─ 累积到 sweepBuf[sweepCount++]
  │
  └─ 圈边界到达时:
      ├─ 插值计算圈结束时间戳
      │   sweepEndNs = packetTsNs - (packetTsNs - lastPacketTsNs)
      │              * (remainingAfterBoundary) / decodedCount
      ├─ ringBuffer_->commit_write(sweepEndNs, sweepCount)
      ├─ sweepCount = 0, sweepBuf = begin_write()
      └─ boundaryIdx 之后的点归入新一圈
```

### 7.2 get_closest_frame 查询流程

```
get_closest_frame(cameraTsNs, outFrame)
  │
  ├─ writeIdx = ringBuffer_->write_index()   // acquire 读最新写索引
  ├─ validCount = min(writeIdx, capacity)    // 有效槽位数
  │
  ├─ 线性扫描 validCount 个槽位:
  │   for i in [0, validCount):
  │     copy_slot(i, tmp)                    // 读帧时间戳
  │     delta = abs(tmp.timestampNs - cameraTsNs)
  │     if delta < bestDelta: bestIdx = i
  │
  └─ copy_slot(bestIdx, outFrame)            // 拷贝最佳帧
```

> **复杂度**：O(validCount) 线性扫描。validCount ≤ 10，实际开销可忽略。

---

## 8. SerialPort 串口操作

### 8.1 串口配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 波特率 | 460800 | N10Plus 固定波特率 |
| 数据位 | 8 | CS8 |
| 校验位 | 无 | PARENB 清除 |
| 停止位 | 1 | CSTOPB 清除 |
| 流控 | 无 | CRTSCTS 清除 |
| 原始模式 | cfmakeraw | 禁用行缓冲、回显、信号字符 |
| VTIME | 5 | 字符间超时 0.5 秒 |
| VMIN | 0 | 非阻塞式字节计数（配合 VTIME） |

### 8.2 read_packet 同步策略

```
read_packet(buffer, packetLen):
  │
  ├─ ::read(fd, chunk, 512)
  │   阻塞直到: 收到任意字节 OR VTIME 超时 0.5s
  │
  ├─ 返回值 ≤ 0:
  │   0  → 超时, 无数据（返回 0 告知调用者重试）
  │   <0 → EINTR 视为超时, 其他错误返回 -1
  │
  ├─ 逐字节扫描 0xA5 0x5A:
  │   找到 → 截取 108 字节 (N10Plus 固定包长)
  │   未找到 → 返回 0（丢弃此 chunk, 下次重试）
  │
  └─ 注意: 不做 poll/select, 纯阻塞 read
      10Hz × 108 字节 = 1KB/s, 阻塞开销可忽略
```

---

## 9. N10Plus 协议实现

### 9.1 数据包格式

```
Byte  0:     0xA5 (包头)
Byte  1:     0x5A (包头)
Byte  2-4:   保留
Byte  5-6:   起始方位角 (0.01° 单位, big-endian uint16)
Byte  7-102: 点数据 (16 组 × 每组 6 字节)
              每组: [echo0_dist_H, echo0_dist_L, echo0_intensity,
                     echo1_dist_H, echo1_dist_L, echo1_intensity]
Byte 103-104: 保留
Byte 105-106: 结束方位角 (0.01° 单位)
Byte 107:     CRC (前 107 字节累加和)
```

### 9.2 方位角插值

```
angleInterval = endAngle - startAngle
angleIncrement = angleInterval / (kPacketPointsMax - 1)   // /15

for group 0..15:
    currentAngle = (startAngle + angleIncrement * group) % 36000
    // 双回波共享同一方位角
```

### 9.3 sin/cos LUT

```cpp
void build_lut_() {
    for (int i = 0; i < 36000; ++i) {
        float rad = i * 0.01f * (M_PI / 180.0f);
        sinLut_[i] = std::sin(rad);
        cosLut_[i] = std::cos(rad);
    }
}
```

36000 项覆盖 [0°, 360°) 以 0.01° 精度，N10Plus 角度分辨率 0.33° 远高于此。576KB 常量内存换掉每秒 5400 次 sin/cos 调用。

---

## 10. 错误处理与容错

| 场景 | 处理方式 |
|------|---------|
| `::open(serialPort)` 失败 | `start()` 返回 false，`serialPort_.reset()` |
| `::read()` 返回 0（超时）| 计数 timeoutCount，连续 5 次无有效包时输出 WARNING；`usleep(1000)` 避免 CPU 空转 |
| `::read()` 返回 <0（非 EINTR）| 退出 reader loop |
| 包头扫描未找到 `0xA5 0x5A` | 计数 syncFailCount，丢弃当前 chunk |
| CRC 校验失败 | 计数 invalidPackets，丢弃该包 |
| `decode_packet_()` 返回 0 | 计数 invalidPackets |
| `start()` 时 `running_` 已为 true | 返回 false（不允许重复启动） |
| `load_config()` 时 `running_` 为 true | 返回 false（运行时不允修改配置） |
| `frame.points` 缓冲区小于 `max_points_per_frame()` | `copy_slot` 中 `min(count, maxPointsPerSlot_)` 自动截断，不越界 |

---

## 11. 性能特征

| 指标 | 数据 | 说明 |
|------|------|------|
| Reader Thread CPU | ~6.0% (A55 小核) | 串口阻塞读取 + CRC + 解码 + LUT + 笛卡尔转换 |
| Main Thread CPU | ~1.3% | 周期性查询 + printf |
| 环形缓冲区 | ~65 KB | 10 帧 × 540 点 × 12 字节，64 字节对齐 |
| 进程 RES | ~2.2 MB | 全预分配，无动态内存 |
| LUT 内存 | ~576 KB | 36000 × 4 字节 × 2（sin/cos），常量数据段 |
| 帧率 | 10 Hz 稳定 | 圈边界检测与原厂驱动一致 |
| 帧龄 | 2-12 ms | 远低于扫频周期 100ms |
| Fd 数量 | 恒定 4 个 | 串口(1) + 线程(1) + stdio(3) |

---

## 12. 代码入口点速查

| 功能 | 函数 | 文件:行号 |
|------|------|-----------|
| 配置加载 | `load_config()` | `sentinel_lslidarer.cpp:19` |
| 启动 | `start()` | `sentinel_lslidarer.cpp:27` |
| 停止 | `stop()` | `sentinel_lslidarer.cpp:53` |
| 帧查询 | `get_closest_frame()` | `sentinel_lslidarer.cpp:77` |
| 读取主循环 | `reader_loop_()` | `sentinel_lslidarer.cpp:132` |
| 圈边界检测 | `abs(diff) > 35000` | `sentinel_lslidarer.cpp:217` |
| 串口打开 | `SerialPort::open()` | `serial_port.cpp:22` |
| 包同步读取 | `SerialPort::read_packet()` | `serial_port.cpp:79` |
| 环形缓冲区写入 | `RingBuffer::commit_write()` | `ring_buffer.cpp:36` |
| 环形缓冲区读取 | `RingBuffer::copy_slot()` | `ring_buffer.cpp:53` |
| LUT 构建 | `build_lut_()` | `m10p_protocol.cpp:9` |
| CRC 校验 | `check_packet_validity_()` | `m10p_protocol.cpp:19` |
| 双回波解码 | `decode_packet_()` | `m10p_protocol.cpp:34` |
| 距离/角度过滤 | `is_point_valid_()` | `m10p_protocol.cpp:80` |
