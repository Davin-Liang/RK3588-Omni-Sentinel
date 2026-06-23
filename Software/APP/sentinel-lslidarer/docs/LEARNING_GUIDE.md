# SentinelLslidarer — 学习指南

## 目标

面试时能说清：做了什么、为什么这么设计、踩过什么坑。

---

## 第一层：能说清"做了什么"（面试讲项目用）

### 一句话概括

> 脱离 ROS 的镭神 N10Plus 单线激光雷达独立驱动库，三层架构（POSIX 串口 → SWCR 无锁环形缓冲区 → 协议解码 + 时间戳融合），预分配 65KB 内存池，reader 线程仅 6.0% CPU，10Hz 稳定帧率，帧龄 2-12ms。

### 架构图（能画出来）

```
N10Plus 雷达 (460800 baud, 8N1)
    │
    ▼
SerialPort (阻塞 read, 内存扫描 0xA5 0x5A)
    │
    ▼
reader_loop_ (独立线程)
    ├── CRC 校验 (字节累加和)
    ├── 双回波解码 (每包 32 点)
    ├── 圈边界检测 (azimuth 跳变 > 350°)
    ├── sin/cos LUT 笛卡尔转换 (36000 项, 0.01°)
    └── commit_write → RingBuffer
         │
         ▼
RingBuffer (SWCR 无锁, 10 帧 × 540 点 ≈ 65KB)
    │
    │ get_closest_frame(cameraTsNs) 线性扫描
    ▼
应用线程 (融合 / 预览)
```

### 关键代码（背下来）

```cpp
// 5 步用法，面试能张口就来
SentinelLslidarer lidar;
LidarConfig config;
lidar.load_config(config);                              // 1. 加载配置

lidar.start();                                          // 2. 启动

LidarFrame frame;
frame.points = new LidarPoint[lidar.max_points_per_frame()];  // 3. 预分配

struct timespec ts;
clock_gettime(CLOCK_MONOTONIC, &ts);
uint64_t cameraTsNs = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

if (lidar.get_closest_frame(cameraTsNs, frame)) {       // 4. 查询最近帧
    // frame.points[0 .. frame.pointsCount-1]
}

delete[] frame.points;
lidar.stop();                                           // 5. 停止
```

---

## 第二层：能解释"为什么这么设计"（面试追问用）

### 决策 1：为什么用 SWCR 无锁环形缓冲区而不是 std::mutex 队列？

| 对比维度 | std::mutex + std::queue | SWCR 无锁环形缓冲区（我们的方案） |
|----------|------------------------|----------------------------------|
| 锁竞争 | 读写竞争同一把锁，虽然 10Hz 下竞争不激烈，但有系统调用开销 | `std::atomic` + `memory_order_release/acquire`，零系统调用 |
| 内存分配 | push 可能触发动态分配 | 预分配 65KB，64 字节对齐，全程零分配 |
| 适用前提 | 通用，无约束 | 单写单读（reader 线程写，应用线程读），雷达场景天然满足 |

**内存屏障关键点**：写者先写完 `pointsCount`、`timestampNs`、点云数据，最后 `sequence.store(seq+1, release)` 做释放屏障；读者先 `sequence.load(acquire)` 做获取屏障，然后读数据。release/acquire 配对保证写者的数据对读者可见。

**面试话术**: "SWCR 模型的前提是单写单读——雷达只有 reader 线程写，融合线程读。这个约束允许我们用 memory_order 屏障替代 mutex，做到真正的零等待。预分配 10 帧 × 540 点 = 65KB，编译期确定，运行时不碰堆。"

---

### 决策 2：为什么用预分配内存池而不是动态分配？

| 对比维度 | 动态分配（每帧 new/delete） | 预分配内存池（我们的方案） |
|----------|--------------------------|--------------------------|
| 堆碎片 | 10Hz × 540 点 × 12 字节 = 65KB/s，长期运行碎片化严重 | 一次 `posix_memalign(64)` 分配 65KB，之后永不碰堆 |
| RES 内存 | 波动大，依赖堆状态 | 恒定 2.2 MB，可以精确预估 |
| 分配耗时 | new 每次几百 ns，累积可观 | O(1) 数组索引，零开销 |

**面试话术**: "嵌入式上最忌讳的就是运行时动态分配。我们用编译期常量确定最大点数 540，一次分配 65KB，之后 10 帧循环覆盖写。进程 RES 只有 2.2MB——面试官可以记住这个数字，这是我们刻意控制的结果。"

---

### 决策 3：为什么用 36000 项 sin/cos LUT 而不是运行时三角函数？

| 对比维度 | 运行时 sin/cos | 预计算 LUT（我们的方案） |
|----------|---------------|----------------------|
| 计算量 | 每次几十个 CPU 周期 | 一次数组索引，单周期 |
| 每秒调用 | 10Hz × 540 点 = 5400 次 | 5400 次数组查表 |
| 内存代价 | 0 | 36000 × 4 字节 × 2 = 288KB，编译期初始化 |

**面试话术**: "每秒 5400 次笛卡尔转换，每次都要 sin/cos。用 288KB 的 LUT 换掉每秒 5400 次三角函数调用——以空间换时间，在嵌入式上是非常划算的买卖。0.01° 精度远超 N10Plus 的 0.33° 角度分辨率，绰绰有余。"

---

### 决策 4：为什么用阻塞 read 而不是 epoll 或 select？

| 对比维度 | epoll / select | 阻塞 read（我们的方案） |
|----------|---------------|----------------------|
| CPU 使用 | 需要事件循环 + 超时管理 | 内核挂起线程，零 CPU |
| 代码复杂度 | epoll_create + epoll_ctl + epoll_wait 循环 | 一行 `::read()` |
| 适用场景 | 多路 I/O 复用（如多摄像头） | 单路持续数据流（雷达 10Hz 不间断输出） |

**面试话术**: "雷达和其他传感器不一样——它一上电就不停地吐数据，10Hz 460800 baud，数据流是持续的。这种情况下阻塞 read 最省事：内核在没数据时挂起线程，数据到了直接唤醒，零 CPU 开销。epoll 的优势在多路复用，但雷达只有一路串口，用 epoll 是杀鸡用牛刀。"

---

### 决策 5：为什么 stop() 要先关串口再 join 线程？

```cpp
void stop() {
    running_.store(false);          // ① 通知退出
    serialPort_->close();           // ② 先关 fd，释放阻塞的 read()
    readerThread_.join();           // ③ 再等线程退出
    ringBuffer_.reset();            // ④ 最后释放内存
}
```

**面试话术**: "reader 线程卡在 `::read()` 阻塞调用里。如果只设 `running_ = false`，线程永远看不到——它还在内核态等着串口数据。必须先 `close(fd)` 让 `::read()` 返回错误，线程从阻塞中醒来、检查 `running_` 标志、退出循环。关闭顺序错了，`join()` 就永远等不到。"

---

## 第三层：能讲清 bug 和教训（面试加分项）

### Bug 1：时间戳域不一致导致融合结果错位

**现象**: 融合算法拿到的点云帧与实际相机帧时间偏差巨大，目标位置完全错位。

**原因**: 调用方用 `CLOCK_REALTIME` 获取相机时间戳，但雷达帧使用 `CLOCK_MONOTONIC`。两种时钟基准不同——REALTIME 从 1970-01-01 开始且受 NTP 调整，MONOTONIC 从系统启动开始。同一物理时刻在两个时钟域的差值可能达数十年（REALTIME 约 50 年 × 365 × 86400 × 10^9 ns）。

**解决**: 文档和 API 注释明确标注必须使用 `CLOCK_MONOTONIC`。融合线程统一切换到同一时钟源。

**面试话术**: "多传感器融合的核心是时间同步。用不同时钟域就像两个人在不同时区对表，找出来的'最近帧'毫无意义。全系统必须统一时间基准——我们选了 CLOCK_MONOTONIC，因为它不受系统时间调整影响，是传感器融合的标准选择。"

---

### Bug 2：缓冲区预分配不足导致点云截断

**现象**: 部分帧尾部点云丢失，融合结果在画面边缘异常。

**原因**: 调用方手动分配 `new LidarPoint[500]` 而非使用 `lidar.max_points_per_frame()`（返回 540）。`copy_slot` 为防止越界做了 `min(count, maxPointsPerSlot_)` 截断保护，无声丢失了尾端约 40 个点（约 7%）。这在视觉融合时表现为主画面边缘的障碍物"消失"。

**解决**: 调用方改用 `lidar.max_points_per_frame()` 动态获取缓冲区大小；文档增加警告说明。

**面试话术**: "库函数为防止越界做了截断保护，不会 crash，但会悄无声息地丢数据——这种 fail-safe 反而更难发现。API 设计上，`max_points_per_frame()` 已经提供了正确的缓冲区大小，调用者不应该硬编码。教训是：安全保护要配警告日志，不要让错误静默发生。"

---

### Bug 3：首圈数据不完整导致 start() 后前几秒无帧

**现象**: `start()` 返回后立即调用 `get_closest_frame()` 持续返回 false，约 2-3 秒后才有正常数据。上层应用在启动阶段读不到点云。

**原因**: 雷达已在上电运行，串口打开瞬间可能处于一帧的中间位置（不是圈边界）。reader 线程通过 `isFirstSweep` 标志主动跳过第一个不完整的半圈。同时雷达电机需约 2 秒加速到 600 RPM，期间角度数据可能不连续。

**解决**: 这是正常的设计行为。文档明确标注 `start()` 返回后需等待 2-3 秒。首圈丢弃保证了环形缓冲区中所有帧都是完整的 360° 扫描。这是一个"宁可少一帧，不传错一帧"的设计选择。

**面试话术**: "这不是 bug，是传感器物理特性的正常处理。雷达电机加速需要时间，串口打开瞬间也未必对齐包边界。我们的首圈丢弃策略确保交给上层的一定是完整帧。对面官可能追问'为什么不缓存首圈'——因为半圈数据角度不连续，强行拼接会引入伪点，对后续融合的破坏比少一帧大得多。"

---

## 怎么对着代码学

**别死记硬背。跟一遍数据流：**

1. 打开 `src/sentinel_lslidarer.cpp`，从 `start()` 开始
2. 跟一遍初始化流程：`SerialPort::open()` → `build_lut_()` → `new RingBuffer` → `readerThread_`
3. 进 `reader_loop_()`，理解主循环的 8 个步骤（见代码中 `// ---- 1-8 ----` 注释）
4. 打开 `src/serial_port.cpp`，理解 `read_packet()` 的包头同步机制
5. 打开 `src/ring_buffer.cpp`，理解 `commit_write()` 的 release 屏障和 `copy_slot()` 的 acquire 屏障
6. 打开 `src/m10p_protocol.cpp`，理解 `decode_packet_()` 的方位角插值和双回波解码
7. 跳到 `get_closest_frame()`，理解 O(n) 线性扫描最近帧

**重点函数入口:**

| 函数 | 作用 | 所在文件 |
|------|------|----------|
| `start()` / `stop()` | 理解完整生命周期和逆序关闭 | `sentinel_lslidarer.cpp` |
| `reader_loop_()` | 理解 8 步主循环：读包→校验→解码→圈边界→过滤→LUT→累积→提交 | `sentinel_lslidarer.cpp` |
| `get_closest_frame()` | 理解 O(validCount) 线性扫描 + delta 比较 | `sentinel_lslidarer.cpp` |
| `SerialPort::open()` | 理解 POSIX 串口原始模式配置（cfmakeraw + VTIME + VMIN） | `serial_port.cpp` |
| `SerialPort::read_packet()` | 理解阻塞 read + 内存扫描 0xA5 0x5A 包头同步 | `serial_port.cpp` |
| `RingBuffer::begin_write()` / `commit_write()` | 理解 SWCR 写入端：直接写 pointsPool + release 屏障 | `ring_buffer.cpp` |
| `RingBuffer::copy_slot()` | 理解 SWCR 读取端：acquire 屏障 + memcpy 拷贝 | `ring_buffer.cpp` |
| `decode_packet_()` | 理解 N10Plus 协议：方位角插值 + 16 组×2 回波解码 | `m10p_protocol.cpp` |
| `build_lut_()` | 理解 36000 项 sin/cos LUT 预计算 | `m10p_protocol.cpp` |
| `check_packet_validity_()` | 理解 CRC 字节累加和校验 | `m10p_protocol.cpp` |
