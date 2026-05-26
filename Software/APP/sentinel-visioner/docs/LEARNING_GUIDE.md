# SentinelVisioner — 学习指南

## 目标

面试时能说清：做了什么、为什么这么设计、踩过什么坑。

---

## 第一层：能说清"做了什么"（面试讲项目用）

### 一句话概括

> 基于 V4L2 + RGA 硬件加速的 RK3588 多路视觉流水线框架，一路 1080P NV12 输入经 DMA-BUF 零拷贝裂变为三路独立数据流（NPU 推理小图、1080P 预览、原始推流副本），全程不经过 CPU 内存搬运。

### 架构图（能画出来）

```
                         /dev/video11 (ISP 输出, 1080P NV12)
                                    │
                                    ▼
                         epoll 捕获线程 (capture_thread_func_)
                         VIDIOC_DQBUF → 拿到 dmaFd
                                    │
                  ┌─────────────────┼─────────────────┐
                  │ RGA 硬件         │ RGA 硬件          │ RGA 硬件
                  │ NV12→RGB888     │ NV12→RGB888       │ NV12→NV12
                  │ 1080P→640x640    │ 1080P→1080P       │ 同格式拷贝
                  │ + Letterbox     │ 无缩放            │ (imcopy)
                  ▼                 ▼                  ▼
         npuRgbPool         previewPool        origCopyPool
         (640x640 RGB888)   (1080P RGB888)     (1080P NV12)
                  │                 │                  │
                  ▼                 ▼                  ▼
         previewTaskQueue   previewTaskQueue   processTaskQueue
         (ThreadSafeQueue)  (ThreadSafeQueue)  (ThreadSafeQueue)
                  │                 │                  │
                  └────────┬────────┘                  │
                           ▼                           ▼
              wait_get_preview()                wait_get_orig_copy_buffer()
              try_get_preview()                  (sentinel-streamer 拉取)
              (NPU 推理 + QT 预览)              release_orig_copy_buffer()
                           │
                    release_preview()
```

### 关键代码（背下来）

```cpp
// 4 步用法，面试能张口就来
SentinelVisioner visioner;
visioner.add_camera("/dev/video11", 1920, 1080, 8, 0);  // 注册 + 建池
visioner.camera_stream_ctrl(0, true);                     // 开流 + 启线程

// 消费者拉取（两条独立的拉取路径）
NpuPreview task = visioner.wait_get_preview(0);           // NPU 推理 + 预览
DmaBuffer_t* raw = visioner.wait_get_orig_copy_buffer(0); // 推流/录像

// 用完必须归还
visioner.release_preview(0, &task);
visioner.release_orig_copy_buffer(0, raw);

// 关闭
visioner.camera_pause(0, true);   // 暂停（不关流）
visioner.camera_stream_ctrl(0, false);  // 完全停流
```

---

## 第二层：能解释"为什么这么设计"（面试追问用）

### 决策 1：为什么用 DMA-BUF + RGA 零拷贝而不是 CPU memcpy？

| 对比维度 | CPU memcpy（替代方案） | DMA-BUF + RGA 零拷贝（我们的方案） |
|----------|----------------------|----------------------------------|
| 数据路径 | V4L2 buffer → mmap vitrAddr → CPU memcpy → 用户 buffer | V4L2 dmaFd → RGA importbuffer_fd → 硬件搬移 → 输出 dmaFd |
| CPU 占用 | 1080P NV12 一帧约 3MB，3路=9MB 每帧，30fps=270MB/s 纯 memcpy | CPU 不参与像素搬运，捕获线程 CPU 峰值仅 8.7% |
| 内存带宽 | 占满 DDR 带宽，挤占 NPU/ISP 的总线资源 | 走内部 DMA 通道，不占用 CPU 侧 DDR 带宽 |
| 延迟 | memcpy 本身耗时 3-5ms | RGA 硬件处理 3 次共 64ms 端到端（含 V4L2 曝光） |
| 格式转换 | CPU 还得额外做 NV12→RGB888 转换（再来几十ms） | RGA 在搬移过程中顺便完成格式转换，零额外开销 |

**核心原理**：V4L2 通过 `VIDIOC_EXPBUF` 导出内核态 DMA-BUF 的文件描述符（dmaFd），RGA 通过 `importbuffer_fd()` 直接引用这块物理内存。整个链路只传一个 int 型的 dmaFd，数据始终在原位不动。

**教训**: 嵌入式中"搬运数据"是最昂贵的操作。能用硬件完成的，绝不让 CPU 碰。一个 dmaFd 4 字节，代替了 3MB 的 memcpy。

---

### 决策 2：为什么三个独立 DmaBufferPool 而不是一个大池？

| 对比维度 | 一个大池，统一分配（替代方案） | 三个独立池（我们的方案） |
|----------|---------------------------|------------------------|
| 内存规格 | 需要统一为最大规格（1080P RGB888 = 6MB），640×640 的图也占 6MB | 每个池精确匹配：npuRgbPool(640×640 RGB=1.2MB)、previewPool(1080P RGB=6MB)、origCopyPool(1080P NV12=3MB) |
| 浪费量 | 8 buffer × 3 路 × 6MB = 144MB，但 NPU 只需要 1.2MB 的图 | 总计约 82MB，节省 43% DMA 内存 |
| 竞争问题 | 三路消费者抢同一个池的 buffer，NPU 慢会导致推流也缺 buffer | 三路独立，NPU 推理卡顿不影响推流/预览 |
| 归还差错 | 容易还错（比如把 NPU 小图还到 origCopyPool），难排查 | 每路有专属 release 接口，编译期+运行期双重保护 |
| 生命周期 | 一个池坏了全部崩 | 单个池出问题，其他两路不受影响 |

**教训**: 不同用途的内存规格不同，强行统一就是浪费。独立池还带来了天然的模块隔离——NPU 推理慢了只丢 NPU 小图，推流继续稳如泰山。

---

### 决策 3：为什么用 epoll 而不是阻塞 read？

| 对比维度 | 阻塞 `read()`（替代方案） | `epoll` + `VIDIOC_DQBUF`（我们的方案） |
|----------|------------------------|--------------------------------------|
| 多路复用 | 一路摄像头一个线程阻塞 read，不能同时等多个 fd | 一个 epoll fd 监听多个摄像头 fd，单线程搞定 |
| 超时控制 | 阻塞 read 没有超时，线程退出必须靠信号打断（EINTR） | `epoll_wait(..., 1000)` 1秒超时，循环检查 `isThreadRunning` 标志，优雅退出 |
| 资源开销 | N 路摄像头 = N 个阻塞线程 | N 路摄像头 = 1 个 epoll fd + 1 个线程（本实现一路一个线程，但框架支持扩展） |
| Linux 生态 | read() 是通用 I/O，但 V4L2 不直接用 read 取帧 | V4L2 标准做法就是 `VIDIOC_DQBUF` + `VIDIOC_QBUF`，epoll 是社区推荐组合 |
| 帧完整性 | 直接 read 可能读到半帧 | DQBUF 保证返回完整帧，配合 epoll 实现帧就绪通知 |

**代码要点**：

```cpp
// epoll 三步：create → add → wait
ctx->epollFd = epoll_create1(0);
epoll_ctl(ctx->epollFd, EPOLL_CTL_ADD, ctx->camFd, &ev);
epoll_wait(ctx->epollFd, events, MAX_EVENTS, 1000);  // 1000ms 超时是关键
```

**教训**: 嵌入式多路视频采集，epoll 是标准答案。1000ms 超时不是拍脑袋——太短浪费 CPU，太长退出延迟不可接受。

---

### 决策 4：为什么 V4L2 MPLANE 模式要设 planes 数组？

| 对比维度 | 不设 planes（替代方案 / 常见错误） | 设 planes 数组（正确做法） |
|----------|----------------------------------|--------------------------|
| 行为 | `VIDIOC_QBUF` / `VIDIOC_DQBUF` 返回 `EINVAL`，静默失败或段错误 | 正常出队/入队，帧数据正确 |
| 原因 | MPLANE 模式下内核期望 `buf.m.planes` 指向有效的 `v4l2_plane` 数组，若不设则内核访问非法内存 | 单平面时 `length=1`，数组只一个元素，内核写回 `bytesused` 等字段 |
| 调试难度 | V4L2 API 不返回明确错误信息，只返回 -1，`strerror` 显示 "Invalid argument" | — |

**代码**：

```cpp
// 两处都要写：VIDIOC_QBUF 和 VIDIOC_DQBUF
struct v4l2_buffer buf = {};
buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
buf.memory = V4L2_MEMORY_MMAP;
buf.index = i;

struct v4l2_plane planes[1] = {};   // ← 关键：必须初始化
buf.m.planes = planes;              // ← 指向有效内存
buf.length = 1;                     // ← 单平面

ioctl(ctx->camFd, VIDIOC_QBUF, &buf);
```

**教训**: 内核 API 的最小约束往往藏在注释或驱动源码里。MPLANE 和单平面 (SINGLE_PLANE) 的 `v4l2_buffer` 布局不一样，RK3588 ISP 驱动只支持 MPLANE。不设 planes = 内核读到栈上的随机地址 = 未定义行为。`VIDIOC_REQBUFS` 用什么 type，后面所有 buf 操作就用什么 type，必须一致。

---

### 决策 5：为什么 camera_pause 而不是 STREAMOFF？

| 对比维度 | STREAMOFF 来回（替代方案） | camera_pause（我们的方案） |
|----------|--------------------------|---------------------------|
| 硬件状态 | `VIDIOC_STREAMOFF` → MIPI 时钟停 → ISP 管线销毁 → 再 `VIDIOC_STREAMON` → 需要完整重初始化 | 硬件流保持，只跳过 RGA 处理和队列推送 |
| 恢复时间 | 数百毫秒到数秒不等，RK3588 ISP 驱动可能直接恢复失败 | 瞬间恢复，下一帧立即可用 |
| 应用场景 | 真正要关闭摄像头时 | 临时停止预览（如切到视频管理页）、系统暂停态 |
| 风险 | STREAMOFF 后 STREAMON 不一定能回来，这是 RK3588 ISP 驱动的已知问题 | 零风险，硬件状态完全不受影响 |

**实现原理**：

```cpp
// camera_pause 做的事很简单：设一个原子标志
ctx->isPaused.store(true);

// 捕获线程检查到这个标志后：
if (ctx->isPaused.load()) {
    ioctl(ctx->camFd, VIDIOC_QBUF, &buf);  // 只归还，不做 RGA
    continue;                                // 跳过处理逻辑
}
```

V4L2 硬件流照常运行（传感器曝光 → ISP 输出 → V4L2 缓冲区就绪），捕获线程照样 DQBUF/QBUF 循环保持 buffer 流转，只是不调用 RGA 处理、不向队列 push 数据。下游消费者收不到新帧，自然"暂停"。

**教训**: 嵌入式硬件驱动的状态机不一定健壮。能保持就保持，别轻易走"关闭-重建"路径。`camera_pause` 本质上是"软件层跳过，硬件层继续"的设计模式。

---

### 决策 6：为什么需要 try_get_preview 超时版？

| 对比维度 | wait_get_preview 无限阻塞（最初方案） | try_get_preview 超时版（现在推荐） |
|----------|-----------------------------------|---------------------------------|
| 内部实现 | `pop()` → `cond_.wait(lock, predicate)` 无超时 | `try_pop(val, timeoutMs)` → `cond_.wait_for(lock, timeoutMs, predicate)` |
| 线程退出 | 相机停产后队列为空，`pop()` 永久阻塞，线程无法退出 → 死锁 | 每次超时检查退出标志，最多 timeoutMs 延迟后响应退出 |
| 适用消费者 | 一直运行的简单消费者 | 需要周期性检查退出条件的长生命周期线程（如 QT PreviewWorker） |
| CPU 占用 | 阻塞时挂起，0.0% CPU | 超时唤醒后检查标志再 sleep，同样是 0.0% CPU（条件变量语义） |
| 额外好处 | 无 | 消费者可以在无帧时做其他事（如检查连接状态、更新心跳） |

**使用对比**：

```cpp
// 危险：相机停产后线程永远卡死在这里
NpuPreview task = visioner->wait_get_preview(0);  // pop() 无限阻塞

// 安全：200ms 内一定响应退出信号
NpuPreview task = visioner->try_get_preview(0, 200);
if (task.npuImage == nullptr && !running_) break;  // 超时或退出
```

**教训**: `std::condition_variable::wait()` 无超时版本是线程退出的大敌。任何生产级代码中的阻塞调用都应有超时机制。这不是 V4L2 的问题，是多线程编程的通用原则——"永远给线程一条出路"。

---

### 决策 7：为什么用 CameraType 枚举而不是自动探测相机类型？

| 对比维度 | 自动探测（替代方案） | 显式指定 CameraType（我们的方案） |
|----------|---------------------|---------------------------------|
| 实现方式 | `VIDIOC_QUERYCAP` 获取 driver name，匹配 "uvcvideo" vs "rkisp" | 调用者传入 `CameraType::ISP_CAM` 或 `CameraType::USB_CAM` |
| 依赖 | 依赖驱动的 name 字段稳定不变，第三方 USB 驱动可能不以 "uvcvideo" 命名 | 无外部依赖，逻辑完全在应用层 |
| 确定性 | 驱动名匹配失败会导致误判，USB 相机被当作 ISP 初始化 → 必崩 | 调用者自己知道插的是什么，100% 确定 |
| 边缘场景 | 同是 USB 但走不同协议（UVC vs gspca），驱动名不同；未来 MIPI→USB bridge 的 driver name 不可预测 | 所有边缘场景由调用者处理，库内逻辑简单 |
| 代码复杂度 | 需要维护驱动名匹配表，每次新硬件都要更新 | 一个 enum + 两路分支，代码量极少 |

**核心逻辑**：

```cpp
enum class CameraType { ISP_CAM, USB_CAM };

// 调用者显式指定，库内据此分流
ctx->v4l2BufType = (camType == CameraType::ISP_CAM)
    ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
    : V4L2_BUF_TYPE_VIDEO_CAPTURE;
```

**面试话术**: "自动探测看起来很智能，但在嵌入式异构相机场景下是个陷阱。不同 USB 芯片的驱动名不一样，甚至同一个摄像头在不同内核版本下 driver name 都可能变化。我们用显式指定——调用者插的什么相机自己清楚，库内只负责按类型走不同 V4L2 初始化路径。这样做代码量极少，没有任何外部依赖，边缘场景不会出现归类错误导致的崩溃。"

### 决策 8：为什么 USB 先尝试 NV12 而不是直接上 YUYV？

| 对比维度 | 统一 YUYV 路径（替代方案） | NV12 优先 + YUYV 回退（我们的方案） |
|----------|-------------------------|---------------------------------|
| 最优路径 | 每帧多一次 RGA YUYV→NV12 转换，无论摄像头是否支持 NV12 | 摄像头原生支持 NV12 时零额外开销 |
| 代码复杂度 | 少一个分支 | 多一次 `VIDIOC_S_FMT` 重试 + 条件化 convert pool 分配 |
| 实际效果 | 市面上越来越多 USB 摄像头原生支持 NV12（UVC 1.5+） | 自适应：NV12 时路径等同 ISP，YUYV 时自动加一层 RGA |
| RGA 负载 | 多一次 RGA 调用（每帧约 1-3ms） | NV12 时 RGA 负载不变，YUYV 时多一次 |

**面试话术**: "USB UVC 规范 1.5 之后越来越多的摄像头支持 NV12 原生输出，这是趋势。如果一刀切走 YUYV，等于给所有相机都加了一层不必要的 RGA 转换——这在多路相机场景下会累加 RGA 负载。我们的方案是先尝试 NV12，失败了再回退 YUYV。对用户来说完全透明，但性能上 NV12 时零额外开销。"

---

## 第三层：能讲清 bug 和教训（面试加分项）

### Bug 1：ThreadSafeQueue 缺少 `<condition_variable>` 头文件

**现象**: 编译报错 `'condition_variable' is not a member of 'std'`，或直接 `error: 'condition_variable' does not name a type`。

**原因**: 最初的 `ThreadSafeQueue.h` 用了 `std::condition_variable` 和 `std::mutex`，只 `#include <mutex>` 和 `#include <queue>`，漏了 `#include <condition_variable>`。在部分编译器/标准库版本上，`<mutex>` 可能间接包含了 `<condition_variable>` 的声明，侥幸编译通过；换到交叉编译的 aarch64 工具链后，间接包含路径不同，立刻报错。

**解决**: 在 `ThreadSafeQueue.h` 头部显式添加 `#include <condition_variable>`。

```cpp
// 修复后的完整 include（缺一不可）
#include <chrono>
#include <condition_variable>   // ← 多出来的这一行
#include <mutex>
#include <queue>
```

**面试话术**: "C++ 标准库的头文件依赖没有传递性保证。`<mutex>` 今天包含了 `<condition_variable>` 不代表明天也包含。交叉编译器切换后编译失败是最典型的表现。后来我们养成习惯，头文件里用了什么类型就显式 include 什么，不依赖间接包含。"

---

### Bug 2：STREAMOFF 后 RK3588 ISP 管线无法恢复

**现象**: 用户点击"暂停系统"后再"恢复"，预览画面黑屏，不再推流。日志无任何报错，只是无新帧到达。必须重启整个程序才能恢复。

**原因**: 早期的"暂停"逻辑直接调用 `camera_stream_ctrl(camNum, false)` 执行 `VIDIOC_STREAMOFF`。RK3588 ISP 驱动在 STREAMOFF 后，MIPI CSI 时钟停止，ISP 子模块（3A 统计、去马赛克、色彩校正）全部进入 reset 状态。重新 `VIDIOC_STREAMON` 时，驱动期望上层重新协商格式（`VIDIOC_S_FMT`）、重新分配 buffer（`VIDIOC_REQBUFS`），而不只是简单开流。仅做 STREAMON 无法让 ISP 恢复正常工作。

更细的原因：ISP 驱动内部有一个"首次开流"标志，第一次 STREAMON 后会清除。STREAMOFF 不会把这个标志设回去，第二次 STREAMON 时驱动以为已经是"运行中"状态，跳过了 MIPI 时钟配置和 CSI 接收器初始化。

**解决**: 引入 `camera_pause` 机制，通过 `std::atomic<bool> isPaused` 标志在软件层跳过 RGA 处理，硬件流全程保持。`VIDIOC_STREAMOFF` 只在真正的关闭流程中调用（程序退出时），并且再次启动时走完整的 `add_camera` 流程。

**面试话术**: "嵌入式硬件驱动的状态机往往有隐含假设。RK3588 ISP 驱动假设 STREAMON 只在初始化后调用一次，重复 STREAMON 不会重新初始化 MIPI 管线。我们的解决思路是不跟驱动较劲——既然它不支持热重启，我们就用软件开关（camera_pause）模拟暂停，让硬件始终保持运行。这是嵌入式开发中和硬件打交道的典型 mindset：硬件能支持的才做，不支持的就用软件绕过去。"

---

### Bug 3：`pop()` 无限阻塞导致消费者线程死锁

**现象**: 用户点击"关闭"按钮后，程序界面卡死，不响应任何操作。检查进程状态发现 `wait_get_preview()` 所在线程卡在 `futex()` 系统调用上永久休眠。必须 `kill -9` 强杀进程。

**原因**: 关闭流程的执行顺序是：

```
主线程                               消费者线程
  │                                     │
  ├─ camera_stream_ctrl(false)          ├─ wait_get_preview(0)
  │   ├─ isThreadRunning = false   ──→  │   └─ previewTaskQueue.pop()
  │   ├─ captureThread.join() ←── ─ ─ ─│      └─ cond_.wait(lock, ...)
  │   │   (等待线程退出...永远等不到)    │         ↑ 队列已空，capture 线程已停
  │   └─ VIDIOC_STREAMOFF               │         永久阻塞，永远检查不到 isThreadRunning
```

问题在于：主线程先停了捕获线程（帧来源断了），但消费者线程还在 `pop()` 里等帧。`pop()` 使用的 `cond_.wait()` 没有超时，必须有人 push 才能唤醒。但捕获线程已死，再无 push，消费者线程永远卡在 `wait()` 里，`running_` 标志永远检查不到，`join()` 永远等不到。

**解决**: 分两步：
1. 在 `ThreadSafeQueue` 中新增 `try_pop(T& val, int timeoutMs)` 超时版本
2. 消费者改用 `try_get_preview(camNum, 200)` 轮询，轮询间隙检查退出标志

```cpp
// 修复前（死锁）
NpuPreview task = visioner->wait_get_preview(0);  // pop() 无超时

// 修复后（安全退出）
while (running_) {
    NpuPreview task = visioner->try_get_preview(0, 200);
    if (task.npuImage != nullptr) {
        process(task);
        visioner->release_preview(0, &task);
    }
    // 超时后自然回到 while(running_) 检查，最多 200ms 延迟
}
```

**面试话术**: "这是 C++ 多线程里最经典的死锁模式——生产者停了，消费者永远等不到数据。`std::condition_variable::wait` 无罪，有罪的是不给它设超时。我们在 SentinelVisioner 里加了 `try_get_preview` 超时版接口，PreivewWorker 改用 200ms 超时轮询，确保退出信号最多 200ms 内被响应。记住一条铁律——凡是用条件变量的阻塞等，要么设超时，要么在析构前手动 notify。"

---

## 怎么对着代码学

**别死记硬背。跟一遍数据流：**

1. 打开 `src/sentinel-visioner.cpp`，从 `add_camera` 开始
2. 跟一遍 V4L2 初始化：`open` → `VIDIOC_S_FMT` → `VIDIOC_S_PARM` → `VIDIOC_REQBUFS` → `VIDIOC_EXPBUF` → `VIDIOC_QBUF` → `epoll_create1`
3. 进 `capture_thread_func_`，理解 epoll 循环：`epoll_wait` → `VIDIOC_DQBUF` → 三次 RGA → `VIDIOC_QBUF`
4. 看 `rga_process_to_rgb_`，理解一次 RGA 调用的完整生命周期：`importbuffer_fd` → `wrapbuffer_handle` → `imfill`（灰边）→ `improcess`（缩放+格式转换）→ `releasebuffer_handle`
5. 跳到消费者侧：`wait_get_preview` / `try_get_preview` / `wait_get_orig_copy_buffer` → 理解两个队列的分发逻辑
6. 最后看 `release_preview` 和 `release_orig_copy_buffer` → 理解 DmaBufferPool 的归还机制
7. 回到 `~SentinelVisioner` → 理解逆序析构

**重点函数入口:**

| 函数 | 作用 | 所在文件 |
|------|------|----------|
| `add_camera` | 理解 V4L2 完整初始化流程（8 个 ioctl 步骤） | `src/sentinel-visioner.cpp` |
| `capture_thread_func_` | 理解 epoll 监听 + DQBUF/QBUF + 三次 RGA 调度的主循环 | `src/sentinel-visioner.cpp` |
| `rga_process_to_rgb_` | 理解一次 RGA 缩放+格式转换+Letterbox+EIS 防抖偏移 | `src/sentinel-visioner.cpp` |
| `rga_convert_to_rgb_full_` | 理解 NV12→RGB888 纯格式转换（无缩放） | `src/sentinel-visioner.cpp` |
| `rga_copy_buffer_` | 理解 RGA imcopy 同格式零拷贝（给推流用的原始副本） | `src/sentinel-visioner.cpp` |
| `camera_pause` | 理解原子标志暂停如何绕过 RGA 而不关闭 V4L2 流 | `src/sentinel-visioner.cpp` |
| `camera_stream_ctrl` | 理解 STREAMON/STREAMOFF + 线程启停的完整生命周期 | `src/sentinel-visioner.cpp` |
| `wait_get_preview` / `try_get_preview` | 理解阻塞和超时两种消费模式的差异和使用场景 | `src/sentinel-visioner.cpp` |
| `release_preview` / `release_orig_copy_buffer` | 理解 DMA 缓冲区归还机制和防泄漏设计 | `src/sentinel-visioner.cpp` |
| `ThreadSafeQueue::pop` / `try_pop` | 理解条件变量阻塞队列的内部实现 | `include/ThreadSafeQueue.h` |
