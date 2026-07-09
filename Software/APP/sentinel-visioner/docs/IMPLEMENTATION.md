# SentinelVisioner — 技术实现文档

## 1. 概述

SentinelVisioner 是 RK3588-Omni-Sentinel 平台的多路视觉流水线组件，实现"一分三"零拷贝扇出架构：每路 V4L2 输入经 RGA 硬件裂变为三路独立数据流——NPU 推理小图、RGB888 预览图像、NV12 原始推流副本。三路数据流各自使用独立的 DMA 缓冲池，通过阻塞队列交付下游消费者，全程零 CPU 像素拷贝。

支持两路摄像头同时接入：CAM0 为 MIPI CSI（RK3588 ISP，1920x1080，MPLANE + NV12），CAM1 为 USB UVC（1280x720，Single-Planar，NV12/YUYV 自动协商）。两路相机通过 `camNum` 索引拥有完全隔离的 `CameraContext`、DMA 内存池和捕获线程。

**已知问题（2026-05-27）**: USB 相机画面存在横向花屏，根因未确定。当前使用 `usbSafePool` 缓解（详见 #7 bug 记录和 2.5 节）。

---

## 2. 架构总览

```
                    V4L2 摄像头驱动 (/dev/video11)
                         │
                         │ epoll EPOLLIN
                         ▼
              ┌──────────────────────┐
              │  capture_thread_func_ │  (每路相机 1 个线程)
              │                      │
              │  VIDIOC_DQBUF        │
              │    │                 │
              │    ├── currentDmaFd  │
              │    │                 │
              │    ├── RGA 操作 A ───▶ npuRgbPool     (640×640  RGB888)
              │    │   (缩放+Letterbox+EIS)  │
              │    │                    previewTaskQueue
              │    │                         │
              │    ├── RGA 操作 B ───▶ previewPool    (1920×1080 RGB888)
              │    │   (NV12→RGB888)        │
              │    │                    previewTaskQueue
              │    │                         │
              │    ├── RGA 操作 C ───▶ origCopyPool   (1920×1080 NV12)
              │    │   (NV12 同格式拷贝)       │
              │    │                    processTaskQueue
              │    │                         │
              │    └── VIDIOC_QBUF            │
              │    归还 buffer 给驱动          │
              └──────────────────────┘        │
                                              │
            ┌─────────────────────────────────┘
            │
            ▼
   ┌────────────────────┐       ┌──────────────────────┐
   │  NPU/预览消费者      │       │  推流/录像消费者        │
   │  (SentinelQT 等)    │       │  (SentinelStreamer)    │
   │                    │       │                        │
   │  wait_get_preview  │       │  wait_get_orig_copy    │
   │  try_get_preview   │       │  _buffer               │
   │  release_preview   │       │  release_orig_copy     │
   └────────────────────┘       └──────────────────────────┘
```

**设计原则**:
- 三路 RGA 操作共享同一个 `currentDmaFd` 源，RGA 硬件直接读取 V4L2 DMA-BUF，无 CPU 介入
- 每路数据流有独立的 `DmaBufferPool`，格式和分辨率按用途定制
- 捕获线程与消费者间通过 `ThreadSafeQueue` 解耦，条件变量休眠，空闲 CPU 0%
- 暂停机制跳过 RGA 处理但不停止 V4L2 硬件流，避免 RK3588 ISP 管线重建问题

### 2.5 USB 相机安全拷贝机制

USB 相机的 NV12 DMA-BUF 经 RGA 直接读取时存在横向花屏问题（根因未确定，详见 `BUG_RECORD.md` #7）。作为缓解措施，为 USB NV12 相机引入 `usbSafePool`（4 个 DMA buffer，NV12 格式）。

USB NV12 相机的捕获线程流程与 ISP 不同：

```
DQBUF → sync_dma_buf_for_device → imcopy(相机BUF → usbSafePool)
→ QBUF(立即归还相机, 约1-2ms内完成)
→ improcess(usbSafePool → NPU, IM_SYNC)
→ improcess(usbSafePool → 预览, IM_SYNC)
→ imcopy(usbSafePool → origCopyPool)
→ 释放 usbSafePool
```

关键设计：
- 相机 DMA-BUF 仅被访问一次（`imcopy`），后续三次 RGA 操作均从 usbSafePool 读取
- `usbSafePool` 由 `dma_heap`（`DMA_HEAP_DMA32_UNCACHE_PATCH`）分配，与 ISP 相机使用的 DMA 池同类型
- `improcess` 全部使用 `IM_SYNC` 同步模式，确保 RGA 完成后再继续
- `DQBUF` 后调用 `DMA_BUF_IOCTL_SYNC`（`sync_dma_buf_for_device`）刷新缓存
- 读取 `VIDIOC_G_FMT` 的 `bytesperline` 字段，通过 `srcStride` 参数传递给 RGA 函数

此机制**未完全解决**花屏问题，相关代码标注了 `safeBuf`/`safeBufToRelease` 变量名，方便后续回退。

---

## 3. 模块划分

| 文件 | 职责 |
|------|------|
| `include/sentinel-visioner.h` | 公共 API 头文件（类声明、CameraContext 结构体） |

---

## 4. 核心数据结构

### 4.1 DmaBuffer_t（定义于 `dma-buffer-pool.h`）

DMA 内存块节点，承载图像数据的元信息：

```cpp
struct DmaBuffer_t {
    int dmaFd;               // DMA 文件描述符，跨硬件模块传递的零拷贝句柄
    void* virtAddr;          // CPU 映射虚拟地址，预览/UI 可直接读取
    std::atomic<bool> ifUse; // 无锁占用标记
    int bufferSize;          // 内存块实际字节大小
    int width;               // 图像宽度
    int height;              // 图像高度
    uint64_t timestampUs;
    uint64_t timestampUs;    // 时间戳 (CLOCK_MONOTONIC，来自 V4L2 buf.timestamp)
    int32_t eisOffsetX;      // EIS 防抖水平偏移（像素），供下游 OSD/streamer 使用
    int32_t eisOffsetY;      // EIS 防抖垂直偏移（像素）
    bool eisActive;          // EIS 是否激活    // 时间戳 (CLOCK_MONOTONIC，来自 V4L2 buf.timestamp)
    DmaBuffer_t* next;       // 空闲链表 next 指针
};
```

### 4.2 CameraContext

每路摄像头完整状态上下文，包含所有硬件资源和缓冲池：

```cpp
struct CameraContext {
    int camNum;                               // 摄像头逻辑编号
    int camFd;                                // V4L2 设备文件描述符
    int epollFd;                              // epoll 实例文件描述符
    std::string deviceName;                   // 设备节点路径 (如 "/dev/video11")
    int width;                                // 采集宽度 (如 1920)
    int height;                               // 采集高度 (如 1080)
    int bufferCount;                          // V4L2 缓冲块数量 + 各池缓冲块数量
    bool isStreaming;                         // 硬件流是否已开启
    std::vector<DmaBufferInfo> buffers;       // V4L2 export 的 DMA-BUF 列表

    std::unique_ptr<std::thread> captureThread;   // 捕获线程对象
    std::atomic<bool> isThreadRunning;            // 线程退出标志
    std::atomic<bool> isPaused;                   // 暂停标志（跳过 RGA，仅 QBUF）

    std::unique_ptr<DmaBufferPool> npuRgbPool;      // NPU 推理小图池 (640×640 RGB888)
    std::unique_ptr<DmaBufferPool> origCopyPool;    // 原始推流拷贝池 (NV12)
    std::unique_ptr<DmaBufferPool> previewPool;     // 预览图像池 (1080p BGR888)
    std::unique_ptr<DmaBufferPool> usbConvertPool;  // USB YUYV→NV12 中间转换池 (NV12)
    std::unique_ptr<DmaBufferPool> usbSafePool;     // USB NV12 安全拷贝缓冲池 (NV12)
    std::unique_ptr<DmaBufferPool> mjpegDecodePool; // USB MJPG→NV12 FFmpeg 解码输出池

    ThreadSafeQueue<DmaBuffer_t*> npuTaskQueue;      // NPU 推理任务队列 (640×640 RGB888)
    ThreadSafeQueue<DmaBuffer_t*> previewTaskQueue;   // 预览任务队列 (1080p BGR888)
    ThreadSafeQueue<DmaBuffer_t*> processTaskQueue;  // 推流/录像原图队列 (NV12)

    // 相机类型相关 (仅 USB 时部分字段有效)
    CameraType camType;               // ISP_CAM 或 USB_CAM
    int v4l2BufType;                  // V4L2 buffer type (MPLANE 或 SINGLE_PLANAR)
    unsigned int actualPixelFormat;   // 实际协商后的像素格式 (NV12/YUYV/MJPG)
    int srcBytesPerLine;              // USB 相机实际行跨度 (V4L2 bytesperline)

    // EIS EMA 低通滤波状态 (每路独立)
    float eisSmoothAlpha;
    int32_t prevEisOffsetX;
    int32_t prevEisOffsetY;
    bool eisPrevValid;
};
```

### 4.3 SentinelVisioner（公共 API）

NPU 和预览已拆分为独立队列，各自有独立的 `wait`/`try`/`release` 方法组：

```cpp
class SentinelVisioner {
public:
    SentinelVisioner();
    ~SentinelVisioner();

    // 初始化与生命周期
    bool add_camera(std::string& deviceName, int width, int height,
                    int bufferCount, int camNum,
                    CameraType camType = CameraType::ISP_CAM);
    bool camera_stream_ctrl(int camNum, bool isOpen);
    void camera_pause(int camNum, bool paused);

    // NPU 消费者 API (独立队列)
    DmaBuffer_t* wait_get_npu(int camNum);
    DmaBuffer_t* try_get_npu(int camNum, int timeoutMs);
    void release_npu(int camNum, DmaBuffer_t* buf);

    // 预览消费者 API (独立队列)
    DmaBuffer_t* wait_get_preview(int camNum);
    DmaBuffer_t* try_get_preview(int camNum, int timeoutMs);
    void release_preview(int camNum, DmaBuffer_t* buf);

    // 推流/录像消费者 API
    DmaBuffer_t* wait_get_orig_copy_buffer(int camNum);
    void release_orig_copy_buffer(int camNum, DmaBuffer_t* buf);

    // EIS 防抖配置
    void set_eis_offset_callback(std::function<bool(uint64_t, int, int32_t&, int32_t&)> cb);
    void set_eis_smooth_alpha(float alpha);

private:
    std::unordered_map<int, std::unique_ptr<CameraContext>> _cameraContextMap;
    std::function<bool(uint64_t, int, int32_t&, int32_t&)> eis_offset_callback_;

    void release_camera_resources_(CameraContext* context);
    void capture_thread_func_(int camNum);

    // RGA 硬件操作
    bool rga_process_to_rgb_(int srcFd, int srcWidth, int srcHeight, int srcStride,
                             DmaBuffer_t* dstBuf, int horizontalOffset, int verticalOffset);
    bool rga_convert_to_rgb_full_(int srcFd, int srcWidth, int srcHeight, int srcStride,
                                   DmaBuffer_t* dstBuf);
    bool rga_copy_buffer_(int srcFd, int width, int height, int srcStride, DmaBuffer_t* dstBuf);
    bool rga_yuyv_to_nv12_(int srcFd, int srcWidth, int srcHeight, int srcStride, DmaBuffer_t* dstBuf);

    // FFmpeg 软件解码
    bool mjpeg_decode_to_nv12_(const uint8_t* jpegData, size_t jpegSize, DmaBuffer_t* dstBuf);
};
```

---

## 5. 线程模型

### 5.1 线程清单

| 线程 | 职责 | 生命周期 | 同步机制 |
|------|------|----------|----------|
| capture_thread_func_ | 每路相机 1 个线程，epoll 监听 V4L2 帧到达，连续 3 次 RGA 调度，写入两个阻塞队列 | `camera_stream_ctrl(true)` 创建，`camera_stream_ctrl(false)` join 销毁 | `isThreadRunning` atomic |
| NPU 消费者线程 | 由调用方创建，调用 `wait_get_npu` / `try_get_npu` 阻塞拉取 | 调用方自行管理 | `npuTaskQueue` 内部 mutex + condvar |
| 预览消费者线程 | 由调用方创建（如 SentinelQT PreviewWorker），调用 `wait_get_preview` / `try_get_preview` 阻塞拉取 | 调用方自行管理 | `previewTaskQueue` 内部 mutex + condvar |
| 推流/录像消费者线程 | 由调用方创建（如 SentinelStreamer 内部线程），调用 `wait_get_orig_copy_buffer` 阻塞拉取 | 调用方自行管理 | `ThreadSafeQueue` 内部 mutex + condvar |

### 5.2 捕获线程生命周期

```
main thread                            capture_thread_func_
    │                                       │
    ├─ camera_stream_ctrl(true)             │
    │   ├─ VIDIOC_STREAMON                  │
    │   ├─ isThreadRunning = true           │
    │   └─ captureThread = new thread ──────▶ while(isThreadRunning)
    │                                           ├─ epoll_wait(1000ms timeout)
    │                                           │   超时 → 检查 isThreadRunning 标志
    │                                           │
    │                                           ├─ VIDIOC_DQBUF
    │                                           ├─ isPaused? → QBUF → continue
    │                                           ├─ RGA 操作 A: NPU 小图
    │                                           ├─ RGA 操作 B: 1080p 预览
    │                                           ├─ previewTaskQueue.push()
    │                                           ├─ RGA 操作 C: NV12 拷贝
    │                                           ├─ processTaskQueue.push()
    │                                           └─ VIDIOC_QBUF
    │
    ├─ camera_stream_ctrl(false)            │  (检测 false，退出循环)
    │   ├─ isThreadRunning = false ────────▶
    │   ├─ captureThread.join() ◀────────── 线程退出
    │   ├─ captureThread.reset()
    │   └─ VIDIOC_STREAMOFF
    │
```

**关键细节**:
- `epoll_wait` 超时设为 1000ms，避免 `isThreadRunning = false` 后线程永久卡在 `epoll_wait`
- `STREAMOFF` 必须在 `thread.join()` 之后执行，防止线程持有 epoll fd 时关闭硬件流导致 ioctl 错误
- 捕获线程中所有 RGA 操作失败不会崩溃，会归还已分配的内存块并继续 QBUF

### 5.3 线程安全规则

| 规则 | 说明 |
|------|------|
| `isThreadRunning` | `std::atomic<bool>`，main thread 写入 false，capture thread 读取 |
| `isPaused` | `std::atomic<bool>`，main thread 通过 `camera_pause()` 写入，capture thread 读取 |
| `DmaBufferPool` (多个) | **仅 capture thread 写**（get_buffer / release_buffer 在失败路径），消费者线程**不直接操作** pool |
| `npuTaskQueue` | capture thread push，NPU 消费者线程 pop（生产者-消费者，mutex + condvar 保护） |
| `previewTaskQueue` | capture thread push，预览消费者线程 pop（生产者-消费者，mutex + condvar 保护） |
| `processTaskQueue` | capture thread push，推流消费者线程 pop（生产者-消费者，mutex + condvar 保护） |
| `_cameraContextMap` | 仅 main thread 写入（add / remove），其他线程通过指针只读访问 |

---

## 6. 主要 DmaBufferPool 的用途与格式

| 池名称 | 分辨率 | 格式 | 用途 | 消费者 |
|--------|--------|------|------|--------|
| `npuRgbPool` | 640 × 640 | RGB888 | NPU 推理小图（缩放 + Letterbox + EIS 防抖偏移） | `wait_get_preview` → `npuImage` |
| `previewPool` | 1920 × 1080 | RGB888 | UI 预览图像（1:1 格式转换，无缩放无 Letterbox） | `wait_get_preview` → `previewImage` |
| `origCopyPool` | 1920 × 1080 | NV12 | 推流/录像原始拷贝（同格式 1:1，保持编码器输入质量） | `wait_get_orig_copy_buffer` |

三个池在 `add_camera` 中统一创建，缓冲区数量与 V4L2 bufferCount 一致（通常 8 块），在 `release_camera_resources_` 中统一销毁。

> **为什么推流副本用 NV12 同格式拷贝而不用 RGB888？** 编码器（MPP/FFmpeg）的输入格式通常是 NV12（YUV420SP），如果转成 RGB888 再编码会多一次无意义的色彩空间转换，增加 RGA 负载且降低画质。

---

## 7. ThreadSafeQueue 阻塞队列机制

### 7.1 实现

```cpp
template<typename T>
class ThreadSafeQueue {
    std::mutex mutex_;
    std::condition_variable cond_;
    std::queue<T> queue_;

    // 无限阻塞弹出
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return !queue_.empty(); });
        T val = queue_.front();
        queue_.pop();
        return val;
    }

    // 带超时阻塞弹出 (timeoutMs 毫秒)
    bool try_pop(T& val, int timeoutMs) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                            [this] { return !queue_.empty(); })) {
            return false;  // 超时返回，不阻塞永久
        }
        val = queue_.front();
        queue_.pop();
        return true;
    }

    // 压入并通知一个等待者
    void push(T val) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(val));
        }
        cond_.notify_one();
    }

    // 清空队列，只取最新元素（跳帧用，非阻塞）
    bool drain_latest(T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        while (queue_.size() > 1) { queue_.pop(); }
        val = queue_.front();
        queue_.pop();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
};
```

### 7.2 使用场景

| 方法 | 调用方 | 场景 |
|------|--------|------|
| `pop()` | `wait_get_npu` / `wait_get_preview` / `wait_get_orig_copy_buffer` | 消费者无需退出检查，可无限阻塞等待 |
| `try_pop(val)` | 非阻塞快速检查 | 消费者不希望阻塞，立即返回 |
| `try_pop(val, timeoutMs)` | `try_get_npu` / `try_get_preview` | 消费者需要周期性检查退出标志（如 QT 子线程），用 200ms 超时轮询 |
| `push(val)` | `capture_thread_func_` | 捕获线程 RGA 完成后投递任务 |
| `drain_latest(val)` | 下游消费者做跳帧处理 | 宁可丢弃旧帧也要最新一帧的场景 |
| `empty()` | 消费者/管理线程 | 轮询检查队列是否有数据就绪 |

### 7.3 易踩坑的细节

- **`wait_get_npu` / `wait_get_preview` 使用无限阻塞 `pop()`**：如果下游消费者线程在阻塞等待时捕获线程已停止，消费者线程将永久挂起。对于需要优雅退出的场景，应使用 `try_get_*` 超时版，在超时后检查退出标志。
- **`push` 后 `notify_one()` 的作用域**：`notify_one()` 在 `lock_guard` 作用域**之外**调用，避免"hurry up and wait"问题——waiting thread 被唤醒后立即因 mutex 仍被锁而再次休眠。
- **`DmaBuffer_t*` 的指针语义**：队列中存储的是 `DmaBuffer_t*` 指针，消费者拿到指针后必须调用对应的 `release_*` 接口归还到对应内存池。NPU 和预览是两个完全独立的队列，归还时各自调用 `release_npu` / `release_preview`。

---

## 8. 核心数据流

### 8.1 捕获线程 capture_thread_func_ 完整流程

```
capture_thread_func_(camNum)
  │
  ├─ 获取 CameraContext* ctx
  │
  └─ while (ctx->isThreadRunning)
       │
       ├─ epoll_wait(ctx->epollFd, ..., 1000ms)
       │    │
       │    ├─ 超时 (nfds==0) → continue (检查退出标志)
       │    ├─ 信号中断 (EINTR) → continue
       │    └─ 错误 → break
       │
       ├─ VIDIOC_DQBUF → currentDmaFd, timestampUs
       │    │
       │    └─ isPaused? ──── YES → VIDIOC_QBUF → continue
       │
       ├─ [USB YUYV→NV12] 若 actualPixelFormat == YUYV
       │    ├─ usbConvertPool->get_buffer() → convBuf
       │    ├─ rga_yuyv_to_nv12_(currentDmaFd, W, H, convBuf)
       │    └─ nv12DmaFd = convBuf->dmaFd (后续操作改用此 fd)
       │
       ├─ [操作 A] npuRgbPool->get_buffer() → targetNpuBuf
       │    ├─ rga_process_to_rgb_(currentDmaFd, 1920, 1080, targetNpuBuf, EIS_offset)
       │    │   (1080P NV12 → 640×640 RGB888, 缩放+Letterbox灰边+EIS平移)
       │    └─ 失败 → npuRgbPool->release_buffer(targetNpuBuf)
       │
       ├─ [操作 B] previewPool->get_buffer() → targetPreviewBuf
       │    ├─ rga_convert_to_rgb_full_(currentDmaFd, 1920, 1080, targetPreviewBuf)
       │    │   (1080P NV12 → 1080P RGB888, 1:1 纯格式转换)
       │    └─ 失败 → previewPool->release_buffer(targetPreviewBuf)
       │
       ├─ 操作 A 和 B 均成功?
       │    ├─ YES → previewTaskQueue.push({targetNpuBuf, targetPreviewBuf})
       │    └─ NO  → 分别释放已分配的两个 buffer (防止泄漏)
       │
       ├─ [操作 C] origCopyPool->get_buffer() → targetOrigBuf
       │    ├─ rga_copy_buffer_(currentDmaFd, 1920, 1080, targetOrigBuf)
       │    │   (1080P NV12 → 1080P NV12, 同格式硬件拷贝)
       │    ├─ 成功 → processTaskQueue.push(targetOrigBuf)
       │    └─ 失败 → origCopyPool->release_buffer(targetOrigBuf)
       │
       └─ VIDIOC_QBUF  (归还 V4L2 buffer)
```

**重要**: 操作 A、B、C 各自独立写入三个队列（`npuTaskQueue`、`previewTaskQueue`、`processTaskQueue`），互不阻塞。NPU 池空不影响预览产出，反之亦然。操作 C 的 buffer 还携带 EIS 偏移信息（`eisOffsetX`/`eisOffsetY`/`eisActive`），供推流/录像方的 OSD 叠加使用。这样下游消费者可以只订阅自己需要的数据流，互不干扰。

### 8.2 RGA 操作 A: rga_process_to_rgb_ (NPU 小图)

功能：1080P NV12 输入 → 640×640 RGB888 输出，带 Letterbox 灰边填充和 EIS 防抖平移偏移。

```
rga_process_to_rgb_(srcFd, srcWidth, srcHeight, srcStride, dstBuf, horizOffset, vertOffset)
  │
  ├─ importbuffer_fd(srcFd, 1920×1080, YCrCb_420_SP)  → rga_handle_src
  ├─ importbuffer_fd(dstFd, 640×640, RGB_888)          → rga_handle_dst
  │
  ├─ 计算 Letterbox 缩放参数:
  │    scale = min(640/1920, 640/1080)
  │    scaled_w = 1920 × scale
  │    scaled_h = 1080 × scale
  │    offset_x = (640 - scaled_w) / 2 + horizOffset
  │    offset_y = (640 - scaled_h) / 2 + vertOffset
  │
  ├─ imfill(rga_buf_dst, 0xFF727272)  // 灰色背景填充 (去脏数据)
  │
  ├─ improcess(rga_buf_src, rga_buf_dst, srect, drect)
  │    srect = {0, 0, 1920, 1080}        // 源全图
  │    drect = {offset_x, offset_y, scaled_w, scaled_h}  // 缩放后带偏移写入
  │
  └─ releasebuffer_handle(src/dst)
```

**EIS 防抖**: `horizOffset` / `vertOffset` 叠加在 Letterbox 居中偏移之上。EIS 偏移通过 `set_eis_offset_callback()` 注入回调，采集线程每帧调用回调获取像素补偿量。回调独立于 NPU buffer 执行，确保无 NPU 消费端时仍能计算偏移。当 camera 发生抖动时，外部 IMU 计算补偿量传入，`drect` 中心随之移动，超出画布边缘由 `imfill` 的灰底承接，不会出现黑边或脏数据。

**回调接口** (`set_eis_offset_callback`):
```cpp
void set_eis_offset_callback(
    std::function<bool(uint64_t timestampUs, int camNum,
                       int32_t& offsetX, int32_t& offsetY)> callback);
```
传入 V4L2 帧时间戳（微秒），输出像素偏移量。传 nullptr 可禁用 EIS。

**Letterbox 灰边**: 使用 `0xFF727272` 填充（对应 RGB 的 114,114,114），与常见深度学习预处理灰度一致，避免纯黑边影响模型推理。

### 8.3 RGA 操作 B: rga_convert_to_rgb_full_ (预览 1080p)

功能：1080P NV12 → 1080P BGR888，纯 1:1 格式转换，无缩放无 Letterbox。

```
rga_convert_to_rgb_full_(srcFd, srcWidth, srcHeight, srcStride, dstBuf)
  │
  ├─ importbuffer_fd(srcFd, 1920×1080, YCrCb_420_SP) → rga_handle_src
  ├─ importbuffer_fd(dstFd, 1920×1080, RGB_888)      → rga_handle_dst
  │
  ├─ improcess(rga_buf_src, rga_buf_dst, srect, drect)
  │    srect = {0, 0, 1920, 1080}
  │    drect = {0, 0, 1920, 1080}
  │
  └─ releasebuffer_handle(src/dst)
```

最简单的 RGA 操作，仅做色彩空间转换。输出格式为 `RK_FORMAT_BGR_888`，与 QT `QImage::Format_RGB888` 的通道顺序吻合，通过 `virtAddr` 可直接构造 `QImage`。原始输出的 RGB888 图像通过 `virtAddr` 可直接构造 `QImage`，供 QT 界面渲染。

### 8.4 RGA 操作 C: rga_copy_buffer_ (推流原图拷贝)

功能：1080P NV12 → 1080P NV12，同格式硬件拷贝。不使用 `improcess`，而是用 `imcopy`，RGA 按原始位深直接复制。

```
rga_copy_buffer_(srcFd, width, height, srcStride, dstBuf)
  │
  ├─ importbuffer_fd(srcFd, 1920×1080, YCrCb_420_SP) → rga_handle_src
  ├─ importbuffer_fd(dstFd, 1920×1080, YCrCb_420_SP) → rga_handle_dst
  │
  ├─ imcopy(rga_buf_src, rga_buf_dst)  // RGA 硬件块拷贝
  │
  └─ releasebuffer_handle(src/dst)
```

> **为什么需要拷贝而不是直接传递 V4L2 的 dmaFd？** V4L2 的 dmaFd 在 `VIDIOC_QBUF` 后所有权交还驱动，驱动可能立即用新帧覆盖。因此必须在本帧内通过 RGA 拷贝一份独立副本给下游消费者。

---

## 9. 消费者 API 详解

### 9.1 previewTaskQueue 消费者

### 9.1 NPU 消费者 (npuTaskQueue)

| API | 阻塞方式 | 返回值 | 使用场景 |
|-----|---------|--------|---------|
| `wait_get_npu(camNum)` | `pop()` 无限阻塞 | `DmaBuffer_t*`，无数据时线程挂起 | 消费者无需退出检查 |
| `try_get_npu(camNum, timeoutMs)` | `try_pop(timeoutMs)` 超时返回 | 超时返回 `nullptr` | 消费者需要周期性检查退出标志 |

```cpp
// 使用示例: YOLO 推理线程
while (running_) {
    DmaBuffer_t* npuBuf = visioner->try_get_npu(camNum, 200);
    if (npuBuf == nullptr) continue;
    // 使用 npuBuf->dmaFd 做 NPU 推理 (640×640 RGB888 + Letterbox + EIS)
    visioner->release_npu(camNum, npuBuf);
}
```

### 9.2 预览消费者 (previewTaskQueue)

| API | 阻塞方式 | 返回值 | 使用场景 |
|-----|---------|--------|---------|
| `wait_get_preview(camNum)` | `pop()` 无限阻塞 | `DmaBuffer_t*` | 消费者无需退出检查 |
| `try_get_preview(camNum, timeoutMs)` | `try_pop(timeoutMs)` 超时返回 | 超时返回 `nullptr` | 消费者需要周期性检查退出标志 |

```cpp
// 使用示例: QT PreviewWorker 子线程
while (running_) {
    DmaBuffer_t* previewBuf = visioner->try_get_preview(camNum, 200);
    if (previewBuf == nullptr) continue;
    // previewBuf->virtAddr 用于 QImage (BGR888 格式)
    visioner->release_preview(camNum, previewBuf);
}
```

### 9.3 推流/录像消费者 (processTaskQueue)

| API | 阻塞方式 | 返回值 |
|-----|---------|--------|
| `wait_get_orig_copy_buffer(camNum)` | `pop()` 无限阻塞 | `DmaBuffer_t*`，nullptr 表示 camNum 无效 |

```cpp
// 使用示例: SentinelStreamer 推流线程
while (threadRunning) {
    DmaBuffer_t* orig = visioner->wait_get_orig_copy_buffer(camNum);
    if (orig == nullptr) continue;

    // RGA 1080p → 720p 缩放 + MPP H.264 编码 + RTSP 推流

    visioner->release_orig_copy_buffer(camNum, orig);  // 必须归还
}
```

### 9.4 release API 归还机制

```cpp
release_npu(camNum, buf)               → npuRgbPool->release_buffer(buf)
release_preview(camNum, buf)           → previewPool->release_buffer(buf)
release_orig_copy_buffer(camNum, buf)  → origCopyPool->release_buffer(buf)
```

归还操作直接将 `DmaBuffer_t` 重新压入对应内存池的空闲链表（Free List），`ifUse` 标记复位，可供下一次 `get_buffer()` 取用。

> **必须归还**: 未归还的 buffer 会永久标记为 `ifUse=true`，从空闲链表移除，导致 `get_buffer()` 返回 `nullptr`，表现为 Drop Frame。长时间运行将耗尽所有 buffer，系统不可恢复。

---

## 10. 完整初始化流程: add_camera

```
add_camera(deviceName, width, height, bufferCount, camNum, camType)
  │
  ├─ 检查 camNum 是否已存在 → 重复则返回 false
  │
  ├─ 1. 创建 CameraContext，设置 camType 和 v4l2BufType
  │     ISP_CAM → V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
  │     USB_CAM → V4L2_BUF_TYPE_VIDEO_CAPTURE (单平面)
  │
  ├─ 2. open(deviceName, O_RDWR | O_NONBLOCK) → camFd
  │
  ├─ 3. 格式协商 (V4L2_BUF_TYPE 随 camType 而定)
  │     ISP: 直接 VIDIOC_S_FMT(NV12, MPLANE)
  │     USB: VIDIOC_S_FMT(NV12) → 失败则回退 VIDIOC_S_FMT(YUYV)
  │          都失败 → release + return false
  │     VIDIOC_G_FMT 回读实际格式和分辨率，更新 ctx->width / ctx->height
  │
  ├─ 4. USB YUYV 时分配 usbConvertPool (NV12, 同分辨率, bufferCount 块)
  │
  ├─ 5. 创建三个 DmaBufferPool (使用 ctx->width / ctx->height)
  │    ├─ npuRgbPool   (640,  640,  RGB888)
  │    ├─ origCopyPool (ctx->w, ctx->h, NV12)
  │    └─ previewPool  (ctx->w, ctx->h, RGB888)
  │
  ├─ 6. VIDIOC_S_PARM (尝试设置 30 FPS，失败忽略)
  │
  ├─ 7. VIDIOC_REQBUFS (V4L2_MEMORY_MMAP, bufferCount 个)
  │
  ├─ 8. 循环 bufferCount 次:
  │    ├─ VIDIOC_EXPBUF → 导出 DMA fd
  │    └─ VIDIOC_QBUF → 压入内核队列 (MPLANE 时条件化设置 m.planes)
  │
  ├─ 9. epoll_create1 + epoll_ctl(EPOLL_CTL_ADD, camFd, EPOLLIN)
  │
  └─ 10. _cameraContextMap[camNum] = std::move(ctx), 返回 true
```

**易踩坑的细节**:
- `VIDIOC_QBUF` 必须填充 `v4l2_plane` 数组（`buf.m.planes = planes; buf.length = 1`），即使是单 plane 模式。不填会导致 ioctl 返回 EINVAL。
- V4L2 设备以 `O_NONBLOCK` 打开，因为后续帧等待由 epoll 负责，非阻塞模式避免 `VIDIOC_DQBUF` 在没有帧时挂起。
- 任一步骤失败都会调用 `release_camera_resources_` 清理已分配的资源，防止半初始化状态泄漏。

---

## 11. camera_stream_ctrl 启停流程

### 11.1 开启流 (isOpen = true)

```
camera_stream_ctrl(camNum, true)
  │
  ├─ 查找 CameraContext → 不存在或已开启则返回 false/true
  │
  ├─ VIDIOC_STREAMON (启动 V4L2 硬件流)
  │    └─ 失败 → 返回 false
  │
  ├─ isStreaming = true
  ├─ isThreadRunning = true
  ├─ captureThread = new std::thread(capture_thread_func_, camNum)
  │
  └─ 返回 true
```

### 11.2 关闭流 (isOpen = false)

```
camera_stream_ctrl(camNum, false)
  │
  ├─ 查找 CameraContext → 不存在或已关闭则返回 false/true
  │
  ├─ isThreadRunning = false (通知线程退出)
  │
  ├─ captureThread->join() (等待线程函数返回)
  ├─ captureThread->reset() (释放线程对象)
  │
  ├─ VIDIOC_STREAMOFF (停止 V4L2 硬件流)
  │
  ├─ isStreaming = false
  │
  └─ 返回 true
```

> **关键顺序**: `STREAMOFF` 必须在 `thread.join()` 之后调用。如果在 join 之前关闭硬件流，capture 线程持有的 epoll fd 上可能收到错误事件，导致 `VIDIOC_DQBUF` 失败并触发线程退出逻辑中的 ioctl 竞态。

---

## 12. camera_pause 暂停机制

### 12.1 设计动机

在 RK3588 平台上，`VIDIOC_STREAMOFF` 会卸载 ISP 管线，仅靠 `VIDIOC_STREAMON` 无法恢复（需重新配置 ISP 参数）。因此，当上游需要临时停止处理（如关闭预览界面）但不希望重建管线时，使用 `camera_pause` 而非 `camera_stream_ctrl(false)`。

### 12.2 实现

```
camera_pause(camNum, paused)
  └─ ctx->isPaused.store(paused)

capture_thread_func_ 中的处理:
  if (ctx->isPaused.load()) {
      VIDIOC_QBUF  // 仅归还 buffer，不执行任何 RGA 操作
      continue      // 不写入任何队列
  }
```

### 12.3 完整启停序列

```
暂停:
  1. 停止下游消费者 (如 PreviewWorker 子线程 join)
  2. camera_pause(cam, true)        // capture 线程跳过 RGA，仅 QBUF 循环

恢复:
  1. camera_pause(cam, false)       // capture 线程恢复 RGA 处理，帧立即产生
  2. 启动下游消费者 (如 PreviewWorker 子线程)
```

> **注意**: 暂停期间 V4L2 驱动持续填充 buffer，但消费者队列不再有新数据入队。下游的 `wait_get_preview` 调用会一直阻塞。必须在暂停前先停止下游消费者线程。

---

## 13. release_camera_resources_ 资源清理

```
release_camera_resources_(ctx)
  │
  ├─ 关闭所有 V4L2 export 的 DMA fd: close(bufInfo.dmaFd)
  │
  ├─ 关闭 epoll fd: close(ctx->epollFd)
  │
  ├─ 关闭 V4L2 设备 fd: close(ctx->camFd)
  │
  ├─ 销毁 npuRgbPool:   destroy_pool() + reset()
  ├─ 销毁 origCopyPool: destroy_pool() + reset()
  ├─ 销毁 previewPool:  destroy_pool() + reset()
  └─ 销毁 usbConvertPool: destroy_pool() + reset() (若存在)
```

**调用时机**:
- `add_camera` 中任一步骤失败时，清理已分配资源
- `~SentinelVisioner()` 析构时，遍历 `_cameraContextMap` 逐一清理

**析构函数清理顺序**:

```cpp
~SentinelVisioner() {
    for (auto& pair : _cameraContextMap) {
        if (pair.second->isStreaming) {
            camera_stream_ctrl(pair.first, false);   // 先关闭硬件流 + join 线程
        }
        release_camera_resources_(pair.second.get()); // 再释放 fd 和内存池
    }
    _cameraContextMap.clear();
}
```

**关键**: `DmaBufferPool::destroy_pool()` 通过 `allBuffers_` 全量花名册逐一释放底层 DMA 内存（`dma_buf_free` + `close`），确保即使有未归还的 buffer 也不会泄漏。

---

## 14. 错误处理

| 场景 | 处理方式 |
|------|---------|
| `add_camera` 中 ioctl 失败 | 调用 `release_camera_resources_` 清理已分配资源，返回 false |
| `VIDIOC_DQBUF` 失败 (EAGAIN) | `continue`，等待下一帧 |
| `VIDIOC_DQBUF` 失败 (其他错误) | `isThreadRunning = false`，退出线程 |
| RGA 操作 A 或 B 失败 | 归还已分配的 targetNpuBuf / targetPreviewBuf，不推送到队列 |
| RGA 操作 C 失败 | 归还 targetOrigBuf，不推送到 processTaskQueue |
| `get_buffer()` 返回 nullptr | 释放已获取的 buffer，丢弃当前帧（Drop Frame），打印警告 |
| `VIDIOC_QBUF` 失败 | `isThreadRunning = false`，退出线程 |

**缓冲区干涸策略**: 当任一 `DmaBufferPool` 的 `get_buffer()` 返回 `nullptr`，意味着下游消费者归还速度跟不上捕获速度。此时直接丢弃当前帧并继续 QBUF，避免整个管线因一个池为空而卡死。

---

## 15. USB 相机支持（CameraType 枚举）

### 15.1 设计思路

USB UVC 摄像头与 MIPI CSI 摄像头的 V4L2 接口在三个层面不同：

| 差异点 | ISP (MIPI CSI) | USB (UVC) |
|--------|---------------|-----------|
| Buffer 类型 | `V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE` | `V4L2_BUF_TYPE_VIDEO_CAPTURE` (单平面) |
| 像素格式 | NV12 (ISP 硬件输出) | YUYV、NV12 或 MJPG (因摄像头而异) |
| planes 数组 | QBUF/DQBUF 必须设置 `v4l2_plane` | 不需要，设了反而出错 |
| stride | 无 padding | 可能有 bytesperline > width |

设计策略：调用者通过 `CameraType` 枚举显式指定相机类型，库内据此分流初始化路径。USB 相机优先尝试 NV12 原生格式（零额外 RGA 开销），不支持时回退 YUYV + RGA 硬件转换。无论哪种路径，最终都产出 NV12 喂入统一的三 RGA 下游管线。

### 15.2 格式协商流程

```
add_camera(dev, w, h, bufCnt, camNum, USB_CAM)
  │
  ├─ v4l2BufType = V4L2_BUF_TYPE_VIDEO_CAPTURE (单平面)
  │
  ├─ VIDIOC_S_FMT(NV12)
  │    ├─ 成功 → actualPixelFormat = NV12, 按需分配 usbSafePool
  │    └─ 失败 → VIDIOC_S_FMT(YUYV)
  │              ├─ 成功 → actualPixelFormat = YUYV, 分配 usbConvertPool
  │              └─ 失败 → return false
  │
  ├─ VIDIOC_G_FMT 回读实际格式、分辨率和 bytesperline
  │    └─ actualPixelFormat == MJPG → 分配 mjpegDecodePool
```

### 15.3 捕获线程中的 YUYV→NV12 转换

当 `actualPixelFormat == V4L2_PIX_FMT_YUYV` 时，捕获线程在 DQBUF 后、三个 RGA 操作前插入一次格式转换：

```
VIDIOC_DQBUF → currentDmaFd (YUYV)
  │
  ├─ usbConvertPool->get_buffer() → convBuf
  ├─ rga_yuyv_to_nv12_(currentDmaFd, width, height, convBuf)
  │    RGA: RK_FORMAT_YVYU_422 → RK_FORMAT_YCrCb_420_SP (1:1, 无缩放)
  ├─ nv12DmaFd = convBuf->dmaFd
  │
  ├─ 操作 A/B/C 使用 nv12DmaFd (NV12) 作为源
  │
  └─ usbConvertPool->release_buffer(convBuf)
       VIDIOC_QBUF
```

转换失败或 convert pool 干涸时：释放已分配的转换缓冲，归还 V4L2 buffer，丢弃当前帧。不向下游队列投递任何脏数据。

### 15.4 错误处理补充

| 场景 | 处理方式 |
|------|---------|
| USB 相机拒绝 NV12 且拒绝 YUYV | `add_camera()` 返回 false，打印错误信息 |
| USB convert pool 干涸 | 丢弃当前帧，QBUF 归还 V4L2 buffer，打印警告 |
| RGA YUYV→NV12 转换失败 | 释放 convert buffer、QBUF、continue |
| 暂停模式下的 convert buffer | 暂停前先释放 convert buffer，再 QBUF |

---

## 16. 性能特征

| 指标 | 数据 | 说明 |
|------|------|------|
| 端到端延迟 | ~64 ms | V4L2 捕获 + 3 次 RGA + 线程通信 |
| 捕获线程 CPU | ~8.7% (单核) | 繁重像素运算已卸载至 RGA |
| 消费者空闲 CPU | 0.0% | 条件变量休眠 |
| RGA 负载 | ~5% | scheduler[0] (rga3)，单帧处理三次 |
| 进程 RES | ~1.7 MB | 图像数据仅在内核态 DMA 区 |
| 进程 VIRT | ~264 MB | DMA Buffer Pool 映射 |
| Fd 数量 | 恒定 38 个 | 高频压测无泄漏 |

---

## 17. 代码入口点速查

| 功能 | 函数 | 文件:行号 |
|------|------|-----------|
| 相机初始化 | `add_camera()` | `src/sentinel-visioner.cpp` |
| 流启停 | `camera_stream_ctrl()` | `src/sentinel-visioner.cpp` |
| 捕获线程 | `capture_thread_func_()` | `src/sentinel-visioner.cpp` |
| 暂停/恢复 | `camera_pause()` | `src/sentinel-visioner.cpp` |
| NPU 小图 RGA | `rga_process_to_rgb_()` | `src/sentinel-visioner.cpp` |
| 预览转换 RGA | `rga_convert_to_rgb_full_()` | `src/sentinel-visioner.cpp` |
| 拷贝 RGA | `rga_copy_buffer_()` | `src/sentinel-visioner.cpp` |
| YUYV→NV12 RGA | `rga_yuyv_to_nv12_()` | `src/sentinel-visioner.cpp` |
| MJPG→NV12 解码 | `mjpeg_decode_to_nv12_()` | `src/sentinel-visioner.cpp` |
| EIS 回调设置 | `set_eis_offset_callback()` | `src/sentinel-visioner.cpp` |
| EIS 平滑系数 | `set_eis_smooth_alpha()` | `src/sentinel-visioner.cpp` |
| NPU 消费 | `wait_get_npu` / `try_get_npu` / `release_npu` | `src/sentinel-visioner.cpp` |
| 预览消费 | `wait_get_preview` / `try_get_preview` / `release_preview` | `src/sentinel-visioner.cpp` |
| 推流消费 | `wait_get_orig_copy_buffer` / `release_orig_copy_buffer` | `src/sentinel-visioner.cpp` |
| 资源清理 | `release_camera_resources_()` | `src/sentinel-visioner.cpp` |
| 阻塞队列基础 | `ThreadSafeQueue::pop` / `push` | `include/ThreadSafeQueue.h` |
| 超时队列 | `ThreadSafeQueue::try_pop(timeoutMs)` | `include/ThreadSafeQueue.h` |
| 非阻塞弹出 | `ThreadSafeQueue::try_pop(val)` | `include/ThreadSafeQueue.h` |
| 跳帧取最新 | `ThreadSafeQueue::drain_latest` | `include/ThreadSafeQueue.h` |
