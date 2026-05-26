# SentinelStreamer — 技术实现文档

## 1. 概述

SentinelStreamer 是 RK3588-Omni-Sentinel 平台的推流与录像组件，作为 `SentinelVisioner` 的下游消费者，从 `processTaskQueue` 拉取 1080p NV12 原始帧，经 RGA 硬件缩放后由 MPP 硬件编码为 H.264，推流通过 `ffmpeg` 子进程管道输出 RTSP，录像通过 FFmpeg API 写入 MP4。

---

## 2. 架构总览

```
SentinelVisioner::capture_thread_
  │
  ├── previewTaskQueue    → NPU/预览消费者 (本组件不涉及)
  │
  └── processTaskQueue    → SentinelStreamer 推流线程
                              │
                              ├── RGA 缩放 (1080p → 720p)
                              │     硬件: rga3, < 3% 负载
                              │
              ┌───────────────┼───────────────┐
              ▼                               ▼
       streamEncCtx (720p)            recordEncCtx (1080p/720p)
       h264_rkmpp 编码, 惰性创建       h264_rkmpp 编码, 惰性创建
              │                               │
              ▼                               ▼
       H.264 裸流 fwrite              av_write_frame
              │                               │
              ▼                               ▼
       ffmpeg 子进程 pipe             MP4 容器文件
       (断线自动重连)
              │
              ▼
          RTSP 推流
              │
              ▼
      StreamerCallback 状态回调
```

**设计原则**:
- 推流和录像各自独立 MPP 编码器，惰性创建，互不干扰
- ffmpeg 子进程通过 `popen` 管理，崩溃不影响主程序，`ferror` 检测后自动重连
- PTS 使用硬件时间戳（`CLOCK_MONOTONIC` 减首帧偏移），帧率波动不影响播放速度
- 所有共享资源（编码器、输出上下文）严格在线程 `join()` 后销毁
- 状态变更通过 `StreamerCallback` 回调通知调用方

---

## 3. 模块划分

| 文件 | 职责 |
|------|------|
| `include/sentinel_streamer.h` | 公共 API 头文件（含回调、事件类型） |
| `src/sentinel_streamer.cpp` | 核心实现：线程管理、帧循环调度、生命周期 |
| `src/mpp_encoder.h` | MPP 编码器 + ffmpeg 管道内部头文件 |
| `src/mpp_encoder.cpp` | MPP 编码器封装、ffmpeg 子进程管理、MP4 复用器 |
| `src/rga_scaler.cpp` | RGA 硬件 1080p→720p NV12 缩放 |
| `src/demo_stream.cpp` | 基础推流+录像 Demo |
| `src/demo_cycle.cpp` | 反复启停循环压测 Demo |

---

## 4. 核心数据结构

### 4.1 StreamerContext (内部，不透明)

每路摄像头的完整推流/录像上下文：

```cpp
struct StreamerContext {
    int camNum;                          // 摄像头编号
    SentinelVisioner* visioner;          // 上游帧源
    std::atomic<bool> threadRunning;     // 线程退出标志
    bool streamEnabled, recordEnabled;   // 启停标志
    StreamOsdMode osdMode;              // OSD 模式
    RecordResolution recordResolution;   // 录像分辨率

    std::thread workerThread;            // 推流线程
    DmaBufferPool* scale720pPool;        // 720p 中间缩放缓冲（大小可配，默认 4）

    AVCodecContext* streamEncCtx;        // 推流编码器（惰性创建，1280×720）
    AVCodecContext* recordEncCtx;        // 录像编码器（惰性创建，1080p/720p）
    FILE* ffmpegPipe;                    // ffmpeg 子进程管道写端
    char  streamUrl[256];               // RTSP URL（用于断线重连）
    AVFormatContext* mp4Ctx;             // MP4 输出上下文
};
```

### 4.2 SentinelStreamer (公共 API)

```cpp
class SentinelStreamer {
public:
    // 生命周期
    bool add_camera(int camNum, SentinelVisioner* visioner, int poolSize = 4);
    bool remove_camera(int camNum);

    // 推流
    bool start_stream(int camNum, const char* rtspUrl);
    bool stop_stream(int camNum);
    bool is_streaming(int camNum) const;

    // 推流 OSD 模式
    bool set_stream_osd_mode(int camNum, StreamOsdMode mode);

    // 录像
    bool start_record(int camNum, const char* filePath, RecordResolution resolution);
    bool stop_record(int camNum);
    bool is_recording(int camNum) const;

    // 状态回调
    void set_callback(StreamerCallback cb);

private:
    StreamerContext* contexts_[2];  // 最多 2 路摄像头
};
```

### 4.3 状态回调

```cpp
enum class StreamerEvent {
    STREAM_STARTED = 0,  // detail = RTSP URL
    STREAM_STOPPED = 1,  // detail = nullptr
    RECORD_STARTED = 2,  // detail = MP4 文件路径
    RECORD_STOPPED = 3,  // detail = nullptr
    ERROR          = 4,  // detail = 错误描述
};

using StreamerCallback = void (*)(int camNum, StreamerEvent event, const char* detail);
```

回调在 SentinelStreamer 内部线程调用。启停时通知，ffmpeg 重连成功/失败也通知。

---

## 5. 线程模型

### 5.1 线程创建与销毁

```
main thread                     stream thread (每路 1 个)
    │                                │
    ├─ start_stream()                │
    │   ├─ 惰性创建 streamEncCtx     │
    │   ├─ popen ffmpeg              │
    │   ├─ streamEnabled = true      │
    │   ├─ 回调 STREAM_STARTED       │
    │   └─ workerThread = thread()   │
    │                                ├─ 首帧记录 firstTsUs
    │                                ├─ while(threadRunning)
    │                                │    ├─ wait_get_orig_copy_buffer()
    │                                │    ├─ RGA scale
    │                                │    ├─ encode + mux
    │                                │    ├─ 释放 buffer
    │                                │    └─ 检测 ferror → 重连 ffmpeg
    │                                │
    ├─ stop_stream()                 │
    │   ├─ threadRunning = false     │   (线程检测到 false，退出循环)
    │   ├─ workerThread.join()  ────┤
    │   ├─ pclose ffmpeg             │
    │   ├─ 销毁 streamEncCtx         │
    │   └─ 回调 STREAM_STOPPED       │
```

### 5.2 线程安全规则

| 规则 | 说明 |
|------|------|
| `threadRunning` | `atomic<bool>`，release/acquire 语义 |
| `streamEnabled` / `recordEnabled` | 仅 main thread 写入，stream thread 读取 |
| 编码器 / 输出上下文 | **只在 `join()` 后销毁**（最高原则） |
| DMA 缓冲区 | SentinelVisioner 内部 mutex 保护，stream thread 仅消费者 |
| `ffmpegPipe` / `mp4Ctx` | 仅 stream thread 写入，main thread 在 join 后关闭 |

---

## 6. PTS 管理

使用 **录制启动时的系统时钟作为 PTS 基准**，跳过队列积压旧帧，兼容任意帧率：

```
启动录制: baselineTsUs = clock_gettime(CLOCK_MONOTONIC)
每帧检查: if (timestampUs < baselineTsUs) skip  // 跳过录制前积压的旧帧
每帧 PTS: pts = (timestampUs - baselineTsUs) × 90000 / 1000000
```

| 项目 | 值 |
|------|-----|
| PTS 来源 | `DmaBuffer_t::timestampUs`（V4L2 `CLOCK_MONOTONIC`） |
| 归零基准 | 录制按钮按下时的系统时钟 `baselineTsUs` |
| 旧帧过滤 | `tsUs < baselineTsUs` 的帧直接跳过并归还 DMA 缓冲 |
| 编码器 time_base | `{1, 90000}` (MPEG 标准时基) |
| pkt->pts/dts | **始终**强制覆盖为 `sentPts`（不依赖 `AV_NOPTS_VALUE`） |
| pkt->duration | 不设，由 FFmpeg 根据帧间 PTS 自动计算 |

---

## 7. 编码器生命周期（惰性创建）

```
add_camera(poolSize):
  └─ 创建 scale720pPool (poolSize 帧)
  └─ 不建编码器（延迟到真正用时）

start_stream:
  └─ streamEncCtx == nullptr → mpp_encoder_open(720p)

stop_stream:
  └─ 线程已 join → mpp_encoder_close(streamEncCtx)

start_record:
  └─ recordEncCtx == nullptr 或分辨率变 → mpp_encoder_open(...)

stop_record:
  └─ 线程已 join → mpp_encoder_close(recordEncCtx)
```

**结果**: 只推流不录像时只有 `streamEncCtx`，只录像不推流时只有 `recordEncCtx`，资源按需分配。

---

## 8. ffmpeg 子进程断线重连

```
每帧结束后:
  if (ffmpegPipe && streamEnabled && ferror(ffmpegPipe)):
    1. ffmpeg_stream_close(ffmpegPipe)        // pclose 旧管道
    2. ffmpegPipe = ffmpeg_stream_open(url)   // popen 新管道
    3. 成功 → 回调 STREAM_STARTED
       失败 → 回调 ERROR("ffmpeg reconnect failed")
```

`ferror()` 检测管道断开（子进程崩溃或网络闪断），全自动无感重连。下一帧继续推流到新管道。

---

## 9. 错误处理

| 场景 | 处理方式 |
|------|---------|
| 编码器打开失败 | `bool` 返回 false，打印日志 |
| 编码帧失败 | 跳过当前帧，继续下一帧 |
| RGA 缩放失败 | 丢弃 scaleBuf，跳过推流/720p 录像 |
| ffmpeg 子进程崩溃 | `ferror` 检测后自动 `popen` 重建 |
| ffmpeg 重连失败 | 回调 `ERROR`，下帧继续尝试 |
| DMA 缓冲池空 | `get_buffer` 返回 nullptr，丢弃当前帧 |
| 输出上下文/编码器野指针 | 严格在线程 join 后销毁，防止 use-after-free |

---

## 10. 内存布局

| 资源 | 大小 | 说明 |
|------|------|------|
| scale720pPool | poolSize × 1280×720×1.5 ≈ 1.4MB/帧 | 720p NV12 中间缓冲，大小可配 |
| streamEncCtx | 仅推流时存在 | MPP 编码器上下文 |
| recordEncCtx | 仅录像时存在 | MPP 编码器上下文 |
| 进程 RES | ~13 MB | 实际常驻物理内存 |
| 进程 VIRT | ~400 MB | DMA Buffer Pool 映射 |

---

## 11. 性能指标

| 指标 | 基础 Demo | 循环 Demo | 说明 |
|------|-----------|-----------|------|
| 主线程 CPU | 0% | 0% | 休眠等待 |
| 捕获线程 CPU | 2.0% | 2.6% | V4L2/RGA 调度 |
| 推流线程 CPU | 0.7% | 0.7% | DMA/编码调度 |
| ffmpeg 子进程 CPU | 25~38% | 38~42% | RTSP 协议栈 (纯软件) |
| RGA 负载 | 3% | 3% | rga3 scheduler[0] |
| Fd 数量 | 恒定 | 恒定 | 无泄漏 |
