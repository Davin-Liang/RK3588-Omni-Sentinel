# SentinelStreamer — 学习指南

## 目标

面试时能说清：做了什么、为什么这么设计、踩过什么坑。

---

## 第一层：能说清"做了什么"（面试讲项目用）

### 一句话概括

> 把摄像头的 NV12 帧，经 RGA 硬件缩放后，MPP 硬件编码为 H.264，推流用 `ffmpeg` 子进程管道输出 RTSP，录像用 FFmpeg API 写入 MP4。

### 数据流（能画出来）

```
V4L2 摄像头
  │
  ▼
SentinelVisioner（取帧、队列分发）
  │
  └─ processTaskQueue  ─→  SentinelStreamer 推流线程
                              │
                              ├─ RGA 硬件缩放 (1080p → 720p)
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
            streamEncCtx          recordEncCtx
            h264_rkmpp 编码       h264_rkmpp 编码
                    │                   │
                    ▼                   ▼
              fwrite pipe          av_write_frame
                    │                   │
                    ▼                   ▼
             ffmpeg 子进程          MP4 文件
                    │
                    ▼
               RTSP 推流
```

### 关键代码（背下来）

```cpp
// 3 步用法，面试能张口就来
SentinelStreamer streamer;
streamer.add_camera(0, &visioner);             // 注册
streamer.start_stream(0, "rtsp://...");        // 推流
streamer.start_record(0, "/tmp/a.mp4", RES_1080P); // 录像
// ... 运行 ...
streamer.stop_record(0);
streamer.stop_stream(0);
streamer.remove_camera(0);
```

### Qt 集成（能解释）

```cpp
// 通过回调把内部事件发射为 Qt 信号
streamer.set_callback([](int cam, StreamerEvent e, const char* detail) {
    emit signalFromStreamer(cam, e, detail);
});
// start/stop 从 Qt 槽函数直接调，内部线程不阻塞 UI
```

---

## 第二层：能解释"为什么这么设计"（面试追问用）

### 决策 1：PTS 为什么用硬件时间戳而不是帧计数器？

| 方案 | 帧计数器（我们最初用的） | 硬件时间戳（现在用的） |
|------|------------------------|----------------------|
| 原理 | 每帧 PTS += 6000 | `(timestampUs - 首帧) × 90000 / 1000000` |
| 帧率不变时 | 没问题 | 没问题 |
| 帧率波动/降帧时 | 播放加速/慢放 | 始终正确，反映真实时间 |

**教训**: 帧计数器假设帧率恒定，实际硬件会降频。

### 决策 2：为什么编码器惰性创建？

```
只推流不录像 → 只建 streamEncCtx，recordEncCtx = nullptr
只录像不推流 → 只建 recordEncCtx，streamEncCtx = nullptr
两路都开   → 两个都建
```

`add_camera` 只建缩放池，编码器在 `start_stream` / `start_record` 时才创建。不用不占资源。

### 决策 3：为什么 ffmpeg 走子进程管道，不用 FFmpeg C API 的 RTSP muxer？

```
我们的代码 → fwrite(H.264) → pipe → ffmpeg 子进程 → RTSP
                                      ↑
                                崩溃了？pclose + popen 重建
                                主程序不受影响
```

FFmpeg C API 的 RTSP 输出在反复启停下状态不稳定（`av_log` 回调野指针崩溃），子进程完全隔离崩溃域。

### 决策 4：为什么编码器/输出上下文严格在 `join()` 后销毁？

```
主线程                           推流线程
  │                                │
  ├─ stop_record()                 ├─ while(threadRunning)
  │   ├─ recordEnabled = false     │    ├─ 检查 recordEnabled ✓
  │   │  （线程看到后不再用）       │    ├─ encode_and_mux(recordEncCtx)
  │   │                            │    │   ↑ 如果主线程这时销毁？
  │   │                            │    │   野指针 → SIGSEGV
  │   │                            │    │
  │   ├─ threadRunning = false     │    ├─ 检测到 false，退出循环
  │   ├─ workerThread.join() ←────┤  线程退出
  │   ├─ mpp_encoder_close()      ← 安全：线程已不在
  │   └─ mp4_output_close()       ← 安全
```

**原则**: 线程还在跑的时候，绝不销毁它可能访问的任何东西。

---

## 第三层：能讲清 bug 和教训（面试加分项）

### 从 BUG_RECORD.md 选 3 个最有代表性的

**1. CLOCK_MONOTONIC 时间戳导致 MP4 黑屏 37 分钟**

- 现象：录 30 秒，文件显示 37 分钟，全黑
- 原因：V4L2 时间戳是系统启动后的累计微秒（5000 秒+），直接做 PTS
- 修复：首帧时间戳做偏移，PTS 归零
- **面试话术**: "一开始直接用了硬件时间戳，没意识到是单调时钟的绝对值，播放器等 PTS 走完才显示画面"

**2. use-after-free 的三次演变**

第一次：`stop_record` 销毁 `recordEncCtx`，线程还在用 → SIGSEGV
修复：编码器销毁推迟到 `join()` 后

第二次：同上，但这次是 `mp4Ctx` 输出上下文 → SIGSEGV
修复：输出上下文也推迟到 `join()` 后

第三次：ffmpeg RTSP API 内部 `av_log` 回调野指针 → SIGSEGV
修复：放弃 FFmpeg C API RTSP，改用子进程管道

- **面试话术**: "多线程共享资源的生命周期是最容易出错的地方。我们三次遇到 use-after-free，每次根因都一样——没等消费者线程退出就销毁了它用的东西。最后的解决方案很简单：线程 join 之前，什么都不销毁。"

**3. 反复启停时 DTS 不单调**

- 现象：第二轮循环报错 `702000 >= 0`
- 原因：第一轮停止时编码器没销毁，内部残留 117 帧，第二轮先吐旧帧
- 修复：每轮停止时销毁编码器，下一轮新建
- **面试话术**: "嵌入式编码器有内部缓冲，不销毁就新建输出上下文，旧帧会污染新视频的时基"

---

## 怎么对着代码学

**别死记硬背。跟一遍数据流：**

1. 打开 `src/sentinel_streamer.cpp`，找到 `stream_thread_func_`
2. 从 `wait_get_orig_copy_buffer` 开始，跟着注释走一帧
3. 看到 `release_orig_copy_buffer` 结束，这就是一帧的完整生命周期
4. 再回头看 `start_stream` / `stop_stream`，理解启停流程

**重点函数入口:**

| 函数 | 作用 |
|------|------|
| `stream_thread_func_` | 理解一帧怎么从队列→编码→输出 |
| `add_camera` | 理解初始化做了什么（现在只建缩放池） |
| `start_stream` | 理解编码器惰性创建 + ffmpeg 子进程启动 |
| `stop_stream` | 理解线程何时停、编码器何时销毁 |
| `encode_and_mux` | 理解 H.264 如何分支到 pipe 和 MP4 |
| `notify_` | 理解回调何时触发 |
