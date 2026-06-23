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

PTS 计算的关键代码（`stream_thread_func_` 线程内）：

```cpp
// 1. start_stream()/start_record() 启动时，记录"此刻"为基准
ctx->baselineTsUs = current_monotonic_time_in_us();

// 2. 线程启动时，快照一份作为 PTS 原点
uint64_t firstTsUs = ctx->baselineTsUs;

// 3. 丢弃时间戳早于启动时刻的旧帧
if (tsUs < firstTsUs) { 释放并 continue; }

// 4. 硬件时间戳 → PTS：归零 + 时基转换
int64_t pts = (tsUs - firstTsUs) * 90000 / 1000000;
```

| 术语 | 对应代码 | 含义 |
|------|---------|------|
| **帧计数器**（替代方案） | `pts += 6000` | 每帧固定加 6000（90kHz 下约 15fps），假设帧率绝对稳定。帧率波动/降帧时播放会加速或慢放 |
| **硬件时间戳**（我们的方案） | `tsUs = origBuf->timestampUs` | V4L2 驱动打的 CLOCK_MONOTONIC 微秒时间戳，反映帧的真实曝光时刻 |
| **首帧偏移 / baselineTsUs** | `ctx->baselineTsUs = now` | start 被调用时的系统时刻，PTS 从这里算起，确保第一帧 PTS 接近 0 |
| **baselineTsUs 归零** | `(tsUs - firstTsUs)` | 所有帧减去同一基准，本质是把绝对时间戳平移成以启动时刻为零点的相对时间 |
| **旧帧过滤** | `if (tsUs < firstTsUs)` | 丢弃启动前积压的旧帧，避免 PTS 为负数或首帧跳跃 |

举个例子：假设摄像头以 29.7fps 跑了 10 分钟。帧计数器每帧固定 +6000，累积偏移约 6 秒，音视频不同步。硬件时间戳用每帧的真实曝光时刻做 PTS，完全不受帧率波动影响。

**面试话术**: "一开始直接用帧计数器做 PTS，没意识到实际硬件帧率会波动。后来改用 CLOCK_MONOTONIC 硬件时间戳——启动时记录 baselineTsUs 做归零，所有帧减去这个基准转成相对时间，再乘以 90000/1000000 转成 MPEG 时基。还有一个细节是旧帧过滤：摄像头一直在跑，队列里可能积压了启动前的帧，必须丢。最后用实际例子说明——29.7fps 跑 10 分钟，帧计数器偏差能达到 6 秒。"

### 决策 2：为什么编码器惰性创建？

```
只推流不录像 → 只建 streamEncCtx，recordEncCtx = nullptr
只录像不推流 → 只建 recordEncCtx，streamEncCtx = nullptr
两路都开   → 两个都建
```

`add_camera` 只建缩放池，编码器在 `start_stream` / `start_record` 时才创建。不用不占 MPP 硬件资源。

**编码器状态问题**：MPP 编码器内部维护帧序号、GOP 计数、码率控制参数等一整组运行时状态。不销毁重建的话，第二轮推流时帧序号从 5001 开始，但新流时间戳从 0 开始——VLC 播放器拿到 PTS=0 的帧，帧头却写着"第 5001 帧"，直接黑屏或报"无法播放"。RTSP 推流场景下客户端反复断开重连，始终无法建立稳定会话。每轮 stop 销毁 → start 重建，整组状态清零。

**为什么不调 `avcodec_flush_buffers()` 而要销毁重建？** FFmpeg 提供了 `avcodec_flush_buffers()` 可以重置编码器内部状态——理论上比销毁重建更轻量。但 `h264_rkmpp` 是 Rockchip 社区维护的硬件编码器 wrapper，其 flush 实现是否清空帧序号、GOP 计数、码率控制历史窗口，文档没有明确承诺。对第三方硬件 wrapper 的内部行为做假设，一旦 flush 漏掉某个冷门状态字段，排查代价远超多花几十毫秒重建。销毁→alloc→open 三步走，整个 context 都不存在了，100% 确定状态清零。这是防御性编程——不赌硬件 wrapper 的实现质量。

**关于 B 帧**：实时推流场景关闭 B 帧是业界通用做法——WebRTC 规范明确禁止 B 帧，视频会议（Zoom、腾讯会议）一律不用，安防监控的 RTSP 推流也普遍关闭。B 帧要等"未来帧"到达才能编码，每级 B 帧至少多 1 帧延迟（30fps 下 ≈ 33ms），实时场景要的是低延迟不是压缩率。我们用 `max_b_frames = 0` 关掉 B 帧——结果流中 PTS ≡ DTS。DTS 这个术语本身是通用 H.264 概念，值得了解以备面试，但对我们项目而言，编码器整体状态的累积才是实际的坑。

**面试话术**: "惰性创建不只是省资源。编码器是有状态的——帧序号、GOP 计数、码率控制都在内部维护。不销毁重建的话新流 PTS 从 0 开始，帧头却写着第 5001 帧，VLC 直接黑屏。我们用 `max_b_frames = 0` 关 B 帧是实时推流的通用做法——WebRTC、视频会议、安防监控都一样——但整体状态的跨启停累积才是实际问题，销毁重建是最彻底的清零方案。"

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

### 决策 5：为什么需要 RecordBufferPool 环形帧缓冲？

**需求**：当融合算法检测到告警（如人员进入警戒区）时，需要回溯告警发生前几秒的原始图像帧，用于事后排查和取证。

**方案**：在编码前用 RGA DMA 硬件拷贝暂存 NV12 帧到环形缓冲区。

```
推流线程: 每帧 → RGA imcopy → RecordBufferPool（150 槽 ≈ 5s @30fps）
外部线程: try_get_record_frame() → 写入磁盘 → release_record_frame()
```

| 对比维度 | 从 MP4 录像文件取帧（替代方案） | RecordBufferPool 环形缓冲（我们的方案） |
|---------|-------------------------------|--------------------------------------|
| 延迟 | 必须先写 MP4 再 seek 解码，延迟数秒 | 编码前直接取，零延迟 |
| 格式 | 需解码 H.264，CPU 开销大 | 原始 NV12，无需解码 |
| 灵活性 | 只能取已写入磁盘的帧 | 可回溯最近 N 秒的任意帧 |
| 资源开销 | 无额外内存 | 150 帧 NV12 × 1080p ≈ 230 MB DMA 内存 |

**面试话术**: "告警回溯需要历史帧，但 MP4 录像有编码延迟且需要解码。我们在编码前用 RGA 硬件 DMA 拷贝把 NV12 帧暂存到环形缓冲里，消费端非阻塞 FIFO 取帧。RGA 拷贝是纯硬件操作，不占 CPU；150 个槽位存满后自动覆盖最老的帧，始终保留最近 5 秒数据。"

---

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
| `draw_osd_boxes_` | 理解 640×640 NPU → 720p 坐标变换 + NV12 Y 平面直写 |
| `set_osd_provider` | 理解回调解耦设计 |

---

### 决策 6：OSD 叠加为什么用 CPU 直写 NV12 而不是 RGA 合成？

**需求**：推流画面上叠加 YOLO 检测框（白色边框 + 标签文字）。

**方案**：在 1280×720 NV12 DMA buffer 的 Y 平面直接用 CPU 写像素，不经过 RGA。

| 对比维度 | RGA imcompose 合成（替代方案） | CPU 直写 Y 平面（我们的方案） |
|---------|-------------------------------|---------------------------|
| 延迟 | 分配额外 buffer → importbuffer_fd → ioctl → RGA 执行 → release，~200μs | 直接 mmap DMA buffer 写内存，< 5μs |
| 适用场景 | 全帧 alpha 融合、大区域叠加 | 稀疏边框+文字（~3000 px/bbox） |
| 复杂度 | 需管理额外 RGA buffer | 仅 Y 平面循环写像素 |

**面试话术**："OSD 叠加我们没用 RGA 硬件合成。画几个检测框就几千个像素，CPU 直写 DMA buffer 的 Y 平面只要几微秒，比 RGA 的 ioctl 来回快几十倍。只在做整帧 alpha 融合的时候 RGA 才划算。"

---

### Bug 故事：USB 相机 OSD 延迟 5-6 秒

**现象**：MIPI 相机 OSD 实时跟随人物，USB 相机 OSD 在人物移动后停留 5-6 秒才更新。

**原因**：YOLO 推理结果通过 `ThreadSafeQueue`（FIFO）传给 streamer。USB NPU 管线产帧 30fps，推流消费 15fps，队列持续积压。每次从队首取最旧帧 — 显示的是 5 秒前的检测结果。

**解决**：OSD provider 改为 `while(try_get_osd_result(0))` 清空队列只保留最新一帧，延迟降至 1 帧以内。

**面试话术**："这个问题本质是生产者快于消费者导致的 FIFO 队列积压。把 OSD 的取帧策略从'取队首'改为'清空队列取最新'，因为 OSD 只需要最新检测结果，不需要历史帧。"

### Bug 故事：OSD 叠加后画面变黑白

**现象**：开启 OSD 后 RTSP 推流画面整体变为灰度。

**原因**：`draw_rect_nv12()` 将 bbox 矩形内 UV 平面全部填为 128（中性色），person bbox 面积大（常覆盖画面 40-50%），大面积去色导致画面黑白。

**解决**：去掉 UV 平面填充循环，仅画 Y 平面 2px 白色边框。边框线宽极窄（2/1280=0.15% 画面宽），即使保留原 UV 值也视觉不受影响。

**面试话术**："NV12 的 UV 控制颜色、Y 控制亮度。我只画 Y=255 白边框不改 UV，画面颜色完全不受影响。UV 全填 128 是典型的过度绘制 —— 本来只想要个边框，结果把框内部全去了色。"
