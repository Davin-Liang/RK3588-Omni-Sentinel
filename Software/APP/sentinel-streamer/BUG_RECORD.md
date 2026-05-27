# BUG_RECORD — sentinel-streamer 问题记录

## 1. DRM_PRIME 零拷贝不被 h264_rkmpp 支持

**现象**: `avcodec_open2` 报错 `Unsupported input pixel format '(null)'`, `Function not implemented`

**原因**: FFmpeg 的 `h264_rkmpp` 编码器不支持 `AV_PIX_FMT_DRM_PRIME` 零拷贝 DMA-BUF 输入。

**解决**: 改用 `AV_PIX_FMT_NV12` + `memcpy` 通过 CPU 虚拟地址传入帧数据。

---

## 2. 画面严重偏绿

**现象**: 推流/录像画面整体严重偏绿。

**原因**: **同时启用两个 MIPI-CSI 摄像头导致信号干扰**，CSI 控制器共享时像素数据错位，U/V 通道偏移。NV12/NV21 格式混淆和 ISP 3A 未启动曾加剧问题，但不是根因。禁用其中一个摄像头后画面恢复正常。

**解决**: 仅启用单路摄像头。硬件上若需双路，确保两路 sensor 的 CSI lane 和数据率匹配。

---

## 3. 画面偏暗

**现象**: 推流画面整体偏暗。

**原因**: 
1. 摄像头模拟增益 (`analogue_gain`) 默认 128（1x）
2. 编码器未设色彩范围，播放器按 TV 范围 (16-235) 解释全范围数据

**解决**: 
- 通过 `v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl=analogue_gain=512` 提增益
- 编码器/帧设置 `AVCOL_RANGE_JPEG`、`AVCOL_PRI_BT709`、`AVCOL_TRC_BT709`、`AVCOL_SPC_BT709`

---

## 4. 录像黑屏 + 时长异常（37 分钟）

**现象**: 录制的 MP4 文件打开黑屏，显示时长 37 分钟，实际录制仅 30 秒。

**原因**: `DmaBuffer_t::timestampUs` 来自 V4L2 `CLOCK_MONOTONIC`，值为系统启动后的微秒累计值（如 5000 秒 = 5,000,000,000μs）。直接用作 PTS 导致首帧 PTS = 5000 秒，播放器前 5000 秒黑屏。

**解决**: 用帧计数器代替真实时间戳作为 PTS，每帧 +1。

---

## 5. 录像时长显示 0 秒

**现象**: 帧数据可播放可拖动，但播放器显示时长为 0。

**原因**: 对 `{1,90000}` 时基强行设 `pkt->duration = 1`，等价于每帧 1/90000 秒，总时长近乎为 0。

**解决**: 去掉手动 `pkt->duration`，让 FFmpeg 根据帧间 PTS 差值自动计算。

---

## 6. RTSP + MP4 同时输出时录像打不开

**现象**: 推流正常，录像文件无法播放。

**原因**: `av_interleaved_write_frame` 内部会缩放 `pkt->pts/dts` 适配输出时基，同一个包先写给 RTSP 时基被修改，再写给 MP4 时基错误。

**解决**: 
- 推流和录像改为各自独立编码器（`streamEncCtx` / `recordEncCtx`）
- 编码包不再共享

---

## 7. 反复启停时 DTS 不单调

**现象**: 第二轮循环出现 `Application provided invalid, non monotonically increasing dts to muxer in stream 0: 702000 >= 0`

**原因**: 第一轮停止时编码器未销毁，内部残留帧在第二轮吐出，DTS 为上一轮的 702000 远超新帧的 0。

**解决**: `stop_stream` / `stop_record` 中调用 `mpp_encoder_close` 销毁编码器，下一轮 `start` 时重建全新编码器。

---

## 8. 编码器 use-after-free 导致 SIGSEGV

**现象**: 约 50% 概率在 `stop_stream` 时 segfault，调用栈落在 `av_interleaved_write_frame` 内部。

**原因**: `stop_record` 调用 `mpp_encoder_close(&ctx->recordEncCtx)` 时，推流线程还在运行且可能正在使用 `recordEncCtx`，造成 use-after-free。

**解决**: 
- `stop_record` / `stop_stream` 不再销毁对方的编码器
- 编码器销毁统一推迟到 `ctx->workerThread.join()` 之后

---

## 9. av_interleaved_write_frame → av_log 偶发崩溃

**现象**: 修复 use-after-free 后仍偶发 segfault，GDB 显示 `av_vlog` 内空指针解引用（`si_addr=(nil)` 或 `si_addr=0x18`）。

**原因**: 反复启停时 FFmpeg 的 RTSP API muxer 内部状态不稳定，出错时调用 `av_log` 打印日志，但日志回调/上下文指针已损坏，导致二次崩溃。

**解决**: 放弃 FFmpeg C API 的 RTSP 输出，改用 `popen("ffmpeg ...")` 子进程通过管道推流。H.264 裸流直接 `fwrite` 进管道，ffmpeg 命令行负责 RTSP 推流。子进程崩溃不影响主程序。

---

## 12. 编码器惰性创建（按需分配）

**现象**: `add_camera` 总是初始化两路编码器，但实际可能只推流不录像或只录像不推流，造成编码器资源浪费。

**解决**: `add_camera` 只建缩放缓冲池，编码器延迟到 `start_stream` / `start_record` 首次调用时创建。`stop` 时销毁，重新 `start` 时再建。资源按需分配，不用的编码器不占资源。

---

## 11. stop_record 提前关闭 mp4Ctx 导致 use-after-free

**现象**: 循环测试中偶发 `av_write_frame` 内 SIGSEGV（`si_addr=0x18`），调用栈落在 `encode_and_mux` → `av_write_frame`，发生在线程还未退出时。

**原因**: `stop_record` 在 `ctx->recordEnabled = false` 后立即调用 `mp4_output_close(&ctx->mp4Ctx)`，但推流线程可能正处于 `av_write_frame(mp4Ctx, pkt)` 内部（已通过 `recordEnabled` 检查，尚未返回）。`mp4Ctx` 被释放后线程继续访问 → use-after-free。

**解决**:
- `stop_record` 仅设置 `recordEnabled = false`，延时关闭 `mp4Ctx`
- `stop_stream` 中 `join()` 线程退出后，再检查并关闭被延时的 `mp4Ctx`
- 原则：**任何输出上下文/编码器，只有在线程 `join()` 之后才允许销毁**

---

## 10. FFmpeg 交叉编译配置问题

**现象**: `./configure` 多次报错（编译器不可用、libdrm 找不到等）。

**原因**: 
- 交叉编译器不在 `PATH`
- `PKG_CONFIG_LIBDIR` 未指向 sysroot
- `PKG_CONFIG_SYSROOT_DIR` 未设，`.pc` 文件的 `prefix=/usr` 无法解析
- FFmpeg 寻找 `aarch64-buildroot-linux-gnu-pkg-config` 而非系统 `pkg-config`
- Perl 模块缺失导致文档编译失败

**解决**: 
```bash
export PATH=<sdk>/bin:$PATH
export PKG_CONFIG_LIBDIR=<sysroot>/usr/lib/pkgconfig
export PKG_CONFIG_SYSROOT_DIR=<sysroot>
export PKG_CONFIG=pkg-config
ln -s $(which pkg-config) <sdk>/bin/aarch64-buildroot-linux-gnu-pkg-config
./configure ... --disable-doc
```

---

## 13. MPP 编码器内部 PTS 覆盖导致录像时长异常

**现象**: 录像时长与实际录制时长严重不符（如录 7s 显示 24s，录 5s 显示 11s）。

**原因**: `encode_and_mux` 中原本只在 `pkt->pts == AV_NOPTS_VALUE` 时才用 `sentPts` 覆盖。MPP 硬件编码器内部会重新打时间戳（非空值），导致写入 MP4 的 PTS 为编码器内部时钟值而非我们计算的正确值。

**解决**: 始终强制覆盖 PTS/DTS：`pkt->pts = sentPts; pkt->dts = sentPts;`，不再依赖 `AV_NOPTS_VALUE` 判断。

---

## 14. 录制开始前队列积压旧帧导致 PTS 基准偏移

**现象**: 录制前相机已在运行（预览中），点击录制后视频前几秒时间戳跳变，总时长偏长。

**原因**: `processTaskQueue` 在录制开始前已积压旧帧（捕获线程持续推送）。录制线程启动时，队列中第一个帧的时间戳是几秒前的旧值，`firstTsUs` 基于此旧值归零 PTS，导致所有帧 PTS 整体偏移。

**解决**: PTS 基准改为录制按钮按下时的 `clock_gettime(CLOCK_MONOTONIC)` 系统时间（`baselineTsUs`），不再使用首帧时间戳。同时跳过队列中 `tsUs < baselineTsUs` 的积压旧帧。

---

## 15. 队列积压旧帧未丢弃

**现象**: 同 #14，录制开始瞬间队列中存在录制前捕获的旧帧。

**原因**: 录制线程启动后直接从 `processTaskQueue` 拉帧，不区分新旧。

**解决**: 在 `stream_thread_func_` 中检查 `tsUs < baselineTsUs`，积压旧帧直接 `release_orig_copy_buffer` 归还 DMA 缓冲池并跳过。

---

## 17. RGA 缩放器硬编码 1080p 源导致 720p 相机推流失败

**现象**: USB 720p 相机（CAM1）启动推流时报 `[RgaScaler] importbuffer_fd src failed`，推流线程立即退出。

**原因**: `rga_scale_nv12_1080p_to_720p()` 硬编码源分辨率为 `{1920, 1080}`，`importbuffer_fd` 向 RGA 驱动声明源 DMA-BUF 为 1080p NV12，但 USB 相机实际 DMA-BUF 为 720p，驱动校验尺寸不匹配，导入失败。

**解决**: 函数改为 `rga_scale_nv12_to_720p(int srcFd, int srcWidth, int srcHeight, int dstFd)`，`importbuffer_fd` 和 `wrapbuffer_handle` 使用实际源分辨率。源已是 720p 时用 `imcopy` 替代 `improcess`。

---

## 18. 720p 源录像产生空 MP4 文件

**现象**: USB 相机（CAM1，720p）录制完成后 MP4 文件无法播放，`avformat` 读不到分辨率和时长。

**原因**: 720p 录像路径通过 `scaleBuf`（RGA 缩放结果）编码。USB 相机源已是 720p，RGA identity copy 可能失败导致 `scaleBuf` 为 null，编码步骤被静默跳过，MP4 只有头尾无帧数据。

**解决**: 720p 录像时检测源分辨率：源已是 720p 则直接用 `origBuf` 编码（与 1080p 路径一致），绕过 `scaleBuf`。1080p 源录 720p 仍走 RGA 缩放路径不变。

---

## 16. 移除 MPP 编码器 framerate 导致画面马赛克

**现象**: 去掉 `ctx->framerate` 设置后，画面出现严重马赛克/块效应，仅第一帧（I帧）清晰。

**原因**: MPP 硬件编码器依赖 `framerate` 参数进行码率控制。不设 `framerate` 时编码器码控异常，分配的码率远低于目标值，导致 P/B 帧质量急剧下降。

**解决**: 恢复 `ctx->framerate = AVRational{15, 1}`（与相机实际帧率匹配），framerate 仅用于码率控制，PTS 由我们独立覆盖，互不干扰。
