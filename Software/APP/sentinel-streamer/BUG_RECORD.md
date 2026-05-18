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
