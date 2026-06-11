# BUG_RECORD — sentinel-visioner 问题记录

## 1. NpuOSD 重命名为 NpuPreview，720p NV12 OSD → 1080p RGB888 预览

**现象**: 预览画面下游（SentinelQT / 推流模块）无法获取到彩色 1080p 预览帧，`NpuOSD` 结构体字段与实际用途不匹配。

**原因**: 早期设计仅提供 720p NV12 格式的 OSD 叠加图像，使用 `NpuOSD` 结构体承载。后续需求变更为"同时输出 NPU 推理小图 + 1080p RGB888 全彩预览"，但 API 和结构体仍沿用旧的 `NpuOSD` 命名，字段仅含一个 `DmaBuffer_t*`，无法承载双路输出。

**解决**:
- 将 `NpuOSD` 重命名为 `NpuPreview`，语义上明确包含 NPU 推理图和预览图两种用途
- 结构体扩为 `{DmaBuffer_t* npuImage; DmaBuffer_t* previewImage;}`，分别指向 640x640 RGB888 推理图和 1920x1080 RGB888 预览图
- 新增 `previewPool` 内存池（`CameraContext::previewPool`），分配 1080p RGB888 缓冲区
- 下游 API 同步更新：`wait_get_osd()` → `wait_get_preview()`，`release_osd()` → `release_preview()`

---

## 2. ThreadSafeQueue 缺 `#include <condition_variable>` 导致编译失败

**现象**: 编译时报错 `error: 'condition_variable' in namespace 'std' does not name a type`，ThreadSafeQueue 模板类编译不通过。

**原因**: `ThreadSafeQueue<T>` 内部使用 `std::condition_variable cond_` 成员 + `cond_.wait()` / `cond_.wait_for()` / `cond_.notify_one()` 调用，但头文件未 `#include <condition_variable>`，仅包含了 `<chrono>`、`<mutex>`、`<queue>`。

**解决**: 在 `ThreadSafeQueue.h` 顶部补上 `#include <condition_variable>`，与 `<chrono>` `<mutex>` `<queue>` 并列。

---

## 3. RK3588 ISP 驱动 STREAMOFF 后无法通过 STREAMON 恢复

**现象**: 调用 `camera_stream_ctrl(camNum, false)` 后再调用 `camera_stream_ctrl(camNum, true)`，`VIDIOC_STREAMON` 返回成功但 epoll 再无事件，相机彻底死寂，只能重启进程恢复。

**原因**: RK3588 ISP 硬件管线在 `VIDIOC_STREAMOFF` 后会复位内部状态（包括 MIPI D-PHY、ISP pipeline 等），简单地再次 `STREAMON` 无法让 ISP 重新正常工作。这是驱动层面的问题，应用层无可靠手段在 STREAMOFF 之后干净重建 ISP 管线。

**解决**: 引入 `camera_pause(camNum, paused)` 接口。暂停时 V4L2 硬件流保持 `STREAMON`，捕获线程仍在 `epoll_wait` + `DQBUF` 循环中运行，只是跳过所有 RGA 处理和队列推送（`isPaused` 原子标记控制），直接将 buffer 通过 `QBUF` 归还驱动。恢复时取消标记，RGA 处理立即继续。全程不触碰 `STREAMOFF` / `STREAMON`。

---

## 4. camera_stream_ctrl 再次 STREAMON 前未重新 QBUF

**现象**: `camera_stream_ctrl(false)` 停流后，再次 `camera_stream_ctrl(true)` 启动流，`epoll` 无事件上报，摄像头不产帧。

**原因**: `VIDIOC_STREAMOFF` 执行时，V4L2 框架会将所有已 `DQBUF` 取出的缓冲区从就绪队列移除。再次 `STREAMON` 之前，需要将这些缓冲区通过 `VIDIOC_QBUF` 重新入队，否则内核无可用缓冲区填入新帧。旧版代码未做重新 QBUF 这一步，导致 STREAMON 后硬件虽然在运行但无处存放数据。

**解决**: 在 `camera_stream_ctrl` 的 `STREAMON` 逻辑中，如果之前执行过 `STREAMOFF`，应遍历 `ctx->buffers` 为每个 buffer 调用一次 `VIDIOC_QBUF` 重新压回内核队列。注意：当前推荐方案使用 `camera_pause` 避免 STREAMOFF/STREAMON 循环（见 #3），此修复作为保底手段以防必须执行 STREAMOFF 的场景。

---

## 5. previewTaskQueue 无限阻塞 pop() 导致消费者线程死锁

**现象**: 预览线程（SentinelQT 的 `PreviewWorker`）在相机停止产帧后永久挂起，`running_` 标志设为 `false` 无效，程序无法正常关闭。

**原因**: `wait_get_preview(camNum)` 内部调用 `ThreadSafeQueue::pop()`，在队列为空时通过 `std::condition_variable::wait` 无限阻塞。当捕获线程因 `camera_stream_ctrl(false)` 或 `camera_pause` 停止向 `previewTaskQueue` 推帧后，消费者线程卡死在 `pop()` 中，永远无法返回检查退出标志。

**解决**: 新增 `try_get_preview(camNum, timeoutMs)` 方法，内部调用 `ThreadSafeQueue::try_pop(val, timeoutMs)`，使用 `wait_for` 替代 `wait`，每次超时后返回 `{nullptr, nullptr}`。消费者线程在每次超时后检查退出标志，确保最迟在 `timeoutMs` 内响应退出信号。`wait_get_preview` 保留用于不需要中断场景的高效阻塞等待。

---

## 7. USB 相机 NV12 预览/推流画面横向花屏（未解决）

**现象**: USB 相机（/dev/video21，1280x720，NV12 原生输出）预览和推流画面出现横向花屏（水平条纹状撕裂），相机静止时较轻微，运动时明显加剧。ISP 相机（MIPI CSI）完全正常。USB 单独运行时问题仍在，排除两路 RGA 竞争。

**原因**: 未确定根因。以下是排查过程：

1. **异步 RGA 导致 QBUF 后缓冲区被覆盖** — 排除：`improcess` 传入 `usage=0` 即异步模式，RGA 未完成就归还缓冲区给驱动。修复：所有 `improcess(..., 0)` → `improcess(..., IM_SYNC)`。无效。

2. **DMA 缓存一致性** — 排除：USB 相机的 V4L2 MMAP 缓冲区导出为 DMA-BUF 后被 RGA 读取，可能存在 CPU 缓存残留。修复：`DQBUF` 后调用 `DMA_BUF_IOCTL_SYNC`（`sync_dma_buf_for_device`）刷新缓存。无效（V4L2 缓冲区可能是 coherent DMA，此 ioctl 为 no-op）。

3. **NV12 bytesperline stride 不匹配** — 排除：USB 相机 V4L2 缓冲区可能有对齐过的 `bytesperline > width`，RGA 用 `width` 作为行跨度导致读取偏移。修复：`add_camera` 中从 `VIDIOC_G_FMT` 读取 `bytesperline`，三个 RGA 函数增加 `srcStride` 参数并用于 `importbuffer_fd` + `wrapbuffer_handle`。诊断日志显示 `bytesperline == width`（1280），无 padding。无效。

4. **USB DMA 缓冲区类型与 RGA 硬件不兼容** — 推测：USB 控制器的 DMA 缓冲区可能使用不同 IOMMU 域或内存类型（vmalloc/DMA-sg），RGA 硬件直接读取产生数据错位。

**当前缓解措施（workaround）**:

为 USB NV12 相机新增 `usbSafePool`（4 个 DMA buffer），捕获线程流程改为：
```
DQBUF → sync_dma → imcopy(相机BUF → usbSafePool) → QBUF(立即归还相机)
→ improcess(usbSafePool → NPU) → improcess(usbSafePool → 预览)
→ imcopy(usbSafePool → origCopyPool) → 释放 usbSafePool
```

所有 RGA 格式转换操作从 `usbSafePool`（`dma_heap` 分配的 uncached DMA 缓冲区，与 ISP 相机同类型）读取，相机 DMA 缓冲区仅被访问一次（`imcopy`）。此措施**未解决**花屏，仅将 RGA 对 USB 缓冲区的直接读取次数从 3 次减少到 1 次，留待后续排查。

**关键文件**:
- `sentinel-visioner/include/sentinel-visioner.h`: `CameraContext` 新增 `usbSafePool`、`srcBytesPerLine` 字段
- `sentinel-visioner/src/sentinel-visioner.cpp`: `add_camera` 分配 `usbSafePool`；`capture_thread_func_` 安全拷贝逻辑；三个 RGA 函数新增 `srcStride` 参数
- `sentinel-streamer/src/rga_scaler.cpp`: 函数改为 `rga_scale_nv12_to_720p(srcFd, srcWidth, srcHeight, dstFd)` 支持动态分辨率
- `sentinel-streamer/src/sentinel_streamer.cpp`: 720p 源录像直接用 `origBuf` 编码（绕过 RGA 缩放）

**回退指南**:
若要回退到无 workaround 的干净版本：
1. 移除 `CameraContext::usbSafePool` 及其分配/释放代码
2. 移除 `capture_thread_func_` 中 USB safe copy 逻辑（`safeBuf`/`safeBufToRelease`）
3. 恢复 `rga_process_to_rgb_`/`rga_convert_to_rgb_full_`/`rga_copy_buffer_` 的 `srcStride` 参数为直接使用 `srcWidth`/`width`
4. 恢复 `rga_scaler.cpp` 函数签名为 `rga_scale_nv12_1080p_to_720p(srcFd, dstFd)`
5. 移除 `DMA_BUF_IOCTL_SYNC` 及 `sync_dma_buf_for_device`
6. `improcess` 的 `IM_SYNC` 改为 `0` 可保留（同步模式无害）

**待排查方向**:
- `cat /sys/kernel/debug/dma_buf/<fd>/bufinfo` 查看 USB 相机 DMA-BUF 的实际内存类型
- 对比 ISP 和 USB 相机的 DMA-BUF exporter（`/sys/kernel/debug/dri/0/` 相关节点）
- 尝试 USB 相机强制使用 YUYV 格式，走 `rga_yuyv_to_nv12_` 转换路径
- 检查 RK3588 RGA 硬件勘误表是否有 USB 相关限制

---

## 6. rga_scale_nv12_to_nv12_ 被 rga_convert_to_rgb_full_ 替代

**现象**: 预览画面为 720p 低分辨率 NV12 格式，色彩空间与 QT 渲染不兼容，显示异常。

**原因**: 早期预览管线使用 `rga_scale_nv12_to_nv12_` 将 1080p NV12 降级缩放到 720p NV12，走 RGA `imresize`。但 NV12 是 YUV 4:2:0 半平面格式，QT `QImage` 无法直接渲染，且 720p 分辨率损失了大量细节，预览质量差。

**解决**: 废弃 `rga_scale_nv12_to_nv12_`，新增 `rga_convert_to_rgb_full_(int srcFd, int srcWidth, int srcHeight, DmaBuffer_t* dstBuf)`，使用 RGA `improcess` 将 1080p NV12 一次转为 1080p RGB888（无缩放、无 letterbox），RGB888 可直接构造 `QImage(QImage::Format_RGB888)` 零开销渲染。更改后预览画面：全分辨率、全彩、与 QT 渲染管线完全兼容。

---

## 8. V4L2 MPLANE 模式未设 planes 数组导致静默失败

**现象**: `VIDIOC_QBUF` / `VIDIOC_DQBUF` 返回 `EINVAL`，`strerror` 仅显示 "Invalid argument"，无帧产出，应用端感知不到明显错误，但管道始终无数据。

**原因**: MPLANE 模式下，内核要求 `struct v4l2_buffer` 的 `m.planes` 必须指向用户空间的有效 `v4l2_plane` 数组。若代码未显式分配和赋值，`m.planes` 就是栈或堆上的未初始化值（野指针）。它大概率非 NULL，内核用该非法地址进行 `copy_from_user` 时会触发段错误（`SIGSEGV`）或返回 `-EFAULT`；部分校验路径也可能因 `length` 等字段不合法而直接返回 `-EINVAL`。无论哪种现象，根因都是没有给内核提供合法的 planes 内存。

另外，RK3588 ISP 驱动只支持 MPLANE，且铁律是：`VIDIOC_REQBUFS` 用的 `type`（如 `V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE`），后续所有 buf 操作的 `type` 都必须严格一致，否则同样会得到 `EINVAL`。

**解决**: 每次 QBUF/DQBUF 前，显式声明 `struct v4l2_plane planes[1] = {};`，然后设置 `buf.m.planes = planes; buf.length = 1;`（内核需要知道 plane 数量）。更稳妥的做法是先用 `memset(&buf, 0, sizeof(buf))` 清零整个 `v4l2_buffer`，再赋值，避免任何遗留的垃圾值。

```c
// MPLANE 模式 QBUF/DQBUF 的正确姿势
struct v4l2_plane planes[1] = {};
memset(&buf, 0, sizeof(buf));
buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
buf.memory = V4L2_MEMORY_MMAP;
buf.index = i;
buf.m.planes = planes;
buf.length = 1;
```

---

## 9. NPU 和预览共用 buffer 门控，NPU 池空导致预览停摆

**现象**: 预览画面启动后约 8 帧卡住，日志刷 `[Thread] Warning: RGA buffer pool empty! Dropping frame.`，PreviewWorker 报 `no frame for N cycles`。

**原因**: NPU 缓冲区和预览缓冲区的获取放在同一个 `if (targetNpuBuf != nullptr)` 门控内。`npuRgbPool` 只有 8 个 buffer，无人消费 `npuTaskQueue`，8 帧后 NPU 池枯竭 → `get_buffer()` 返回 nullptr → 跳过整个 NPU+预览处理块 → 预览帧停止产出。

**解决**: 将 NPU 和预览处理拆为独立 `if` 块，各自 buffer 池互不阻塞。NPU buffer 无人消费时直接 `release_buffer` 回池子（预留 `TODO: NPU 推理接入后改为 npuTaskQueue.push`）。

---

## 10. USB 相机不支持 NV12，驱动接受 NV12 请求但实际选 MJPG → usbSafePool 空指针崩溃

**现象**: 更换 USB 相机后程序启动即 segfault，dmesg 无 kernel 报错。G_FMT 读回 `actualPixelFormat = MJPG`。

**原因**: `VIDIOC_S_FMT` 请求 NV12 时驱动未拒绝，但实际选了默认格式 MJPG。`usbSafePool` 仅在 `actualPixelFormat == NV12` 时分配，MJPG 路径进入 `camType == USB_CAM && convBufToRelease == nullptr` 分支后访问未分配的 `usbSafePool` → 空指针 segfault。

**解决**: 新增 MJPG 格式检测，分配 `mjpegDecodePool`（FFmpeg 软件解码 NV12 输出池）。捕获线程新增 MJPG→NV12 解码分支（FFmpeg avcodec），解码后送入统一下游管线。

---

## 11. USB 相机声明 YUYV 但实际输出 YVYU，U/V 通道互换致肤色偏紫

**现象**: USB 相机预览画面人脸肤色偏紫偏绿，类似中毒。ISP 相机正常。

**原因**: RGA `rga_yuyv_to_nv12_()` 使用 `RK_FORMAT_YUYV_422`（Y0:U0:Y1:V0），但相机实际输出 YVYU（Y0:V0:Y1:U0）。U/V 通道互换后 RGA NV12→RGB 转换产生错误的红蓝色度，肤色变为紫色。

**解决**: 将 RGA 源格式改为 `RK_FORMAT_YVYU_422`。

---

## 12. FFmpeg MJPG 解码器输出 YUVJ422P，UV 子采样处理错误

**现象**: MJPG 1080p 画面显示正常但肤色再次偏色。

**原因**: FFmpeg MJPEG 解码器输出 `yuvj422p`（YUJV 4:2:2 平面格式），U/V 平面高度与 Y 平面相同（非 4:2:0 的一半）。原有 YUV420P→NV12 打包代码按半高度采样 UV，丢失一半色度数据。

**解决**: 检测实际像素格式，422 时 `uvRowStep = 2`（垂直跳行子采样），420 时 `uvRowStep = 1`（直通）。
