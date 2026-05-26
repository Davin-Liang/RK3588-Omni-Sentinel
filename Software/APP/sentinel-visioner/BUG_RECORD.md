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

## 6. rga_scale_nv12_to_nv12_ 被 rga_convert_to_rgb_full_ 替代

**现象**: 预览画面为 720p 低分辨率 NV12 格式，色彩空间与 QT 渲染不兼容，显示异常。

**原因**: 早期预览管线使用 `rga_scale_nv12_to_nv12_` 将 1080p NV12 降级缩放到 720p NV12，走 RGA `imresize`。但 NV12 是 YUV 4:2:0 半平面格式，QT `QImage` 无法直接渲染，且 720p 分辨率损失了大量细节，预览质量差。

**解决**: 废弃 `rga_scale_nv12_to_nv12_`，新增 `rga_convert_to_rgb_full_(int srcFd, int srcWidth, int srcHeight, DmaBuffer_t* dstBuf)`，使用 RGA `improcess` 将 1080p NV12 一次转为 1080p RGB888（无缩放、无 letterbox），RGB888 可直接构造 `QImage(QImage::Format_RGB888)` 零开销渲染。更改后预览画面：全分辨率、全彩、与 QT 渲染管线完全兼容。
