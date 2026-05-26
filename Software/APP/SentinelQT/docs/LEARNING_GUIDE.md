# SentinelQT 学习指南

记录开发过程中的关键设计决策、踩坑经验和知识沉淀。

## 第一层：基础决策

### 决策 1: QWidget vs QMainWindow

选择 `QWidget`，不是 `QMainWindow`。

- RK3588 + 触摸屏 = 嵌入式 HMI，要全屏无边框。QMainWindow 自带的菜单栏/工具栏/状态栏/停靠窗口全是无用负担
- `setWindowFlags(Qt::FramelessWindowHint)` + `showFullScreen()` 直接全屏，绕过窗口管理器
- 更轻量，布局完全自由

### 决策 2: eglfs vs wayland

使用 `-platform eglfs`，不走 Wayland。

- RK3588 的 Mali GPU 通过 eglfs 直接渲染到 DRM/KMS，零中间层
- Buildroot 默认的 Weston 会独占 DRM master，先 `killall -9 weston` 再启 Qt
- eglfs 独占 DRM 设备的限制：ffplay 等 SDL2 应用无法同时运行（DRM master 冲突），应用内播放需要用 Qt Multimedia 或 MPP 解码

### 决策 3: QStackedWidget 双页布局

主页面和视频管理页用 `QStackedWidget` 切换，不用多窗口或弹出对话框。

- 1024×600 触屏，弹出窗口影响操作
- 两个页面共享同一套底层组件（visioner/streamer），无需跨窗口传引用
- 视频管理页返回时相机预览等状态完全保留

## 第二层：线程与异步

### 决策 4: QThread + moveToThread vs std::thread

PreviewWorker 使用 Qt 线程模型：`QThread + moveToThread + 信号槽`。

- 信号槽天然支持跨线程安全投递（`Qt::QueuedConnection` 自动序列化 QImage）
- `QThread::quit()/wait()` 提供优雅退出机制
- 相比 std::thread，Qt 线程与事件循环集成更好，调试更直观

### 决策 5: StreamerCallback 跨线程桥接

SentinelStreamer 的回调是 C 风格函数指针，运行在 streamer 内部线程。

```cpp
static void streamer_callback_(int camNum, StreamerEvent event, const char* detail) {
    Widget* w = Widget::instance();
    QString detailStr = detail ? QString::fromUtf8(detail) : QString();
    QMetaObject::invokeMethod(w, [=]() {
        w->on_streamer_event(camNum, event, detailStr);
    }, Qt::QueuedConnection);
}
```

关键设计点：
- **C 字符串立即转 QString**：`detail` 是栈上指针，回调返回后失效，必须在 `invokeMethod` 前复制
- **static 单例桥接**：C 回调无 this 指针，通过 `Widget::instance()` 找到对象
- **QueuedConnection 强制排队**：确保 lambda 在主线程事件循环中执行，避免 streamer 线程直接操作 UI

### 决策 6: try_get_preview 超时轮询 vs wait_get_preview 无限阻塞

预览线程使用 `try_get_preview(camNum, 200)` 而非 `wait_get_preview()`。

- `wait_get_preview` 内部用 `ThreadSafeQueue::pop()` 无限阻塞，产帧停止后线程永远挂起
- `try_get_preview` 带 200ms 超时，每次超时检查 `std::atomic<bool> running_`，确保 200ms 内响应退出
- 代价：CPU 在无帧时每 200ms 唤醒一次（可忽略不计）

### 决策 7: 析构顺序

组件析构必须严格逆序，否则 use-after-free：

```
1. camera_stream_ctrl(false)    ← 最先停相机，停止所有帧源
2. stop_preview_()              ← 再停消费者线程（队列不再有新帧）
3. streamer_->remove_camera(0)  ← 清理推流/录像线程
4. delete streamer_ / visioner_ ← 最后释放硬件抽象层
```

违反此顺序会导致 worker 线程访问已释放的 visioner_ → SIGSEGV。

## 第三层：RK3588 平台特有问题

### 决策 8: camera_pause vs STREAMOFF

系统暂停/恢复使用 `camera_pause()` 而非 `camera_stream_ctrl(false/true)`。

- RK3588 ISP 驱动在 STREAMOFF 后仅靠 STREAMON 无法恢复（MIPI 管线需要完整重初始化）
- `camera_pause` 保持 V4L2 流运行，capture 线程仅跳过 RGA 处理和队列推送，buffer 仍在驱动和用户态间循环
- 代价："暂停"期间 DMA buffer 仍在占用（约 8 个 1080p NV12 buffer ≈ 24MB），但对于 8GB RK3588 可忽略

### 决策 9: DebugFS 权限

RGA/NPU 利用率读取需要 root 权限。

- `/sys/kernel/debug/rkrga/load` 和 `/sys/kernel/debug/rknpu/load` 仅 root 可读
- 非 root 运行时显示 `--%`（降级处理，不报错）
- 生产环境可配置 sudo 或调整 debugfs 挂载权限

## 第四层：UI 设计经验

### 决策 10: 复用式按钮 vs 分离式按钮

推流和录像用单个复用按钮，不用启动/停止分离。

- 触屏空间有限（1024×600），每个像素都要精打细算
- 状态互斥（同时只能推或不推），分离按钮必然有一个始终禁用，浪费空间
- `update_button_states_()` 动态切换文字和颜色，视觉反馈清晰

### 决策 11: 标题栏左右对称居中

标题居中需要左右两侧控件等宽：

- `hwLabel` 和 `clockLabel` 设为相同 fixed width（260px）
- 中间 `titleLabel` 两侧 stretch=1 的 spacer 均分剩余空间
- 两侧不等宽时标题视觉偏移，用户会注意到不对称
