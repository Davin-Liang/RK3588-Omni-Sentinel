# SentinelQT — 学习指南

## 目标

面试时能说清：做了什么、为什么这么设计、踩过什么坑。

---

## 第一层：能说清"做了什么"（面试讲项目用）

### 一句话概括

> 基于 Qt5 Widgets 的 RK3588 嵌入式触屏 HMI，集成 sentinel-visioner (相机采集) 和 sentinel-streamer (推流/录像)，通过 QStackedWidget 双页布局提供预览监控、推流录像控制和视频文件管理。

### 架构图（能画出来）

```
RK3588 触屏
    │
    ▼
QApplication (eglfs, 全屏无边框)
    │
    └─ QStackedWidget
        ├── Page 0: 主控页面
        │   ├── 标题栏 (温度/CPU/RGA/NPU 监控 + 时钟)
        │   ├── 预览区 (1080p RGB888 实时画面)
        │   ├── 状态栏 (FPS + 录制状态)
        │   └── 控制行 (推流/录像/系统 三复用按钮)
        │
        └── Page 1: 视频管理页面
            └── QTableWidget (文件名/分辨率/时长/删除)

底层组件:
    ├── SentinelVisioner visioner_   (相机采集 + RGA 预处理)
    ├── SentinelStreamer streamer_   (RTSP推流 + MP4录像)
    └── PreviewWorker (子线程)       (try_get_preview → QImage 帧投递)
```

### 关键代码（背下来）

```cpp
// Widget 持有 visioner 和 streamer，构造即启动相机 + 预览
Widget::Widget() {
    visioner_ = new SentinelVisioner();
    streamer_ = new SentinelStreamer();
    visioner_->add_camera("/dev/video11", 1920, 1080, 8, 0);
    visioner_->camera_stream_ctrl(0, true);
    streamer_->add_camera(0, visioner_);
    streamer_->set_callback(streamer_callback_);
    start_preview_();  // 启动子线程拉帧
}

// 推流/录像复用式按钮：同一按钮根据状态启停
void on_btn_stream_() {
    if (streamer_->is_streaming(0))
        streamer_->stop_stream(0);
    else
        streamer_->start_stream(0, rtspUrl_.toUtf8().constData());
}
```

---

## 第二层：能解释"为什么这么设计"（面试追问用）

### 决策 1：为什么用 QStackedWidget 双页，不用多窗口？

| 方案 | 多窗口 / QDialog | QStackedWidget（我们用的） |
|------|-----------------|--------------------------|
| 触屏体验 | 弹窗遮挡，需手动关闭 | 页面切换，返回即恢复原状态 |
| 状态保持 | 弹窗关闭后需重建 | 两个页面共享同一套 visioner/streamer |
| 实现复杂度 | 管理多个窗口生命周期 | 一行 `setCurrentIndex(0/1)` |

**教训**: 嵌入式触屏上弹窗会让用户困惑，页面内切换更自然。

### 决策 2：预览为什么用独立子线程 + try_get_preview 超时轮询？

```
主线程 (UI 不阻塞)           PreviewWorker 子线程
    │                              │
    ├─ start_preview_()            ├─ while(running_) {
    │   ├─ new QThread             │    task = try_get_preview(cam, 200ms)
    │   ├─ moveToThread            │    QImage(virtAddr, 1920, 1080, RGB888)
    │   └─ thread->start()  ────→  │    emit frameReady(img.copy())
    │                              │    release_preview()
    │                              │  }
    ├─ on_frame_ready_(QImage) ←───┘
    │   └─ previewLabel->setPixmap()
```

- `wait_get_preview()` 内部用 `pop()` 无限阻塞，相机停产后线程永远卡死 → **必须用超时版**
- `try_get_preview(cam, 200)` 每次超时检查 `running_` 标志，200ms 内响应退出
- QImage 构造用 `virtAddr` 直接引用 DMA 内存，`.copy()` 深拷贝后立即 `release_preview()` 归还

### 决策 3：为什么系统暂停用 camera_pause 而不是 STREAMOFF？

```
STREAMOFF 方案（有问题）:
  camera_stream_ctrl(false) → V4L2 管线关闭 → 再 STREAMON → RK3588 ISP 无法恢复

camera_pause 方案（我们用的）:
  camera_pause(true) → capture 线程跳过 RGA，仅做 QBUF 循环 → 硬件流保持
  camera_pause(false) → 立即恢复 RGA 处理，帧马上可用
```

**教训**: RK3588 ISP 驱动的 MIPI 管线在 STREAMOFF 后需要完整重初始化，仅靠 STREAMON 不够。保持硬件流是最稳妥的做法。

### 决策 4：StreamerCallback 怎么安全跨到 Qt 主线程？

```cpp
// streamer 回调在内部线程调用，不能直接操作 UI
static void streamer_callback_(int camNum, StreamerEvent event, const char* detail) {
    Widget* w = Widget::instance();                              // ① 静态单例桥接
    QString s = detail ? QString::fromUtf8(detail) : QString();  // ② C串立即转QString
    QMetaObject::invokeMethod(w, [=]() {                         // ③ 排队到主线程
        w->on_streamer_event(camNum, event, s);
    }, Qt::QueuedConnection);
}
```

三个关键点对应三个常见 bug：
- ① 无对象指针 → 用静态单例
- ② 栈上临时字符串 → 回调返回前转 QString（所有权转移）
- ③ 直接调 UI 方法 → `QueuedConnection` 确保在主线程事件循环执行

---

## 第三层：能讲清 bug 和教训（面试加分项）

### 从 BUG_RECORD.md 选 3 个最有代表性的

**1. 预览线程关闭死锁**

- 现象：点"关闭系统"后程序卡死，`kill -9` 才能退出
- 原因：`PreviewWorker::start()` 用 `wait_get_preview()` 拉帧，内部 `pop()` 无限阻塞。相机停产后队列为空，线程永久卡在 `pop()` 内，`running_ = false` 永远检查不到
- 修复：SentinelVisioner 新增 `try_get_preview(cam, timeoutMs)`，PreviewWorker 改用 200ms 超时轮询
- **面试话术**: "条件变量的 `wait()` 没有超时就是定时炸弹。我们在 SentinelVisioner 里加了 `try_pop` 超时版接口，确保退出信号最多 200ms 内被响应"

**2. config.ini 相对路径找不到**

- 现象：改 `config.ini` 内容不生效，程序始终用默认值 `192.168.1.100`
- 原因：`QSettings("config.ini", IniFormat)` 用相对路径，查找位置取决于 `$PWD` 而非可执行文件目录。板端 `cd /tmp && /mnt/nfs/install/SentinelQT` 就找不到
- 修复：`QCoreApplication::applicationDirPath() + "/config.ini"` 始终从二进制同目录读
- **面试话术**: "Qt 的相对路径 QSettings 在当前工作目录下找文件，嵌入式环境的工作目录不可控。用 applicationDirPath 绑定二进制位置是最可靠的做法"

**3. 析构顺序错误导致 use-after-free**

- 现象：退出程序偶发 SIGSEGV
- 原因：析构时先 `delete visioner_`，但 PreviewWorker 子线程可能还在 `try_get_preview` 里访问它
- 修复：析构顺序改为 `camera_stream_ctrl(false)` → `stop_preview_()` → `join()` → `delete streamer_/visioner_`
- **面试话术**: "多线程下的析构顺序是个硬约束——必须先停帧源，再停消费者，最后释放资源。违反顺序 = use-after-free，偶发崩溃最难排查"

---

## 怎么对着代码学

**别死记硬背。跟一遍数据流：**

1. 打开 `widget.cpp`，从构造函数开始
2. 走一遍 `init_camera_()` → 理解相机和 streamer 怎么注册
3. 走一遍 `start_preview_()` → 理解 PreviewWorker 怎么创建和启动
4. 在 `preview_worker.cpp` 的 `start()` 跟一帧的完整生命周期
5. 看 `on_btn_stream_()` / `on_btn_system_()` → 理解启停流程
6. 回到 `widget.cpp` 的析构函数 → 理解逆序释放

**重点函数入口:**

| 函数 | 作用 |
|------|------|
| `Widget::Widget()` / `~Widget()` | 理解完整生命周期和析构顺序 |
| `start_preview_()` / `stop_preview_()` | 理解子线程创建/销毁 |
| `PreviewWorker::start()` | 理解一帧怎么从 DMA → QImage → 信号 |
| `on_btn_stream_()` / `on_btn_record_()` | 理解复用式按钮逻辑 |
| `on_btn_system_()` | 理解 camera_pause 暂停/恢复 |
| `scan_videos_()` | 理解 libavformat 读取视频元数据 |
| `update_hw_usage_()` | 理解 CPU/RGA/NPU/温度四个数据源 |
| `streamer_callback_()` | 理解跨线程回调安全投递 |
