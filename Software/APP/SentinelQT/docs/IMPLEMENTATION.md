# SentinelQT 实现文档

## 1. 架构总览

```
SentinelQT 进程
├── 主线程 (Qt Event Loop)
│   ├── Widget (QStackedWidget 四页布局)
│   │   ├── Page 0: 主控页面 (预览 + 控制 + OSD + 监控)
│   │   ├── Page 1: 视频管理页面 (QTableWidget + libavformat)
│   │   └── Page 2: 融合管理页面 (俯视图 + 参数面板 + 虚拟键盘)
│   ├── SentinelVisioner visioner_         // 相机采集管线
│   ├── SentinelYoloInfer yoloInfer_        // NPU 推理（懒加载）
│   ├── SentinelStreamer streamer_         // 推流/录像 + OSD
│   ├── SentinelLslidarer lidar_           // 激光雷达驱动
│   ├── LidarCameraFusion fusion_          // 视觉-雷达融合引擎
│   ├── ThermalController thermalCtrl_     // 温控调频引擎
│   ├── TopDownView topDownView_           // 鸟瞰俯视图组件
│   ├── VirtualKeyboard virtualKeyboard_   // 触屏数字键盘
│   └── QTimer × 3 (clock + record + fusionStatus)
│
├── PreviewWorker × 2 子线程
│   └── 循环: try_get_preview(200ms) → QImage → emit frameReady() → release_preview()
│
├── SentinelYoloInfer × 2 推理子线程 (每路相机独立)
│   └── try_get_npu → RKNN infer → push(fusionQueue + osdQueue) → release_npu
│
├── FusionWorker 子线程
│   └── 100ms 轮询 copy_tracked_targets() → emit trackingUpdated()（无条件推送）
│
├── SentinelLslidarer 内部线程 (reader)
│   └── SerialPort::read_packet() → RingBuffer (10Hz)
│
├── LidarCameraFusion 内部线程 (fusion)
│   └── DetectionProvider 回调获取 YOLO → get_closest_frame → fuse_data → update_tracking
│
└── SentinelStreamer 内部线程
    └── 消费 processTaskQueue → RGA缩放 → OSD叠加(StreamOsdProvider) → MPP编码 → RTSP/MP4
```

## 2. 线程模型

### 2.1 PreviewWorker (子线程预览拉帧)

- **创建**: `PreviewWorker(visioner_, camNum)` 构造于主线程，`moveToThread(previewThread_)` 移至子线程
- **启动**: `QThread::started` 信号 → `PreviewWorker::start()` 槽（DirectConnection，在子线程执行）
- **拉帧循环**: `try_get_preview(camNum, 200)` 带 200ms 超时轮询，避免 `wait_get_preview` 的无限阻塞 `pop()`
- **帧投递**: `emit frameReady(QImage)` → AutoConnection 自动转为 QueuedConnection（跨线程），主线程 `on_frame_ready_` 处理
- **退出**: `stop()` 设置 `std::atomic<bool> running_ = false` → 下次超时检查退出 → `QThread::quit()/wait()`

### 2.2 StreamerCallback 跨线程通知

`SentinelStreamer` 的回调在 streamer 内部线程调用，需安全跨越到 Qt 主线程：

```cpp
static void streamer_callback_(int camNum, StreamerEvent event, const char* detail) {
    Widget* w = Widget::instance();
    QString detailStr = detail ? QString::fromUtf8(detail) : QString();
    QMetaObject::invokeMethod(w, [=]() {
        w->on_streamer_event(camNum, event, detailStr);
    }, Qt::QueuedConnection);
}
```

关键点：
- C 字符串 `detail` 在回调返回前转为 `QString`（所有权转移）
- `Qt::QueuedConnection` 确保 lambda 在主线程事件循环中执行
- `Widget::instance()` 静态单例指针桥接 C 风格回调与 C++ 对象

### 2.3 析构顺序

```
~Widget():
  1. visioner_->camera_stream_ctrl(0, false)  // 先停相机捕获线程
  2. stop_preview_()                            // 再停预览线程
  3. streamer_->remove_camera(0)               // 清理推流/录像
  4. delete streamer_ / visioner_ / ui          // 最后释放资源
```

### 2.4 FusionWorker (子线程跟踪轮询)

- **创建**: `FusionWorker(fusion_)` 构造于主线程，`moveToThread(fusionThread_)` 移至子线程
- **启动**: `QThread::started` → `FusionWorker::start()` 槽（DirectConnection）
- **轮询循环**: 100ms 间隔调用 `fusion_->copy_tracked_targets()`，无条件 emit `trackingUpdated(QVector<TrackedTarget>)`（已去掉变化去重逻辑，确保 Web 俯视图刷新流畅不冻结）
- **退出**: `stop()` 设置 `std::atomic<bool> running_ = false` → 最多 100ms 内响应

### 2.5 LidarCameraFusion 内部线程

`LidarCameraFusion::start()` 创建独立 `std::thread` 运行 `fusion_thread_()`：
1. 通过 `DetectionProvider` 回调获取 YOLO 检测结果（`try_get_fusion_result`），无 provider 时回退假检测
2. 类别过滤（classId=0 person）和置信度过滤（≥0.75）
3. 取最近雷达帧（`get_closest_frame`）
4. 累积融合（`reset` → `fuse_data` × camCount）
5. 目标跟踪（`update_tracking`，7 步流水线）
6. 告警回调（`fusion_warning_callback_`，终端 stderr 输出）

### 2.6 SentinelYoloInfer 推理线程

`SentinelYoloInfer` 每路相机独立 `std::thread` 运行 `infer_thread_loop_()`：
1. `try_get_npu(camNum, 200ms)` 从 NPU 队列拉取 640×640 RGB888 DMA buffer
2. `inferFromDmaBuffer(dmaFd)` 零拷贝 RKNN 推理
3. 结果同时推入 `fusionQueue` 和 `osdQueue`
4. `NpuBufferGuard` RAII 归还 DMA buffer（含异常路径）

### 2.7 标题栏共享

`titleBar`（HW 监控 + 标题 + 时钟）从 pageMain 提升到根布局 `wrapperLayout`，位于 QStackedWidget 上方，所有三页共享显示。通过 `QHBoxLayout` 包裹实现 8px 左右边距，hwLabel 和 clockLabel 均为 280px 确保标题绝对居中。

### 2.7 析构顺序（含融合）

```
~Widget():
  1. FusionWorker stop → quit+wait → delete    // 先停跟踪轮询
  2. fusion_->stop()                            // 停融合线程
  3. lidar_->stop()                             // 停雷达（幂等）
  4. delete fusion_ / lidar_
  5. visioner_->camera_stream_ctrl(false)       // 停相机
  6. stop_preview_()                             // 停预览线程
  7. streamer_->remove_camera()                 // 清理推流/录像
  8. delete streamer_ / visioner_ / ui
```

## 3. 核心实现细节

### 3.1 预览管线

```
V4L2 capture thread
  → RGA: NV12 → 1080p RGB888 (rga_convert_to_rgb_full_)
  → previewTaskQueue.push(NpuPreview{npuImage, previewImage})

PreviewWorker 子线程
  → try_get_preview(200ms) → NpuPreview
  → QImage(previewImage->virtAddr, 1920, 1080, RGB888)  // 零拷贝引用
  → emit frameReady(img.copy())                           // 深拷贝后立即归还 DMA
  → release_preview()

主线程 on_frame_ready_
  → QPixmap::fromImage(image).scaled(labelSize, KeepAspectRatio)
  → previewLabel->setPixmap()
```

### 3.2 系统暂停/恢复 (camera_pause)

RK3588 ISP 驱动在 STREAMOFF 后仅靠 STREAMON 无法恢复，因此不停止硬件流：

```
关闭系统:
  1. stop_record / stop_stream    (停止编码和输出)
  2. stop_preview_()               (停止预览线程)
  3. camera_pause(0, true)         (capture 线程跳过 RGA，仅 QBUF 循环)

启动系统:
  1. camera_pause(0, false)        (恢复 RGA 处理，帧立即产生)
  2. start_preview_()              (重启预览线程)
  3. 启用推流/录像按钮
```

`camera_pause` 实现：capture 线程检查 `ctx->isPaused`，为 true 时跳过所有 RGA 操作和队列推送，仅将 buffer 归还 V4L2 驱动。

### 3.3 视频管理子页面

- **页面切换**: `QStackedWidget::setCurrentIndex(0/1)`
- **文件扫描**: `QDir::entryList("*.mp4", QDir::Time)` 按时间倒序
- **元数据读取**: `avformat_open_input → avformat_find_stream_info → codecpar->width/height, container duration`
- **时长换算**: 容器 `duration` 为 `AV_TIME_BASE` (微秒)，直接可用；流 `duration` 为 `time_base` 单位，需 `av_rescale_q` 转换
- **删除**: `QFile::remove` + `QMessageBox::question` 确认对话框
- **表头自适应**: 文件名列 `QHeaderView::Stretch`，其余列 `resizeColumnsToContents`

### 3.4 硬件监控

每秒由 `clockTimer_` 触发 `update_hw_usage_()`，ThermalController 接管温度和频率读
取，QT 继续读取利用率：

| 指标 | 数据源 | 读取者 | 格式 |
|------|--------|--------|------|
| CPU 利用率 | `/proc/stat` | QT | 相邻采样差分 |
| CPU 频率 | `policy4/scaling_cur_freq` | ThermalController | `thermalCtrl_->cpuBigFreq()` |
| RGA | `/sys/kernel/debug/rkrga/load` | QT | 逐行 `sscanf("load = %d%%")` 取 3 核 |
| NPU 利用率 | `/sys/kernel/debug/rknpu/load` | QT | 3 核百分比 |
| NPU 频率 | `fdab0000.npu/cur_freq` | ThermalController | `thermalCtrl_->npuFreq()` |
| 温度 | `/sys/class/thermal/thermal_zone0/temp` | ThermalController | `thermalCtrl_->currentTempC()` |
| 策略等级 | — | ThermalController | Normal/Warm/Hot/Critical |

### 3.5 ThermalController 集成

`thermal-controller` 静态库通过 `ThermalController` 类提供温控调频。Widget 构造时创
建实例，`update_hw_usage_()` 中每 1 秒调用 `tick()`：

1. `tick()` 内置 `read_sensors_()`：读 temp + 3 个 `cur_freq`，缓存到成员变量
2. 每 `intervalSec` 秒执行 `evaluate_and_apply_()`：4 级回滞策略评估
3. 仅在等级变化（或首次）时调用 `write_max_freq_()` 写 5 个 sysfs 频率上限节点
（policy0/policy4/policy6 + NPU max_freq）
4. 启动时 `startup_restore_()` 恢复全速；退出时 `restoreOnExit` 可选恢复

REST API：`GET /api/v1/thermal/status`（返回 `ThermalController::status_json()`）
WebSocket 推送：`get_hw_json_()` 中通过 `thermalCtrl_` 获取温度、等级、
`thermalFreq{cpuLittle, cpuBig, npu}`，附加到状态 JSON

### 3.6 按钮复用式设计

每个控制按钮（推流/录像）为单一 `QPushButton`，点击时根据 `is_streaming()/is_recording()` 判断当前状态执行启/停操作。`update_button_states_()` 动态切换文字和样式：

- 推流停止时: "启动推流" 绿色 `#238636`
- 推流进行时: "停止推流" 红色 `#da3633`
- 录像停止时: "启动录像" 蓝色 `#1f6feb`
- 录像进行时: "停止录像" 红色 `#da3633`

### 3.7 YOLO 推理生命周期管理

`SentinelYoloInfer* yoloInfer_` 采用懒加载模式，由 OSD 按钮或融合启用触发创建，两路都不需要时自动销毁。

**创建条件**（任一满足）：
- 用户按下 OSD 按钮（`on_btn_osd_`）
- 用户启用融合（`on_btn_fusion_toggle_` / `web_fusion_start_`）

**销毁条件**（全部满足）：
- 两路 OSD 都关闭（`!osdEnabled_[0] && !osdEnabled_[1]`）
- 融合未启用（`!fusionEnabled_`）

**Provider 绑定**：
- `streamer_->set_osd_provider(lambda)` — 从 `osdQueue` 获取，清空队列取最新帧
- `fusion_->set_detection_provider(lambda)` — 从 `fusionQueue` 获取

创建时两路 inference thread 都启动（`create_infer_thread(0)` + `create_infer_thread(1)`），与当前使用的相机数无关。

### 3.8 OSD 双端控制同步

OSD 状态通过 WebSocket `status` JSON 推送 `osdEnabled` 字段到 Web 前端，确保 Qt 桌面按钮和 Web 按钮状态一致。Web 端 OSD API 路由（`/api/v1/cam/{0,1}/osd/start|stop`）需在 `web_server.cpp` 显式注册（cpp-httplib HTTP 路由不经过 Widget 的 fallback handler）。

### 3.9 EIS 电子防抖集成

通过回调注入模式将 ICM45686 EIS 算法集成到 sentinel-visioner 采集管线，QT 和 Web 界面各相机独立控制。

**架构**:
```
Widget 拥有:
  Icm45686Reader (后台 std::thread, 100Hz 读 /dev/icm45686)
  EisStabilizer (绑定 reader, 陀螺仪积分 → 像素偏移)
  └─ visioner_->set_eis_offset_callback(lambda)
       └─ capture_thread_func_ 每帧调用 → 输出像素偏移 → rga_process_to_rgb_()
```

**线程安全**: 两路相机采集线程并发调用回调，`setAxisSign()` 不在回调内调用。init 时设 signX/Y=1.0，回调内对结果手动乘 per-camera 符号。

**控制**: 
- QT: `btnEis0/btnEis1` 按钮，切换 `eisEnabled_[0/1]`，首次触发懒加载 `init_eis_()`
- Web: `/api/v1/cam/{0,1}/eis/start|stop` POST 路由，与 QT 按钮双向同步
- 两路 EIS 都关闭时自动 `deinit_eis_()` 释放 IMU 资源

**status JSON**: 含 `eisEnabled` 字段，WebSocket 推送同步。

### 3.10 Web 远程控制集成

WebServer 在 SentinelQT 进程内运行独立 `std::thread`，通过 `BlockingQueuedConnection` 与 Qt 主线程安全通信。

**线程安全模型**:
- REST 命令：`QMetaObject::invokeMethod(widget, lambda, Qt::BlockingQueuedConnection)` 阻塞 WebServer 线程直到主线程执行完毕，确保 HTTP 响应包含操作结果
- 状态查询：同上，由主线程构建完整状态 JSON 后返回
- WebSocket 推送：`push_status()` 将 JSON 推入 `std::queue`（mutex 保护），广播线程每 50ms 消费发送，主线程非阻塞
- MJPEG 快照：预览帧由 `on_frame_ready_()` 写入 `QImage` 缓存（mutex 保护），HTTP handler 在锁内完成 JPEG 编码

**集成要点**:
- 构造函数中 `webServer_ = new WebServer(port)` + `set_command_handler` + `start()`
- `on_frame_ready_()` 中调用 `webServer_->set_cached_preview(camNum, image)` 更新缓存
- `update_hw_usage_()` 尾部调用 `webServer_->push_status(get_status_json_())` 推送状态 (1Hz)
- `on_tracking_updated_()` 中调用 `webServer_->push_tracking()` 推送跟踪数据
- 析构函数中**优先**调用 `webServer_->stop()` 再销毁其他成员，避免 BlockingQueuedConnection 死锁
- `get_hw_json_()` 使用独立 static 变量计算 CPU，不干扰 `update_hw_usage_()` 的 `prevCpuTotal_`/`prevCpuIdle_`

**暂停行为（Web vs QT 一致性）**: `web_pause_()` 暂停前先停止推流→录像→预览→最后暂停相机，与 `on_btn_pause_()` 逻辑一致，避免 PreviewWorker 无帧超时告警。

## 4. 配置文件

`config.ini` 使用 `QSettings::IniFormat`，路径为 `QCoreApplication::applicationDirPath() + "/config.ini"`，确保与可执行文件同目录。包含七节（含 EIS）：

- `[Camera0]` / `[Camera1]`: 设备路径、分辨率、RTSP 推流 URL、录像分辨率
- `[WebServer]`: Web 服务器端口 (`port=8080`) 和启用开关 (`enabled=true`)
- `[Record]`: 录像文件输出目录
- `[Lidar]`: 串口设备、波特率、扫描频率、测距范围
- `[Fusion]`: 跟踪器参数 + 每路相机内参

界面修改分辨率/融合参数时通过 `config_.setValue()` 即时写回文件。

### 3.11 融合参数管理

**配置加载**：`load_lidar_config_()` 和 `load_fusion_config_()` 从 `config.ini` 读取全部参数到 `fusionTrackerCfg_`（TrackerConfig）和 `fusionCamCfg_[2]`（CameraConfig）。细节参数（`minDtSec`、`requireClassIdMatch` 等）仅从 config.ini 读取，UI 不暴露。

**UI 构建**：`build_fusion_param_ui_()` 在 `paramScrollContent` 中动态创建参数行，每个参数含 `?` 帮助按钮（点击在对应 section 下方蓝底卡片显示说明，4 秒自动隐藏）、参数名标签、QLineEdit（含 QDoubleValidator/QIntValidator）、单位标签。说明标签按 section（跟踪器/CAM0/CAM1）独立创建，点击 `?` 时在对应 section 标题下方展示。

**双向同步**：
- `sync_fusion_config_to_ui_()`：config struct → QLineEdit
- `sync_ui_to_fusion_config_()`：QLineEdit → config struct → `config_.sync()` 写回文件
- 参数修改（`editingFinished`）触发 `on_fusion_param_changed_()`：若融合运行中，实时调用 `configure_tracker()` 和 `update_camera_intrinsics()` 热更新

**融合启停**：`on_btn_fusion_toggle_()` 控制完整生命周期：
1. sync UI → config → `configure_tracker` 推送参数
2. `lidar_->start()`（失败则终止并显示错误）
3. `fusion_->start(lidar_, fusionCamCfg_, fusionCamCount_)`（失败则回滚 lidar stop）
4. 创建 FusionWorker + QThread → 开始轮询
5. 开启 statusTimer (1Hz) → 更新状态栏目标数/告警数/距离值
6. 注册告警回调 `fusion_warning_callback_` → 终端 stderr 输出

禁用时逆序释放，`lidar_->stop()` 在从未 start 时幂等（内部检查 `running_` 标志和 `readerThread_.joinable()`）。

### 3.12 鸟瞰俯视图 (TopDownView)

自定义 `QWidget::paintEvent` 实现：

- **坐标映射**：中心 = 雷达原点 (0,0)，屏幕 X = LiDAR Y，屏幕 Y = -LiDAR X
- **网格层**：同心距离环 (1/2/5/10/15/20m) + 十字虚线轴
- **目标层**：按 TrackState 着色（Confirmed 绿 / Tentative 黄 / Coasting 灰），速度矢量箭头（长度 = `√(vx²+vy²) × 4px`，最大 22px），ID 标注
- **告警层**：红色脉冲圈（基于帧计数器正弦缩放动画）
- **图例**：右下角半透明方框（中文标识）

通过 `set_targets()` 更新数据，FusionWorker 每 100ms 无条件 emit 信号触发重绘（确保画面流畅不冻结）。

### 3.13 虚拟数字键盘 (VirtualKeyboard)

4×4 网格 QPushButton 布局：`[7 8 9 .] [4 5 6 -] [1 2 3 ←] [0 Del Done]`。默认隐藏，Widget::eventFilter 检测 param QLineEdit 的 FocusIn 事件自动弹出（`show_for(QLineEdit*)`），点击 Done 隐藏并清除焦点。`←` 退格、Del 删除、数字/小数点/负号在光标位置插入。

### 3.14 标题栏硬件监控格式

为适应共享标题栏的有限宽度（hwLabel 280px，11px 字体），硬件监控文本采用紧凑格式：
`45°C  CPU30  RGA5/2/1  NPU80/75/60`（去除了 `%` 符号，CPU 使用 `%1` 3 位右对齐）。
