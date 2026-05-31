# WebControl — 技术实现文档

## 1. 概述

WebControl 是 RK3588-Omni-Sentinel 平台的 Web 远程控制组件，在 SentinelQT 进程中嵌入轻量级 HTTP/WebSocket 服务器，通过 REST API + WebSocket 实时推送实现浏览器远程操控。前端为单文件 SPA，完全复刻 QT 界面的视觉风格。

---

## 2. 架构总览

```
浏览器 (index.html SPA)
  │
  ├── HTTP REST (port 8080) ──── cpp-httplib::Server (独立 std::thread)
  │                                  │ QMetaObject::invokeMethod
  │                                  │ (BlockingQueuedConnection)
  │                                  ▼
  ├── WebSocket (port 8080) ────  Widget (Qt5 主线程)
  │                                  │ 直接调用
  │                                  ▼
  └── MJPEG snapshot.jpg ──────── sentinel-visioner / sentinel-streamer /
                                   sentinel-lslidarer / lidar-camera-fusion
                                   (现有 C++ 组件，零修改)
```

**设计原则**:
- HTTP 服务器与 Qt 主线程通过 `BlockingQueuedConnection` 同步调度，保证 UI 操作安全
- WebSocket 推送使用 `std::queue` + `std::mutex` 消息队列，Qt 主线程非阻塞投递
- 预览帧缓存（`QImage` + `std::mutex`）供 MJPEG 端点读取，避免跨线程竞争 DMA 缓冲区
- 前端 HTML 从文件系统加载（多路径搜索），编辑后无需重编译

---

## 3. 线程模型

| 线程 | 职责 | 生命周期 |
|------|------|---------|
| WebServer 线程 | 运行 cpp-httplib `listen()` 阻塞循环，处理 HTTP 请求和 WebSocket 连接 | `WebServer::start()` → `stop()` |
| 广播线程 | 每 50ms 消费消息队列，向所有 WebSocket 客户端广播 | 随 WebServer 线程启动/结束 |
| Qt 主线程 | 执行 `handle_web_command()` 和所有 UI 更新操作 | QApplication 事件循环 |
| cpp-httplib 线程池 (2) | 并行处理 HTTP 请求，通过 `BlockingQueuedConnection` 等待主线程结果 | httplib 内部管理 |

**同步机制**:
- REST 命令：`QMetaObject::invokeMethod(widget, lambda, Qt::BlockingQueuedConnection)` — 阻塞 WebServer 线程直到主线程完成
- 状态查询：同上，由主线程构建完整状态 JSON 后返回
- WebSocket 推送：`WebServer::push_*()` 将消息推入 `std::queue`（mutex 保护），广播线程消费发送
- MJPEG 快照：预览帧由 `on_frame_ready_()` 写入缓存（mutex 保护），HTTP handler 在锁内完成 JPEG 编码

**析构顺序**: WebServer 必须在 Widget 其他成员之前 `stop()` + `delete`，否则 BlockingQueuedConnection 可能在事件循环停止后死锁。

---

## 4. 核心数据流

### 4.1 REST 命令流（浏览器 → QT）

```
浏览器 fetch POST /api/v1/cam/0/stream/start
  → cpp-httplib 路由匹配 → wrap_post lambda
  → handle_api → cmdHandler_(method, path, body)
  → [WebServer 线程] QMetaObject::invokeMethod(widget, lambda, BlockingQueuedConnection)
  → [Qt 主线程] Widget::handle_web_command("POST", "/api/v1/cam/0/stream/start", "{}")
  → Widget::web_start_stream_(0)
  → streamer_->start_stream(0, rtspUrl)
  → update_camera_button_states_(0)
  → 返回 {"ok":true}
  → [WebServer 线程] HTTP 200 response
```

### 4.2 状态推送流（QT → 浏览器）

```
[Qt 主线程] QTimer 1Hz → Widget::update_hw_usage_()
  → ui->hwLabel->setText(...)
  → webServer_->push_status(get_status_json_())
  → [push_status] {"type":"status","data":{...}} 推入 sendQueue_ (mutex)
  → [广播线程 50ms] drain_send_queue()
  → 遍历 wsClients_ → ws.send(msg) → WebSocket → 浏览器
```

### 4.3 融合跟踪推送流

```
[FusionWorker 线程] 100ms poll → copy_tracked_targets()
  → emit trackingUpdated(QVector<TrackedTarget>)
  → [Qt 主线程] Widget::on_tracking_updated_()
  → topDownView_->set_targets(targets)
  → webServer_->push_tracking(tracking_json)
  → [广播线程] → WebSocket → 浏览器 Canvas 俯视图渲染
```

### 4.4 录像在线播放流

```
浏览器 点击 ▶ 播放按钮
  → GET /api/v1/playback?path=URL_ENCODED_PATH
  → cpp-httplib 路由直接处理（不走 CommandHandler，避免 BlockingQueuedConnection 延迟）
  → URL 解码路径（手动处理 %2F 等编码）
  → set_content_provider(fileSize, "video/mp4", lambda)
  → cpp-httplib 自动处理 Range 请求（bytes=START-END）
  → 流式读取文件块 → DataSink.write() → HTTP 206 响应
  → 浏览器 <video> 元素解码播放，支持拖拽 seek
```

### 4.5 推流视频显示流

```
浏览器 点击"开始推流"
  → POST /api/v1/cam/0/stream/start
  → sentinel-streamer 启动 ffmpeg 推 RTSP
  → toggleStream() 延迟 1s 后调用 startMjpegVideo()
  → 每 150ms 轮询 GET /api/v1/cam/0/snapshot.jpg
  → WebServer 从预览帧缓存（mutex 保护）读取 QImage → JPEG 编码
  → <img> 标签显示实时画面
```

---

## 5. WebSocket 协议

**服务端 → 客户端推送**:
```json
{"type":"status","data":{"cam0":{...},"cam1":{...},"hw":{...}}}
{"type":"tracking","data":{"targets":[...],"ts":123456789}}
{"type":"event","data":{"cam":0,"event":"stream_started","detail":"rtsp://..."}}
```

**客户端 → 服务端**（可选，通过 WebSocket 发送命令）:
```json
{"method":"POST","path":"/api/v1/cam/0/stream/start","body":"{}"}
```

---

## 6. 前端 SPA 设计

单文件 `index.html`，仪表盘式单页布局，所有功能一页呈现：

- **顶部栏**: Logo + 硬件监控（温度/CPU/RGA/NPU）+ 时钟
- **相机行（2 列）**: 双路相机卡片，各含 16:9 视频预览区 + 控制按钮（预览/推流/录像/暂停）+ 状态指示灯
- **状态栏**: 录像信息 + 系统状态
- **底部行（2 列）**: 左列融合跟踪（雷达图 + 参数面板），右列系统控制（上半）+ 录像文件列表（下半，含在线播放按钮）
- **播放器**: 全屏覆盖层，支持 Range 拖拽 seek，路径 `GET /api/v1/playback?path=`

**完全复刻 QT 配色**:
- 根背景 `#549688`、卡片 `#F4EAC5`/`#F5F0D7`、强调色 `#58a6ff`
- 语义按钮：绿=启动 `#238636`、红=停止 `#da3633`、蓝=录像 `#1f6feb`、黄=暂停 `#d29922`
- 字体层级：标题 18px/600、正文 12-14px、按钮 12px/600

---

## 7. 关键设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| JSON 库 | nlohmann/json | 单头文件久经考验，支持嵌套对象/数组等多种 schema |
| HTTP 库 | cpp-httplib (master) | MIT 协议，单头文件，支持 HTTP/HTTPS/WebSocket |
| 视频传输 | MJPEG 轮询 (150ms) | 不依赖 mediamtx WebRTC 配置，利用现有预览帧缓存 |
| 线程通信 | BlockingQueuedConnection | 确保 HTTP 响应包含操作结果，避免异步回调复杂性 |
| 前端部署 | 文件系统加载 | 编辑 HTML 后重启即可生效，无需重新编译 |
