# WebControl

Web 远程控制组件。在 SentinelQT 进程中嵌入 HTTP/WebSocket 服务器（cpp-httplib），提供 REST API 远程操控板端设备，并通过单文件 SPA 前端实现与 QT 界面风格一致的浏览器控制台。推流画面通过 MJPEG 轮询显示。

---

## 快速上手（3 步）

```cpp
// 1. 创建服务器
WebServer server(8080);

// 2. 注册命令处理器（运行在 Qt 主线程）
server.set_command_handler([](const std::string& method,
                               const std::string& path,
                               const std::string& body) -> std::string {
    // 处理 REST 命令，返回 JSON 响应
    if (path == "/api/v1/status")
        return R"({"ok":true,"system":"online"})";
    return R"({"ok":false})";
});

// 3. 启动服务器
server.start();
// ... 运行 ...
server.stop();
```

浏览器打开 `http://<板端IP>:8080` 即可访问控制台。

---

## 在 SentinelQT 中集成

```cpp
// Widget 构造函数中
webServer_ = new WebServer(8080);
webServer_->set_command_handler([this](auto& method, auto& path, auto& body) {
    std::string result;
    QMetaObject::invokeMethod(this, [&]() {
        result = handle_web_command(method, path, body);
    }, Qt::BlockingQueuedConnection);
    return result;
});
webServer_->start();

// 定时推送状态到 WebSocket 客户端
webServer_->push_status(get_status_json_());
webServer_->push_tracking(tracking_json);
webServer_->push_event("stream_started", detail_json);

// 析构函数中优先停止
webServer_->stop();
delete webServer_;
```

---

## API 参考

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/v1/status` | GET | 完整系统状态（相机、硬件、融合） |
| `/api/v1/status/hw` | GET | 硬件状态（CPU/温度/RGA/NPU） |
| `/api/v1/cam/{0,1}/stream/start` | POST | 启动 RTSP 推流 |
| `/api/v1/cam/{0,1}/stream/stop` | POST | 停止推流 |
| `/api/v1/cam/{0,1}/record/start` | POST | 启动 MP4 录像 |
| `/api/v1/cam/{0,1}/record/stop` | POST | 停止录像 |
| `/api/v1/cam/{0,1}/pause` | POST | 暂停相机（先停推流/录像/预览） |
| `/api/v1/cam/{0,1}/resume` | POST | 恢复相机 |
| `/api/v1/system/start` | POST | 一键启动所有相机 |
| `/api/v1/system/stop` | POST | 一键停止所有相机 |
| `/api/v1/lidar/start` | POST | 启动激光雷达 |
| `/api/v1/lidar/stop` | POST | 停止激光雷达 |
| `/api/v1/fusion/start` | POST | 启动融合跟踪 |
| `/api/v1/fusion/stop` | POST | 停止融合跟踪 |
| `/api/v1/fusion/config` | GET/POST | 读取/更新跟踪器参数 |
| `/api/v1/fusion/camera/{0,1}/intrinsics` | POST | 更新相机内参 |
| `/api/v1/videos` | GET | 录像文件列表 |
| `/api/v1/videos` | DELETE | 删除录像文件 |
| `/api/v1/cam/{0,1}/record-resolution` | PUT | 设置录像分辨率 |
| `/api/v1/cam/{0,1}/snapshot.jpg` | GET | MJPEG 单帧快照 |
| `/ws` | WebSocket | 实时状态/跟踪/事件推送 |

---

## 编译 & 部署

```bash
./build.sh                                      # 本地测试编译
# 或作为 SentinelQT 子目录交叉编译：
cd ../SentinelQT && ./build.sh                  # 产物在 install/web/index.html
```

板端运行时 Web 前端从 `web/index.html` 加载（支持多路径搜索），无需重新编译即可更新界面。

---

## 核心架构

```
浏览器 (index.html SPA)
  │
  ├── HTTP REST (8080) ──── WebServer (cpp-httplib, std::thread)
  │                              │ BlockingQueuedConnection
  │                              ▼
  ├── WebSocket (8080) ────  Widget (Qt5 主线程)
  │                              │
  │                              ▼
  └── MJPEG snapshot ─────── sentinel-visioner / sentinel-streamer / ...
```

WebServer 线程与 Qt 主线程通过 `QMetaObject::invokeMethod` + `BlockingQueuedConnection` 安全通信。WebSocket 推送使用消息队列（mutex + queue），Qt 主线程非阻塞投递。

---

## 注意事项

1. **线程安全**：REST 命令通过 `BlockingQueuedConnection` 同步调度到主线程，WebSocket 推送通过消息队列解耦
2. **析构顺序**：WebServer 必须在 Widget 其他成员之前停止，避免 BlockingQueuedConnection 死锁
3. **httplib 功能开关**：不要 `#define CPPHTTPLIB_OPENSSL_SUPPORT` 或 `CPPHTTPLIB_ZLIB_SUPPORT`（用 `#ifdef` 检测，定义为 0 照样启用）
4. **SPA 热加载**：HTML 从文件系统加载，编辑 `web/index.html` 后重启 SentinelQT 即可生效
