# SentinelStreamer

推流与录像组件。从 `SentinelVisioner` 拉取 NV12 帧（支持任意分辨率），RGA 动态缩放为 720p + MPP 硬件编码 H.264，ffmpeg 子进程推 RTSP，FFmpeg API 写 MP4。720p 源直通录像（绕过 RGA 缩放）。

---

## 快速上手（3 步）

```cpp
// 1. 初始化帧源
SentinelVisioner visioner;
std::string dev("/dev/video11");
visioner.add_camera(dev, 1920, 1080, 8, 0);
visioner.camera_stream_ctrl(0, true);

// 2. 初始化推流器
SentinelStreamer streamer;
streamer.add_camera(0, &visioner);

// 3. 启停推流/录像
streamer.start_stream(0, "rtsp://127.0.0.1:8554/live/cam0");
streamer.start_record(0, "/tmp/output.mp4", RecordResolution::RES_1080P);
// ... 运行 ...
streamer.stop_record(0);
streamer.stop_stream(0);
streamer.remove_camera(0);
visioner.camera_stream_ctrl(0, false);
```

---

## 配合 Qt 使用

`SentinelStreamer` 内部自建推流线程，**无需继承 QObject，也不依赖 Qt**。通过状态回调将事件发射到 Qt 信号槽：

```cpp
class CameraController : public QObject {
    Q_OBJECT
public:
    CameraController() {
        streamer_.set_callback([](int cam, StreamerEvent e, const char* detail) {
            QString msg = detail ? detail : "";
            switch (e) {
                case StreamerEvent::STREAM_STARTED: emit instance().streamStarted(cam, msg); break;
                case StreamerEvent::STREAM_STOPPED: emit instance().streamStopped(cam);      break;
                case StreamerEvent::RECORD_STARTED: emit instance().recordStarted(cam, msg); break;
                case StreamerEvent::RECORD_STOPPED: emit instance().recordStopped(cam);      break;
                case StreamerEvent::ERROR:          emit instance().error(cam, msg);         break;
            }
        });
        visioner_.add_camera(dev_, 1920, 1080, 8, 0);
        streamer_.add_camera(0, &visioner_);
    }
    ~CameraController() { stop(); }

    static CameraController& instance() { static CameraController c; return c; }

public slots:
    void start() {
        visioner_.camera_stream_ctrl(0, true);
        streamer_.start_stream (0, "rtsp://127.0.0.1:8554/live/cam0");
        streamer_.start_record (0, "/tmp/record.mp4", RecordResolution::RES_1080P);
    }
    void stop() {
        streamer_.stop_record(0);
        streamer_.stop_stream(0);
        visioner_.camera_stream_ctrl(0, false);
    }

signals:
    void streamStarted(int cam, QString url);
    void streamStopped(int cam);
    void recordStarted(int cam, QString path);
    void recordStopped(int cam);
    void error(int cam, QString msg);

private:
    std::string dev_ = "/dev/video11";
    SentinelVisioner visioner_;
    SentinelStreamer streamer_;
};
```

> **注意**：回调在 SentinelStreamer 内部线程调用，跨线程 `emit` 需 Qt 队列连接。

---

## API 参考

### 生命周期 / 推流 / 录像 / OSD

| 方法 | 说明 |
|------|------|
| `add_camera(camNum, visioner)` | 注册摄像头（初始化编码器 + 缩放池） |
| `remove_camera(camNum)` | 注销摄像头（停推流/录像，释放资源） |
| `start_stream(camNum, rtspUrl)` | 启动 720p RTSP 推流（内部拉起 ffmpeg 子进程） |
| `stop_stream(camNum)` | 停止 RTSP 推流（关闭 ffmpeg 子进程） |
| `is_streaming(camNum)` | 查询推流状态 |
| `start_record(camNum, path, res)` | 启动 MP4 录像（res 可选 `RES_1080P` / `RES_720P`） |
| `stop_record(camNum)` | 停止 MP4 录像 |
| `is_recording(camNum)` | 查询录像状态 |
| `set_stream_osd_mode(camNum, mode)` | OSD 模式（`WITH_OSD` 预留） |

---

## Demo

```bash
# 基础推流+录像，30 秒
./sentinel_streamer_demo /dev/video11 rtsp://127.0.0.1:8554/live/cam0 /tmp/test.mp4 30

# 循环启停压测，5 轮
./sentinel_streamer_demo_cycle /dev/video11 rtsp://127.0.0.1:8554/live/cam0 5
```

---

## 编译 & 部署

```bash
./build.sh                              # 交叉编译
ls install/lib/libsentinel_streamer_lib.a   # 静态库
ls install/include/sentinel_streamer.h      # 头文件
```

板子上需要预先启动 **RTSP 服务器**（`resources/` 目录有 MediaMTX ARM64 包）和 **ISP 3A 服务**：

```bash
./mediamtx &
/etc/init.d/S40rkaiq_3A start
```

---

## 核心架构

```
SentinelVisioner
  └── processTaskQueue  →  streamThread (SentinelStreamer)
                              ├── RGA: 1080p → 720p
                              ├── streamEncCtx → H.264 → pipe → ffmpeg → RTSP
                              └── recordEncCtx → H.264 → av_write_frame → MP4
```

推流和录像各自独立 MPP 编码器，互不干扰。ffmpeg 子进程由 `popen` 管理，崩溃不影响主程序。

---

## 注意事项

1. **资源销毁**：编码器和输出上下文严格在线程退出后销毁，可反复启停
2. **推流前需启动 RTSP 服务器**，板子上配好 MediaMTX
3. **DMA 缓冲区**：组件内部严格"获取→使用→归还"，无需外部管理
4. **线程安全**：编码器和输出上下文严格在线程退出后销毁，可反复启停
