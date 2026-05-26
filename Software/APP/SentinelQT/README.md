# SentinelQT

基于 Qt5 Widgets 的嵌入式触屏应用程序，运行于 RK3588 ARM64 Linux 平台。集成 sentinel-visioner（相机视觉管线）和 sentinel-streamer（推流/录像），提供全触屏操作的实时监控界面。

## 功能概述

- **1080p 实时预览**：通过独立 RGA 管线获取 RGB888 预览图，经 `PreviewWorker` 子线程异步拉取，`QImage` → `QPixmap` 渲染到 `QLabel`。预览启停与 V4L2 硬件流解耦，关闭预览不中断底层帧捕获，NPU/推流/录像仍可照常运行。

- **推流与录像分离控制**：基于 `SentinelStreamer` 的双编码器架构，推流（720p MPP 硬编码 → ffmpeg 子进程 RTSP）与录像（1080p/720p MPP 硬编码 → MP4）各自独立启停，互不干扰。三组复用式按钮（推流/录像/系统），一键启停，运行时按钮变色，状态一目了然。

- **录像分辨率可选**：设置行提供 `QComboBox` 下拉选择 1080p 或 720p，选择立即写入 `config.ini`。

- **视频文件管理子页面**：通过 `QStackedWidget` 切换到独立子页面，`QTableWidget` 列出录像目录下所有 `.mp4` 文件。分辨率和时长通过 libavformat 读取封装层元数据，每行附带"删除"按钮（含二次确认对话框）。

- **系统暂停/恢复**：`camera_pause(true/false)` 暂停/恢复 RGA 处理管线，底层 V4L2 流保持活跃不 STREAMOFF。暂停时自动停止推流、录像和预览线程，恢复时自动重启。

- **硬件状态监控**：标题栏左侧实时显示（1 秒刷新）：
  - **温度**：`/sys/class/thermal/thermal_zone0/temp`
  - **CPU 利用率**：`/proc/stat` 差分计算
  - **RGA 利用率**：`/sys/kernel/debug/rkrga/load`，逐核显示
  - **NPU 利用率**：`/sys/kernel/debug/rknpu/load`，逐核显示

- **时间戳文件名**：录像文件命名格式 `record_yyyyMMdd_HHmmss.mp4`，由 `QDateTime::currentDateTime()` 生成。

## 构建与部署

### 交叉编译

需要 aarch64 交叉编译工具链和 Qt5 交叉编译 SDK：

```bash
cd SentinelQT/

# 通过环境变量指定工具链路径（可选）
export CROSS_COMPILE_PATH=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
export QT5_PREFIX=$CROSS_COMPILE_PATH/aarch64-buildroot-linux-gnu/sysroot/usr/lib/cmake/Qt5

./build.sh
```

产物输出到 `SentinelQT/install/SentinelQT` 可执行文件及 `config.ini`。

### RTSP 服务器准备

推流前需启动 RTSP 服务（如 [mediamtx](https://github.com/aler9/mediamtx)）：

```bash
./mediamtx
```

默认推流地址 `rtsp://127.0.0.1:8554/live/cam0`，VLC 或 ffplay 连接同一地址即可拉流。

## 配置文件

`config.ini` 格式：

```ini
[Stream]
rtspUrl=rtsp://127.0.0.1:8554/live/cam0

[Record]
dir=/mnt/sdcard
resolution=1080
```

| 键 | 节 | 说明 | 默认值 |
|----|-----|------|--------|
| `rtspUrl` | `[Stream]` | RTSP 推流目标地址 | `rtsp://127.0.0.1:8554/live/cam0` |
| `dir` | `[Record]` | 录像文件保存目录 | `/mnt/sdcard` |
| `resolution` | `[Record]` | 默认录像分辨率，`1080` 或 `720` | `1080` |
