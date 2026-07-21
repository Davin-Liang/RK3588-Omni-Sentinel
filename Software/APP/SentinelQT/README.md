# SentinelQT

基于 Qt5 Widgets 的嵌入式触屏应用程序，运行于 RK3588 ARM64 Linux 平台。集成 sentinel-visioner（双路相机视觉管线）、sentinel-streamer（推流/录像）、sentinel-lslidarer（激光雷达驱动）和 lidar-camera-fusion（视觉-雷达融合跟踪），提供全触屏操作的双路实时监控与融合目标跟踪界面。

## 功能概述

- **双路实时预览**：左右并排同时显示 CAM0（ISP 1080p）和 CAM1（USB 720p）预览画面。两个独立 `PreviewWorker` 各自运行在独立 QThread，通过 lambda 捕获 camNum 路由帧到对应 label。每路预览可独立开启/关闭，关闭预览不中断底层帧捕获。

- **按相机独立控制**：每路相机拥有独立 4 按钮（预览切换、推流启停、录像启停、暂停/恢复）。全局系统按钮一键启停两路。按钮运行时变色，状态一目了然。

- **推流与录像分离控制**：基于 `SentinelStreamer` 的双编码器架构，每路相机独立启停 RTSP 推流（720p MPP 硬编码 → ffmpeg 子进程）和 MP4 录像（1080p/720p）。USB 相机录像强制 720p。

- **录像分辨率可选**：设置行提供 `QComboBox` 下拉选择 CAM0 录像分辨率 1080p 或 720p，CAM1 固定 720p。

- **视频文件管理子页面**：通过 `QStackedWidget` 切换到独立子页面，`QTableWidget` 列出录像目录下所有 `.mp4` 文件。分辨率和时长通过 libavformat 读取封装层元数据，每行附带"删除"按钮（含二次确认对话框）。

- **系统暂停/恢复**：按相机独立 `camera_pause(camNum, paused)` 暂停/恢复 RGA 处理管线，底层 V4L2 流保持活跃不 STREAMOFF。暂停时自动停止该路推流、录像和预览线程。

- **硬件状态监控 + 主动温控调频**：标题栏左侧实时显示（1 秒刷新）：
  - **温度**：`/sys/class/thermal/thermal_zone0/temp`
  - **温控等级**：Normal / Warm / Hot / Critical（4 级回滞）
  - **CPU 利用率 + 频率**：`/proc/stat` 差分计算 + policy4 当前频率
  - **RGA 利用率**：`/sys/kernel/debug/rkrga/load`，逐核显示
  - **NPU 利用率 + 频率**：`/sys/kernel/debug/rknpu/load`，逐核显示 + fdab0000.npu cur_freq
  - **日期时间**：标题栏右侧显示 `yyyy-MM-dd HH:mm:ss`
  - 标题栏在三个页面顶部共享显示
  - 主动温控：CPU（3 簇）+ NPU 频率上限自动调节，温度上 65°C 渐降，下 60°C 恢复。新增 `thermal-controller` 静态库实现策略引擎，详见 `thermal-controller/` 目录

- **融合目标跟踪子页面**：基于 `SentinelLslidarer` + `LidarCameraFusion`，通过 `QStackedWidget` 切换到独立子页面（第 3 页）：
  - **鸟瞰俯视图**：自定义 `TopDownView` 组件实时绘制跟踪目标位置、速度矢量箭头、告警脉冲圈，支持距离网格和图例
  - **参数动态配置**：9 个跟踪器参数 + 每路相机 4 个内参参数，通过嵌入式虚拟数字键盘触屏输入，每个参数带 "?" 帮助按钮说明
  - **融合启停控制**：一键启用/停止雷达驱动、融合线程和跟踪轮询，启动失败自动回滚
  - **实时状态显示**：底部状态栏左侧显示目标总数/已确认数/告警数，右侧显示最多 3 个目标的实际距离值
  - **告警输出**：距离告警同时在俯视图（红色脉冲圈）、状态栏（告警计数）和终端 stderr（详细日志）三层输出
  - **假检测模式**：NPU 推理未就绪时使用内置虚构检测框验证全链路，参数实时热更新无需重启融合

- **双路状态栏**：底部状态栏自动合并两路相机状态（`CAM0: xxx | CAM1: xxx`），全局消息直接显示。

- **时间戳文件名**：录像文件命名格式 `camX_record_yyyyMMdd_HHmmss.mp4`。

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

## 板端运行

RK3588 的 Buildroot 默认使用 Weston (Wayland 合成器) 管理显示，与 Qt eglfs 冲突，需先停止：

```bash
# 1. 停止 Weston
killall -9 weston

# 2. 以 root 权限启动（debugfs 文件需 root 读取）
sudo ./SentinelQT -platform eglfs

# 3. 永久禁用 Weston 开机自启（可选）
mv /etc/init.d/S50weston /etc/init.d/disabled.S50weston
```

`-platform eglfs` 让 Qt 直接通过 DRM/KMS 渲染，绕过 Wayland，获得最佳性能和全屏体验。

## 配置文件

`config.ini` 按相机分节，USB 相机分辨率上限 720p（超限自动钳位）：

```ini
[Camera0]
device=/dev/video11
width=1920
height=1080
streamUrl=rtsp://127.0.0.1:8554/live/cam0
recordResolution=1080

[Camera1]
device=/dev/video21
width=1280
height=720
streamUrl=rtsp://127.0.0.1:8554/live/cam1
recordResolution=720

[Record]
dir=/mnt/sdcard
```

| 键 | 节 | 说明 | 默认值 |
|----|-----|------|--------|
| `device` | `[Camera0/1]` | V4L2 设备节点 | `/dev/video11` / `/dev/video21` |
| `width/height` | `[Camera0/1]` | 采集分辨率（CAM1 上限 1280×720） | 1920×1080 / 1280×720 |
| `streamUrl` | `[Camera0/1]` | RTSP 推流目标地址 | 各不同 |
| `recordResolution` | `[Camera0/1]` | 录像分辨率（CAM1 强制 720） | 1080 / 720 |
| `dir` | `[Record]` | 录像文件保存目录 | `/mnt/sdcard` |
| `device` | `[Lidar]` | 雷达串口设备 | `/dev/sentinel_lidar` |
| `baudRate` | `[Lidar]` | 雷达波特率 | `460800` |
| `camCount` | `[Fusion]` | 融合相机数量 | `1` |
| `clusterEpsMeters` | `[Fusion]` | 聚类半径 (m) | `0.5` |
| `alpha` / `beta` | `[Fusion]` | Alpha-Beta 滤波增益 | `0.7` / `0.3` |
| `maxAssociationDistMeters` | `[Fusion]` | 关联门限距离 (m) | `2.0` |
| `minHitsToConfirm` | `[Fusion]` | 确认航迹所需帧数 | `3` |
| `maxCoastingFrames` | `[Fusion]` | 丢失外推帧数 | `5` |
| `warningEnterDistMeters` | `[Fusion]` | 告警触发距离 (m) | `3.0` |
| `warningExitDistMeters` | `[Fusion]` | 告警解除距离 (m) | `3.5` |
| `Cam0Fx` 等 | `[Fusion]` | 相机 0/1 内参 (px) | `400` |
| `enabled` | `[Thermal]` | 温控开关 | `true` |
| `intervalSec` | `[Thermal]` | 策略评估间隔 (s) | `2` |
| `warmThreshold` / `warmRecover` | `[Thermal]` | Warm 升温/恢复阈值 (°C) | `65` / `60` |
| `hotThreshold` / `hotRecover` | `[Thermal]` | Hot 升温/恢复阈值 (°C) | `75` / `70` |
| `critThreshold` / `critRecover` | `[Thermal]` | Critical 升温/恢复阈值 (°C) | `85` / `80` |
| `cpuBig*` | `[Thermal]` | CPU A76 各级频率上限 (kHz) | 2304000 / 1800000 / 1200000 / 816000 |
| `cpuLittle*` | `[Thermal]` | CPU A55 各级频率上限 (kHz) | 1800000 / 1416000 / 1008000 / 600000 |
| `npu*` | `[Thermal]` | NPU 各级频率上限 (Hz) | 1000000000 / 800000000 / 600000000 / 300000000 |

## Visual-Primary + IMU-Assisted EIS 集成说明

当前版本的防抖不是离线工具，而是已接入最终 `SentinelQT` 主程序。运行方式仍为：

```bash
./SentinelQT -platform eglfs
```

点击界面中的“防抖”按钮后，`SentinelQT` 会调用 `SentinelVisioner::enable_visual_eis()` 开启对应相机的实时视觉 EIS。`sentinel-visioner` 在采集线程内使用 LK 光流估计相邻帧画面运动，结合 ICM45686 输出的 `gyroRms / vibrationLevel` 动态调整平滑强度，并通过 RGA 输出防抖后的预览和处理画面。

详细说明见：`docs/VISUAL_IMU_EIS_RUNTIME_INTEGRATION.md`。
