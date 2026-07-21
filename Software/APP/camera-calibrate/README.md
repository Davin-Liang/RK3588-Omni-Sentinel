# camera-calibrate — 相机标定工具

基于原生 V4L2 API + OpenCV 的棋盘格相机标定命令行工具，支持 RK3588 ISP MPLANE 格式。

## 功能

- 自动检测棋盘格并采集多角度图像
- OpenCV 标定输出内参（fx, fy, cx, cy）和畸变系数（k1）
- 自动计算 NPU 640×640 letterbox 缩放内参
- 支持 `--save-frames` 保存原图+角点标注图，方便离线检查
- 板端无 GUI 自动采集模式

## 构建

```bash
cd camera-calibrate
./build.sh
# 产物: install/camera_calibrate, install/lib/*.so
```

依赖 `../3rdparty/opencv`（预编译 aarch64）。

## 用法

```bash
# 基本用法（ISP 相机）
./camera_calibrate /dev/video11 25

# USB 相机 + 自定义棋盘格
./camera_calibrate /dev/video21 30 --boards 8x6 --min-frames 15

# 保存每帧图片到指定目录
./camera_calibrate /dev/video11 25 --save-frames ./frames

# 完整参数
./camera_calibrate <device> <square_mm> [--boards WxH] [--min-frames N] [--resolution WxH] [--output FILE] [--save-frames DIR]
```

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `<device>` | V4L2 设备路径 | `/dev/video11` |
| `<square_mm>` | 棋盘格每格边长 (mm) | - |
| `--boards WxH` | 内角点数 | 9×6 |
| `--min-frames N` | 最少采集帧数 | 20 |
| `--resolution WxH` | 采集分辨率 | 1920×1080 |
| `--output FILE` | 输出文件 | `<device>_calib.txt` |
| `--save-frames DIR` | 保存采集帧的目录 | 不保存 |

## 操作

1. 打印棋盘格（推荐 9×6 内角点），量好每格边长
2. 运行程序，自动检测棋盘格并捕获不同位置/角度
3. 同一位置自动跳过，两次捕获间有 5 帧冷却
4. 凑够帧数后自动标定，输出 NPU 640×640 内参和 config.ini 片段

## 将结果填入 config.ini

标定后复制输出中的 `[config.ini-snippet]` 节到 `SentinelQT/config.ini` 的 `[Fusion]` 节，将 `CamX` 替换为 `Cam0` 或 `Cam1`。
