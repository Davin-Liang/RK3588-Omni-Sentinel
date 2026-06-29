```
 ____  _  ___________   ___   ___      ___                  _      ____             _   _            _
|  _ \| |/ /___ / ___| ( _ ) ( _ )    / _ \ _ __ ___  _ __ (_)    / ___|  ___ _ __ | |_(_)_ __   ___| |
| |_) | ' /  |_ \___ \ / _ \ / _ \   | | | | '_ ` _ \| '_ \| |____\___ \ / _ \ '_ \| __| | '_ \ / _ \ |
|  _ <| . \ ___) |__) | (_) | (_) |  | |_| | | | | | | | | | |_____|__) |  __/ | | | |_| | | | |  __/ |
|_| \_\_|\_\____/____/ \___/ \___/    \___/|_| |_| |_|_| |_|_|    |____/ \___|_| |_|\__|_|_| |_|\___|_|
```

基于瑞芯微 **RK3588** 的边缘端多传感器融合平台。集成激光雷达驱动、相机视觉管线、NPU YOLO 推理、传感器融合跟踪、RTSP 推流录像（含 OSD 叠加）、IMU 电子防抖、嵌入式触控界面七大组件，全部脱离 ROS，以 C++14 静态库形式存在，通过 **DMA-BUF** 在 NPU / RGA / V4L2 硬件加速器之间实现零拷贝数据流转。

---

## 🏗️ 系统架构

```
                        /dev/video11 (MIPI-CSI ISP 1080p)
                        /dev/video21 (USB UVC 720p)
                               │
                               ▼
                    SentinelVisioner (视觉管线)
                    双路"一分三"零拷贝扇出
                    ├── NPU 推理小图 (640×640 RGB888)
                    ├── 预览大图 (RGB888, 1080p/720p)
                    └── 推流副本 (NV12, 1080p/720p)
                         │           │            │
                         ▼           ▼            ▼
                 SentinelYoloInfer  SentinelQT  SentinelStreamer
                 (YOLOv8 RKNN推理) (双路触控HMI) ├─ RGA 缩放 → 720p
                    │              │   │         ├─ OSD 叠加(YOLO框)
                    ▼              │   │         ├─ MPP H.264 编码
             LidarCameraFusion ←───┘   │         ├─ ffmpeg → RTSP
             (视觉-雷达融合跟踪)        │         └─ FFmpeg API → MP4
                    │                  │
                    └──── 告警 ←───────┘

 /dev/sentinel_lidar (N10Plus 单线雷达, 10Hz)
         │
         ▼
 SentinelLslidarer (雷达驱动)
 三层架构: SerialPort → RingBuffer(SWCR) → 协议解码
         │
         ▼
 LidarCameraFusion (融合跟踪)
 外参变换 + 内参投影 + Alpha-Beta 跟踪
         │
         ▼
    SentinelQT (UI 展示航迹 + 告警)
```

**数据流向**: 相机和雷达各自独立采集 → 时间戳在 `CLOCK_MONOTONIC` 域对齐 → 融合引擎输出跟踪航迹 → UI 实时展示。

---

## 🧩 组件地图

| 组件 | 目录 | 角色 | 依赖 |
|------|------|------|------|
| **SentinelLslidarer** | `Software/APP/sentinel-lslidarer/` | 镭神 N10Plus 单线雷达驱动，SWCR 无锁环形缓冲区，时间戳融合接口 | 仅 libpthread |
| **SentinelVisioner** | `Software/APP/sentinel-visioner/` | 双路相机视觉管线（ISP 1080p + USB 720p），V4L2 + RGA 硬件加速"一分三"零拷贝扇出 | dma-buffer-pool, librga |
| **SentinelYoloInfer** | `Software/APP/sentinel-yolo-infer/` | RKNN YOLOv8 NPU 推理，双队列输出（融合+OSD），DMA-BUF 零拷贝 | sentinel-visioner, rknpu2, lidar-camera-fusion (类型) |
| **LidarCameraFusion** | `Software/APP/lidar-camera-fusion/` | 视觉-雷达数据融合，Alpha-Beta 多目标跟踪，四态生命周期管理 | sentinel-lslidarer (仅头文件) |
| **SentinelStreamer** | `Software/APP/sentinel-streamer/` | RTSP 推流 + MP4 录像 + OSD 叠加，MPP 硬件编码 H.264，双编码器独立架构 | sentinel-visioner, librga, ffmpeg |
| **SentinelQT** | `Software/APP/SentinelQT/` | Qt5 Widgets 嵌入式触控 HMI，双路预览 + 推流/录像/暂停/OSD 控制 + 融合管理 | sentinel-visioner, sentinel-streamer, sentinel-yolo-infer, Qt5 |
| **icm45686-eis-app-parameterized** | `Software/APP/icm45686-eis-app-parameterized/` | ICM45686 电子防抖（参数化版），ImuConfig/EisCameraConfig 双相机独立配置，平滑内置 Stabilizer，Web 热修改参数，录制双输出（防抖/未防抖对照） | 仅 libpthread + libm |
| **DmaBufferPool** | `Software/APP/dma-buffer-pool/` | DMA 内存池，O(1) 空闲链表分配/归还 | librga, libdrm |

每个组件目录下均有独立的 `README.md` 和完整文档，详见各组件的 `docs/` 目录。

---

## 🖥️ 硬件平台

| 项目 | 规格 |
|------|------|
| **SoC** | Rockchip RK3588 (4×A76 + 4×A55, Mali-G610 GPU, 6 TOPS NPU) |
| **摄像头** | 双路：MIPI-CSI ISP (`/dev/video11`, 1080p NV12) + USB UVC (`/dev/video21`, 720p NV12/YUYV) |
| **激光雷达** | 镭神 N10Plus 单线 TOF, 串口 460800 baud, 10Hz, 540 点/圈 |
| **操作系统** | Linux (Buildroot), ARM64, 无 ROS 运行时 |
| **显示** | DSI 触屏, Qt5 eglfs 直接渲染 (DRM/KMS) |

---

## 🚀 快速构建

全部组件使用 **aarch64 交叉编译**，在 x86 开发机上运行。每个组件独立编译，产物输出到各自的 `install/` 目录。

```bash
# 1. 设置交叉编译器路径（按实际 SDK 路径修改各组件 build.sh 中的 TOOL_CHAIN）

# 2. 按依赖顺序编译
cd Software/APP/dma-buffer-pool   && ./build.sh   # DMA 内存池（基础设施）
cd ../sentinel-lslidarer          && ./build.sh   # 雷达驱动（无依赖）
cd ../sentinel-visioner           && ./build.sh   # 视觉管线（依赖 dma-buffer-pool）
cd ../lidar-camera-fusion         && ./build.sh   # 融合跟踪（依赖 lslidarer 头文件）
cd ../sentinel-yolo-infer         && ./build.sh   # YOLO NPU 推理（依赖 visioner + rknpu2）
cd ../sentinel-streamer           && ./build.sh   # 推流录像 + OSD（依赖 visioner）
cd ../SentinelQT                  && ./build.sh   # 触控界面（依赖 visioner + streamer）
```

**编译产物**:
- 静态库: `install/lib/lib*.a`
- 头文件: `install/include/`
- Demo 可执行文件: `install/*_demo`

**板端运行**（以 SentinelQT 为例）:
```bash
killall -9 weston                      # 停止 Wayland 合成器
./mediamtx &                           # 启动 RTSP 服务器
/etc/init.d/S40rkaiq_3A start         # 启动 ISP 3A 服务
sudo ./SentinelQT -platform eglfs      # 启动触控界面
```

---

## 📖 文档索引

| 文档 | 说明 |
|------|------|
| [飞书技术手册](https://my.feishu.cn/docx/WDmEd9HnKo8eg1xAlN9cwez6nUb) | 5 组件完整技术文档，含架构设计、设计决策对比、面试备战（每章 9 节统一结构） |
| `Software/APP/CLAUDE.md` | 编码规范、构建系统、架构约定、文档规范 |
| `sentinel-lslidarer/` | [README](Software/APP/sentinel-lslidarer/README.md) · [实现](Software/APP/sentinel-lslidarer/docs/IMPLEMENTATION.md) · [学习指南](Software/APP/sentinel-lslidarer/docs/LEARNING_GUIDE.md) · [Bug记录](Software/APP/sentinel-lslidarer/BUG_RECORD.md) |
| `sentinel-visioner/` | [README](Software/APP/sentinel-visioner/README.md) · [实现](Software/APP/sentinel-visioner/docs/IMPLEMENTATION.md) · [学习指南](Software/APP/sentinel-visioner/docs/LEARNING_GUIDE.md) · [Bug记录](Software/APP/sentinel-visioner/BUG_RECORD.md) |
| `sentinel-yolo-infer/` | [README](Software/APP/sentinel-yolo-infer/README.md) · [实现](Software/APP/sentinel-yolo-infer/docs/IMPLEMENTATION.md) · [学习指南](Software/APP/sentinel-yolo-infer/docs/LEARNING_GUIDE.md) · [Bug记录](Software/APP/sentinel-yolo-infer/BUG_RECORD.md) |
| `lidar-camera-fusion/` | [README](Software/APP/lidar-camera-fusion/README.md) · [实现](Software/APP/lidar-camera-fusion/docs/IMPLEMENTATION.md) · [学习指南](Software/APP/lidar-camera-fusion/docs/LEARNING_GUIDE.md) · [Bug记录](Software/APP/lidar-camera-fusion/BUG_RECORD.md) |
| `sentinel-streamer/` | [README](Software/APP/sentinel-streamer/README.md) · [实现](Software/APP/sentinel-streamer/docs/IMPLEMENTATION.md) · [学习指南](Software/APP/sentinel-streamer/docs/LEARNING_GUIDE.md) · [Bug记录](Software/APP/sentinel-streamer/BUG_RECORD.md) |
| `SentinelQT/` | [README](Software/APP/SentinelQT/README.md) · [实现](Software/APP/SentinelQT/docs/IMPLEMENTATION.md) · [学习指南](Software/APP/SentinelQT/docs/LEARNING_GUIDE.md) · [Bug记录](Software/APP/SentinelQT/BUG_RECORD.md) |

---

## 💡 核心设计原则

**零拷贝**: 全链路通过 DMA-BUF 文件描述符传递图像数据，应用层不执行 `mmap` 或 CPU `memcpy` 像素搬运。RGA 硬件在搬移过程中顺便完成格式转换（NV12→RGB888）和缩放。

**预分配**: 所有内存池（DMA Buffer Pool、雷达环形缓冲区、融合跟踪缓冲区）均在初始化时一次性分配，运行时零 `new`/`delete`。避免堆碎片和分配延迟抖动。

**统一时钟**: 全系统统一使用 `CLOCK_MONOTONIC` 纳秒时间戳。相机帧、雷达帧、融合检索全部在同一时间域，`get_closest_frame(cameraTsNs)` 的时间对齐才有意义。

**无 ROS 依赖**: 所有应用层组件脱离 ROS/ROS2，编译为独立静态库。仅依赖 POSIX 系统调用、标准 C++14 和 RK3588 硬件 SDK（RGA、MPP）。

**模块隔离**: 每个组件独立编译、独立测试。组件间通过明确的公共头文件和运行时 API 交互，不共享全局状态。

---

## 📋 开发约定

- 各开发者在自己分支上工作，**勿交叉修改他人负责的组件**
- 第三方库存放在 `Software/APP/3rdparty/` 目录下
- 编译中间文件在 `build/` 目录，由 `.gitignore` 统一忽略
- 编码规范、命名约定、文档规范详见 [`Software/APP/CLAUDE.md`](Software/APP/CLAUDE.md)
- 每个组件遵循四类文档规范：README（概览）、IMPLEMENTATION（实现）、LEARNING_GUIDE（面试学习）、BUG_RECORD（踩坑记录）

---

## 📂 目录结构

```
RK3588-Omni-Sentinel/
├── README.md                          # 本文件
├── Software/
│   └── APP/
│       ├── CLAUDE.md                  # 编码规范 + 架构约定
│       ├── 3rdparty/                  # 第三方库 (librga, ffmpeg, opencv, rknpu2...)
│       ├── dma-buffer-pool/           # DMA 内存池（基础设施）
│       ├── sentinel-lslidarer/        # 激光雷达驱动
│       ├── sentinel-visioner/         # 相机视觉管线
│       ├── sentinel-yolo-infer/       # YOLOv8 RKNN NPU 推理
│       ├── lidar-camera-fusion/       # 视觉-雷达融合跟踪
│       ├── sentinel-streamer/         # RTSP 推流 + MP4 录像 + OSD
│       └── SentinelQT/                # Qt5 嵌入式触控 HMI
└── Hardware/                          # 硬件驱动层
    └── Driver/
```
