# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

RK3588-Omni-Sentinel 是一个基于瑞芯微 RK3588 的边缘端多传感器融合平台。所有应用层 C++ 组件均脱离 ROS，以静态库形式存在，通过 DMA-BUF 在 NPU/RGA/V4L2 硬件加速器之间实现零拷贝数据流转。

## 构建系统

本项目不是 ROS 工作空间。各组件独立编译，每个组件目录下有自己的 `build.sh` 和 `CMakeLists.txt`。全部使用 aarch64 交叉编译，目标平台为 RK3588 ARM64 Linux。

**编译环境**：交叉编译必须在虚拟机（Linux）中进行，Windows 开发机无法直接编译。代码在 Windows 上编写，通过共享目录或 `scp` 传到虚拟机后再执行 `build.sh`。

### 通用编译流程

每个组件目录下执行：

```bash
# 设置交叉编译器路径（按实际 SDK 路径修改 build.sh 中的 TOOL_CHAIN）
./build.sh

# 或手动执行:
mkdir -p build/ && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
```

产物输出到组件的 `install/` 目录。

### 交叉编译器

- `sentinel-lslidarer` / `sentinel-visioner` / `SentinelQT`: `aarch64-buildroot-linux-gnu`
- `dma-buffer-pool`: `aarch64-linux-gnu` (Linaro GCC 6.3.1)

环境变量 `CROSS_COMPILE_PATH` 可覆盖默认工具链路径（仅 `sentinel-lslidarer` 的 build.sh 支持此环境变量）。

### 编译单个组件

```bash
cd sentinel-visioner && ./build.sh     # 视觉管线
cd sentinel-lslidarer && ./build.sh    # 激光雷达驱动
cd dma-buffer-pool && ./build.sh       # DMA 内存池
cd SentinelQT && ./build.sh            # QT5 触控界面
```

注意 `sentinel-visioner` 通过相对路径 `../dma-buffer-pool` 和 `../3rdparty/librga` 引用依赖，需保持目录树结构不变。

## 架构

### 组件层级

```
sentinel-visioner (相机视觉管线 + RGA 硬件加速)
  ├── dma-buffer-pool (DMA 内存池，O(1) 空闲链表分配)
  │     └── 3rdparty/allocator (DMA/DRM 底层分配器，OBJECT 库)
  │           ├── 3rdparty/librga (RGA 2D 加速库)
  │           └── 3rdparty/libdrm (DRM 头文件 + .so)
  ├── 3rdparty/librga (直接依赖)
  ├── 3rdparty/ffmpeg (libavcodec/libavutil，MJPG 软件解码)
  ├── 3rdparty/opencv (OpenCV 3.4.5, 预编译 aarch64)
  └── 3rdparty/rknpu2 (RKNN NPU 运行时)

sentinel-yolo-infer (RKNN YOLOv8 NPU 推理)
  ├── sentinel-visioner (通过 wait_get_npu/try_get_npu 获取 640x640 RGB888 小图)
  ├── 3rdparty/rknpu2 (librknnrt.so, DMA-BUF fd 零拷贝导入 RKNN)
  ├── lidar-camera-fusion (复用 YoloBBox 类型定义)
  └── Threads::Threads + libdl (pthread + dlopen)

lidar-camera-fusion (视觉-雷达数据融合，纯算法组件)
  ├── sentinel-lslidarer/include (仅依赖 LidarPoint/LidarFrame 类型定义)
  └── DetectionProvider 回调解耦 (不直接依赖 sentinel-yolo-infer)

web-control (Web 远程控制组件)
  ├── cpp-httplib (单头文件 HTTP/WebSocket 库, MIT)
  ├── nlohmann/json (单头文件 JSON 库, MIT)
  └── SentinelQT (嵌入进程内, 通过 BlockingQueuedConnection 通信)

sentinel-lslidarer (激光雷达驱动，完全独立)
  └── Threads::Threads (唯一外部依赖)

sentinel-streamer (推流与录像 + OSD 叠加组件)
  ├── sentinel-visioner (依赖头文件 + 运行时调用 wait/get/release 接口)
  ├── dma-buffer-pool (720p 中间缩放缓冲池 + RecordBufferPool 录像帧环形缓冲)
  ├── 3rdparty/librga (1080p→720p NV12 硬件缩放 + rga_nv12_copy DMA 零拷贝)
  ├── 3rdparty/ffmpeg (libavcodec/libavformat/libavutil + ffmpeg CLI 子进程推流)
  └── StreamOsdProvider 回调 (推理结果 → NV12 CPU 绘制，不依赖 yolo-infer 头文件)

NVMe-SSD (NVMe 高速存储 + 视频回溯)
  ├── 原始块设备 O_DIRECT 写入 (Header+payload+512B对齐, 环形缓冲)
  ├── 3rdparty/ffmpeg (libavcodec/libavformat/libavutil/libswscale, MPP 硬件编码导出 MP4)
  ├── NVMeDataManager (生产者-消费者模式, 独立 writer 线程, 预分配内存池)
  └── RecordBufferPool 下游消费 (NV12 帧直存, 绕过 swscale 零开销编码)

SentinelQT (QT5 嵌入式触控界面)
  ├── sentinel-visioner (预览帧获取 + RGA 预处理)
  ├── sentinel-yolo-infer (NPU 推理实例管理，懒加载创建/销毁)
  ├── sentinel-streamer (推流/录像启停 + OSD provider 绑定 + RecordBufferPool 回溯源)
  ├── NVMe-SSD (NvmeWorker QThread 持续写入帧, 回溯导出 MP4)
  ├── web-control (嵌入进程内 HTTP/WebSocket 服务器, REST API 远程控制)
  ├── sentinel-lslidarer (激光雷达驱动, 融合页启用时启动)
  ├── lidar-camera-fusion (视觉-雷达融合 + 多目标跟踪, 含内部线程)
  ├── icm45686-eis-app-parameterized (IMU 电子防抖, 参数化 API, 回调注入 sentinel-visioner NPU 路径)
  ├── Qt5 Widgets (QStackedWidget 四页布局)
  └── config.ini (运行时配置, 含 [Lidar] [Fusion] [WebServer] [Backtrack] [EIS] [NVMe] 等)
```

### web-control — Web 远程控制组件

嵌入式 HTTP/WebSocket 服务器，在 SentinelQT 进程中运行，提供 REST API 远程操控板端设备，配套单文件 SPA 前端完全复刻 QT 界面风格。

- **WebServer**: 封装 cpp-httplib HTTP 服务器，独立 `std::thread` 运行 `listen()` 阻塞循环。注册 30+ REST 路由（含回溯、EIS 参数热更新）和 WebSocket 端点
- **线程安全模型**: REST 命令通过 `QMetaObject::invokeMethod(widget, lambda, Qt::BlockingQueuedConnection)` 同步调度到 Qt 主线程；WebSocket 推送使用 `std::queue` + `std::mutex` 消息队列（Qt 主线程非阻塞投递，广播线程消费发送）
- **MJPEG 快照**: 预览帧由 `on_frame_ready_()` 写入 `QImage` 缓存（mutex 保护），HTTP handler 在锁内完成 JPEG 编码后返回。不直接调 `try_get_preview()` 避免跨线程竞争 DMA 缓冲区
- **SPA 前端**: 单文件 `index.html`，仪表盘式单页布局。每路相机独立预览/推流/录像/暂停按钮 + 状态指示灯（录像时显示时长和分辨率）。推流视频通过 iframe 嵌入 MediaMTX WebRTC 播放器（端口 8889，延迟 <1s），录像文件支持在线播放（流式输出 + Range seek）。**五主题切换**：默认墨绿、深红+象牙白、皇家蓝+碧落、灰蓝+珍珠、紫罗兰+thistle，🎨 按钮切换，localStorage 持久化。按钮配色随主题联动（btn-on 用 accent 色，btn-off 用 bg2 色）
- **数据回溯面板**: 左列系统控制+数据回溯，右列录像管理。回溯/刷新按钮在标题右侧，相机选择和秒数输入并排。文件列表在各卡片下方，最多显示 30 条
- **融合跟踪**: Canvas 2D API 复刻 TopDownView 俯视图，WebSocket 推送 TrackedTarget 和 ClusterVisData，实时绘制距离网格（刻度自适应范围）、目标（蓝/绿/黄/红四色状态）、速度箭头、告警脉冲圈、DBSCAN 聚类半透明圆（YOLO认领绿/孤儿蓝）。未启动融合时显示"融合待启动"
- **暂停保护**: 暂停时自动停止推流/录像/预览；推流/录像启动时若相机已暂停则自动恢复，避免死锁
- **配置**: `config.ini` 中 `[WebServer]` 节（`port=8080`, `enabled=true`）
- **SPA 热加载**: HTML 从文件系统多路径搜索加载（`web/index.html`），编辑后无需重编译

唯一公共头文件: `include/web_server.h`，API 类: `WebServer`

### icm45686-eis-app-parameterized — ICM45686 电子防抖（参数化版本）

基于 ICM45686 SPI 内核驱动的用户态 EIS（电子防抖）组件。通过 `/dev/icm45686` 字符设备读取 IMU 数据，在用户态完成陀螺仪积分和像素偏移计算。通过回调注入模式集成到 SentinelQT 的 sentinel-visioner 管线。

**参数化架构**: 参数分为两层 — `ImuConfig`（IMU 全局配置：sampleHz、gyroRange/accelRange、零偏标定）和 `EisCameraConfig`（每路相机独立配置：focalX/Y、halfWindowMs、maxOffsetPixel、signX/Y、swapXY、timeOffsetMs、smoothingAlpha、enableSmoothing）。双相机共享同一 IMU Reader，但各用独立的 EisCameraConfig。

- **Icm45686Reader**: 后台 `std::thread` 以可配置频率读取 `/dev/icm45686`，推入 `ImuRingBuffer`（默认 512 样本）。新增 `configure(ImuConfig)` 统配接口（替代旧的 setAccelRange/setGyroRange 分散调用）、`start()` 无参启动（采样频率从 ImuConfig 取）、`calibrateGyroBias()` 静置零偏标定。`readSample()` 自动扣除已配置的陀螺零偏
- **EisStabilizer**: 梯形陀螺仪积分 → 像素偏移。核心 API `calculate_eis_offset(const EisCameraConfig& config, uint64_t frameTimestampNs, offsetX, offsetY)`，所有参数封装在 config 中，sign/smoothing/maxOffset 内置应用。旧签名 `(focalX, focalY, targetTs, halfWindowMs, ...)` 保留兼容。平滑按 camId 隔离状态，通过 `resetSmoothing(camId)` 清除残留
- **回调注入**: SentinelVisioner 加入 `set_eis_offset_callback(std::function)`，采集线程每帧调用回调获取偏移，传入 `rga_process_to_rgb_()` 的 `horizontalOffset/verticalOffset` 参数。**仅作用于 NPU 640×640 RGB888 推理路径**。visioner 侧 EMA 平滑已禁用（`set_eis_smooth_alpha(1.0f)`），平滑移至 Stabilizer 内部
- **Streamer EIS**: `rga_scale_nv12_to_720p()` 支持 EIS crop+scale，偏移通过 DmaBuffer_t 字段随帧传递。裁切边距 `set_eis_params()`（默认 32px，`config.ini` `streamerMargin`）
- **EIS 录制调试双输出**: sentinel-streamer 支持 `set_eis_record_debug(camNum, bool)` — 开启后 720p 录制同时输出两路 MP4：`xxx.mp4`（正常防抖）+ `xxx_raw.mp4`（无防抖对照）。第二路编码器和 MP4 muxer 在 `start_record()` 时创建，`stop_record()` 时清理。无防抖的 RGA scale 在 origBuf release 前完成，避免 dmaFd 复用竞态。`config.ini` `eisRecordDebug` 控制
- **两路隔离**: 两路相机共享 IMU Reader，各自独立 `eisEnabled_[camNum]` 和 `EisCameraConfig eisCamCfg_[2]`
- **Web 防抖控制卡片**: 网页端 `showEisControl=true` 时显示"防抖控制"卡片（位于系统控制下），CAM1/CAM2 标签切换，所有热修改参数可在线调整并保存。`config.ini` `showEisControl=false` 隐藏卡片。REST: `GET/POST /api/v1/eis/config`、`GET /api/v1/eis/visible`
- **配置**: `config.ini` `[EIS]` 节：`device`、`sampleHz`、`gyroRange/accelRange`、`enableGyroBiasCalib`/`biasCalibMs`、per-camera `FocalX/Y`/`AxisSignX/Y`/`SwapXY`/`TimeOffsetMs`/`FrameRate`/`HalfWindowMs`/`MaxOffsetPixel`/`EnableSmoothing`/`SmoothAlpha`、`streamerMargin`、`showEisControl`、`eisRecordDebug`
- **API**: REST `POST /api/v1/cam/{0,1}/eis/start|stop`、`GET /api/v1/eis/config`（查询参数）、`POST /api/v1/eis/config`（热更新）、`GET /api/v1/eis/visible`（卡片开关），WebSocket status 含 `eisEnabled`、`showEisControl` 字段
- **依赖**: POSIX + pthread + libm + `<functional>`，不依赖 ICM45686 头文件（sentinel-visioner 侧解耦）

构建: `icm45686_eis_lib` (STATIC, C++14, `src/imu_eis.cpp`) + `icm45686_user_lib` (STATIC, C99, `src/icm45686_user.c`)

### sentinel-lslidarer — 镭神 N10Plus 单线雷达驱动

三层结构：`SerialPort` → `RingBuffer(lock-free SWCR)` → `SentinelLslidarer`

- **SerialPort**: POSIX 串口（460800 baud, 8N1），阻塞读取，内存扫描 `0xA5 0x5A` 包头
- **RingBuffer**: SWCR 无锁环形缓冲区（`std::atomic` + memory_order 屏障），预分配 10 帧 × 540 点 ≈ 65KB
- **SentinelLslidarer**: 拥有 reader 线程，持续解码 N10Plus 108 字节固定包协议（双回波、CRC 校验），检测 360° 圈边界后提交完整帧。以 `CLOCK_MONOTONIC` 纳秒时间戳为索引，通过 `get_closest_frame(cameraTsNs, outFrame)` 线性扫描返回与相机时间戳最近的点云帧

唯一公共头文件: `include/sentinel_lslidarer.h`，API 类: `SentinelLslidarer`

### lidar-camera-fusion — 视觉-雷达数据融合 (v2 全局聚类)

纯算法组件，将 YOLO 2D 检测框与单线激光雷达点云进行融合，内置多目标跟踪器（`LidarTargetTracker`）。独立融合线程 `fusion_thread_()` 以 LiDAR 帧率（10Hz）驱动 DBSCAN 聚类 + Alpha-Beta 跟踪管线，融合循环仅在 LiDAR 新帧到达时执行。

- **DBSCAN 聚类** (`dbscan_cluster_`): 2D 笛卡尔空间聚类（O(n²)，540点可忽略），BFS 扩张。替代旧 CDC 角度链聚类。参数 `dbscanEpsMeters` / `dbscanMinPoints` / `maxPointDistanceMeters` / `maxClusterDistanceMeters`
- **时间证据累积** (`persist_clusters_`): DBSCAN 输出不直接变 detection，先做帧间 cluster 匹配（质心距离 < max(eps×2.5, 1.0m)）。连续出现 ≥ `clusterPersistenceFrames`(2) 帧才输出为正式 detection。槽位满时复用失效记录或淘汰最老记录
- **bbox 认领评分** (`bbox_claim_`): 对每个 confirmed 簇，投影质心到各相机，计算 `score = 1/(1+dist) * proximityFactor(pixelDist)`。**只匹配同相机 bbox**（通过 `bboxCamIdx` 数组区分来源），避免 cam0 簇被 cam1 bbox 误认领。每 bbox 选最高分簇，冲突时一个簇只能被一个 bbox 认领（赢家通吃+补选）。点数 < `minBboxClaimPoints`(10) 的簇不参与竞争。**深度优先仲裁**：得分最高簇若比最近候选簇远 ≥ 0.5m，且最近簇未被认领，则强制选最近簇
- **Alpha-Beta 跟踪器** (`LidarTargetTracker`): 预测 → 评分制贪心最近邻关联（纯距离评分 `1/(1+d²)`，无 bboxIdx/classId 门控，不同检测类型用不同硬门限）→ 校正 → 5 状态生命周期
- **5 状态生命周期** (`manage_lifecycle_`): Tentative(蓝) → FusionTracking(绿, YOLO+LiDAR) ⇄ PureRadarTracking(黄, 纯LiDAR) → Lost(红, 预测中) → Deleted。允许连续丢失 `maxFusionMisses`(2) 帧缓冲，单帧丢失不跳变
- **孤儿检测**: 未被 bbox 认领的确认簇，仅匹配非 Tentative 非 Deleted 航迹续命（门限 `orphanAssocMaxDistMeters=0.5m`），不创建新 track
- **纯雷达模式**: YOLO 无检测时全点归孤儿聚类，维持已有 track
- **LiDAR 帧去重**: `update()` 入口检查时间戳，相同则跳过；融合循环先取雷达帧再决定是否 drain YOLO
- **maxTracks 淘汰**: 槽位满时自动淘汰最老的 Lost track 腾出空间
- **OSD 快照**: `fuse_data()` 投影 LiDAR 点供推流画面显示（独立于 tracker 聚类），tracker 质心供距离标签
- **聚类可视化**: 通过 `copy_cluster_vis()` 导出 ClusterVisData（质心+半径+类型），WebSocket 推送至前端 Canvas 渲染半透明圆。orphan 判定通过 `clusterBboxMatch_` 索引精确匹配，避免距离阈值误判
- **外参变换**: 手写 4×4 齐次变换矩阵，针对 2D 激光雷达（z=0）优化
- **预分配内存**: 全栈数组预分配，零运行时堆分配

核心头文件: `include/lidar_camera_fusion.h`, `include/lidar_target_tracker.h`, `include/lidar_tracking_types.h`

关键调试日志: `[Fusion]` 每 5 帧输出 track 状态 F/P/L/T, `[BboxClaim]` 每帧输出 bbox 认领详情, `[TrackerTime]` 每帧输出 tracker 耗时

可调参数: 见 `config.ini [Fusion]` 节 (dbscanEpsMeters, alpha, bboxAssocMaxDistMeters, maxFusionMisses 等 24 项，全部热修改)

已知局限:
- 孤儿不创建新 track (需 YOLO 首次探测)
- 每 bbox 最多一个检测 (人数>框数时多余的人仅孤儿续命)
- YOLO 检测到但 LiDAR 看不到的目标无法跟踪 (单线雷达扫描面有限)
- OSD 投影与 tracker 聚类独立 (画面上有点 ≠ tracker 检测到)

详细文档: `lidar-camera-fusion/docs/FUSION_PIPELINE.md`, `lidar-camera-fusion/BUG_RECORD.md`

### sentinel-yolo-infer — RKNN YOLOv8 NPU 推理

基于哨兵视觉者（SentinelVisioner）的 640×640 RGB888 NPU 小图进行 YOLOv8 RKNN 推理。每路摄像头独立一个推理线程 + RKNN context，零拷贝 DMA-BUF fd 导入 NPU。

- **双队列输出**: 推理结果同时推入 `fusionQueue`（供融合）和 `osdQueue`（供推流 OSD 叠加），各自独立消费
- **DMA 零拷贝**: `rknn_create_mem_from_fd(dmaFd)` 直接导入 DMA-BUF，无 CPU memcpy
- **NpuBufferGuard (RAII)**: 作用域守卫确保异常路径也归还 DMA buffer
- **result provider 回调**: 通过 `SentinelYoloInfer` 提供 `try_get_fusion_result()` / `try_get_osd_result()` 供消费者轮询
- **模型**: `yolov8n.rknn`（INT8 量化，640×640×3 输入，9 输出分支），通过 `rknn_model_zoo` 转换生成
- **配置**: `SentinelYoloInferConfig` 含 modelPath、boxThreshold、nmsThreshold、waitTimeoutMs、pushEmptyResult

唯一公共头文件: `include/SentinelYoloInfer.h`，API 类: `SentinelYoloInfer`

### sentinel-visioner — 多路视觉流水线

双路"一分三"零拷贝扇出：每路 V4L2 输入（MIPI CSI 或 USB UVC），经 RGA 硬件裂变为三路独立数据流：
1. RGB888 640×640 NPU 推理小图（带 Letterbox 灰边 + EIS 防抖偏移）
2. RGB888 预览图像（供 QT 界面渲染，分辨率随相机 1080p/720p）
3. NV12 原始推流副本（同格式 RGA 拷贝）

支持两种相机类型混合接入，通过 `CameraType` 枚举区分（`ISP_CAM` / `USB_CAM`），两路通过 `camNum` 索引完全隔离：

- **CameraContext**: 每路相机状态。包含 5~7 个 DmaBufferPool：`npuRgbPool`(640×640)、`previewPool`(全分辨率 RGB888)、`origCopyPool`(全分辨率 NV12)；USB 额外 `usbConvertPool`(YUYV→NV12)、`usbSafePool`(NV12 安全拷贝，4 buffer)、`mjpegDecodePool`(MJPG→NV12 FFmpeg 软件解码输出，支持 1080p@30fps)
- **捕获线程**: epoll 监听 V4L2 `VIDIOC_DQBUF`，只传递 dmaFd。ISP 路径 MPLANE + NV12 直通；USB 路径单平面，自动协商 NV12/YUYV/MJPG：NV12 直通，YUYV 时 RGA 硬件转为 NV12（含 YVYU 变体适配），MJPG 时 V4L2 buffer mmap 读取 JPEG 数据 → FFmpeg 软件解码为 NV12 → 统一下游管线。USB NV12 先 `imcopy` 到 `usbSafePool`（缓解横向花屏问题，根因未确定）。所有 `improcess` 使用 `IM_SYNC` 同步模式。
- **消费者线程**: 通过 `wait_get_preview()` / `try_get_preview(camNum, timeoutMs)` / `wait_get_orig_copy_buffer()` 拉取，条件变量休眠（空闲 CPU 0.0%）。**必须调用对应的 `release_*()` 归还 DMA 缓冲区**
- **camera_pause**: `camera_pause(camNum, paused)` 可在不执行 STREAMOFF 的前提下暂停/恢复 RGA 处理，避免 RK3588 ISP 管线重建问题
- **ThreadSafeQueue**: 泛型阻塞队列模板（`include/ThreadSafeQueue.h`），`std::mutex` + `std::condition_variable`

已知问题：USB 相机 NV12 画面存在横向花屏（2026-05-27），详见 `BUG_RECORD.md` #7。

唯一公共头文件: `include/sentinel-visioner.h`，API 类: `SentinelVisioner`

### sentinel-streamer — 推流与录像 + OSD 叠加组件

作为 SentinelVisioner 的下游消费者，从 `processTaskQueue` 拉取 NV12 帧 → RGA 缩放为 720p → **YOLO 检测框 OSD 叠加（CPU NV12 绘制）** → MPP 硬件编码 H.264 → 双路输出。

- **双编码器架构**: `streamEncCtx`（720p, 推流）和 `recordEncCtx`（1080p/720p, 录像）各自独立，惰性创建，互不干扰
- **动态源分辨率**: RGA 缩放器 `rga_scale_nv12_to_720p(srcFd, srcWidth, srcHeight, dstFd)` 支持任意源分辨率（1080p→720p 下采样、720p→720p imcopy），不再硬编码 1080p
- **720p 源直通录像**: 源已是 720p 时录像直接从 `origBuf` 编码（绕过 RGA 缩放），避免 identity scale 失败导致空 MP4
- **ffmpeg 子进程推流**: `popen("ffmpeg -f h264 -i pipe:0 -c copy -rtsp_transport tcp -f rtsp ...")` 通过管道推流到 MediaMTX，TCP 传输避免 RTP 包过大；`ferror` 检测断线自动重连；子进程崩溃不影响主程序
- **PTS 硬件时间戳**: `(timestampUs - 首帧偏移) × 90000 / 1000000`，帧率波动不影响播放速度
- **MP4 录像**: FFmpeg API `av_write_frame` 写入 MP4，支持 1080p / 720p
- **线程安全**: 每路摄像头独立推流线程，编码器和输出上下文严格在 `workerThread.join()` 后销毁，杜绝 use-after-free
- **状态回调**: `StreamerCallback` 函数指针，通知上层启停/错误事件
- **反复启停**: 编码器每轮销毁重建，无 DTS 残留
- **OSD 叠加**: 通过 `StreamOsdProvider` 回调注入检测框，推流线程每帧非阻塞轮询（5ms 超时），仅叠加推流画面不影响录像。Y 平面 2px 白色边框 + 标签（3×5 点阵字体 2x 放大），640×640 NPU → 720p 坐标自动变换（含 letterbox 逆变换）。`set_osd_provider()` 绑定 provider，`set_stream_osd_mode()` 运行时切换
- **RecordBufferPool 环形缓冲**: 基于 DmaBufferPool + RGA DMA 拷贝的 NV12 帧环形缓冲区，在编码前暂存历史帧供数据回溯。每路独立，槽位数可配
- **rga_nv12_copy**: RGA IM2D `imcopy` 硬件 DMA 零拷贝，避免 CPU memcpy 开销
- **EIS 防抖推流/录像**: `rga_scale_nv12_to_720p()` 支持 EIS 参数（`eisOffsetX/Y`、`eisActive`、`eisMargin`），EIS 激活时在 1080p→720p 缩放中一次 RGA 完成 crop+scale 防抖。偏移量通过 DmaBuffer_t 字段随帧传递，无需重复计算
- **EIS 录制调试双输出**: `set_eis_record_debug(camNum, bool)` 开启后 720p 录制同时输出两路 MP4（正常防抖 + 无防抖对照 `_raw.mp4`）。第二路编码器/MP4 在 `start_record()` 创建、`stop_record()` 清理。无防抖 RGA scale 在 origBuf release 前完成。`config.ini` `[EIS]` `eisRecordDebug` 控制
- **录像 PTS 独立基准**: `recordBaseTsUs` 与推流 `baselineTsUs` 解耦，每次录像从首帧时间戳自动初始化，确保 PTS 从 0 开始
- **回溯公共 API**: `init_record_buffer(camNum, slotCount, width, height)` 初始化缓冲池，`try_get_record_frame(camNum, &data, &size, &ts)` 非阻塞 FIFO 消费帧，`release_record_frame(camNum, data)` 归还缓冲

唯一公共头文件: `include/sentinel_streamer.h`，API 类: `SentinelStreamer`

### dma-buffer-pool — DMA 内存池

- O(1) 空闲链表（Free List）分配/归还模型
- `DmaBuffer_t` 包含 dmaFd、virtAddr、timestampUs、eisOffsetX/Y、eisActive、链表 next 指针
- "花名册"（`allBuffers_` vector）确保析构时零泄漏回收
- 通过 `3rdparty/allocator/dma_alloc.h` 调用底层 `dma_buf_alloc()`（DMA heap ioctl + mmap）

### 3rdparty/ 第三方库存放目录

| 库 | 内容 |
|---|---|
| `librga` | 瑞芯微 RGA 2D 加速，提供 `librga.a` + IM2D API 头文件 |
| `rknpu2` | RKNN NPU 运行时，提供 `librknnrt.so` |
| `opencv` | OpenCV 3.4.5 预编译 aarch64 完整包 |
| `libdrm` | DRM 头文件 + `libdrm.so`（allocator 通过 dlopen 使用） |
| `allocator` | DMA/DRM 底层分配器，CMake OBJECT 库（非 .a） |
| `ffmpeg` | FFmpeg 动态库 (`libavcodec.so` 等，nyanmisaka/ffmpeg-rockchip 分支交叉编译) |

### camera-calibrate — 相机标定工具

独立命令行工具，基于原生 V4L2 API + OpenCV 实现棋盘格相机标定，支持 RK3588 ISP MPLANE 格式。

- **原生 V4L2 捕获**: 绕过 OpenCV VideoCapture（不支持 MPLANE），直接使用 V4L2 MMAP/USERPTR 实现 MPLANE 设备的帧抓取
- **NV12→BGR 转换**: CPU 端逐行去 stride padding 后 OpenCV 转换，兼容 RKISP stride≠width 的情况
- **标定参数**: `CALIB_FIX_ASPECT_RATIO | CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K2 | CALIB_FIX_K3`，固定 fx=fy，只用 k1 径向畸变，避免 CALIB_RATIONAL_MODEL 过拟合导致 fx 爆涨
- **自动采集模式**: 检测到棋盘格新位置自动捕获，带 5 帧冷却和 40px 去重阈值，`--save-frames` 可保存原图+角点标注图
- **NPU 内参输出**: 自动计算 NPU 640×640 letterbox 缩放后的 fx/fy/cx/cy，直接输出 config.ini 片段格式

用法: `./camera_calibrate /dev/video11 25 --save-frames ./frames`
构建: `cd camera-calibrate && ./build.sh`（依赖 `../3rdparty/opencv`）

### SentinelQT — QT5 嵌入式触控界面

RK3588 边缘端嵌入式触控人机交互界面（HMI），作为 SentinelVisioner、SentinelStreamer 和 SentinelYoloInfer 的上层集成者。

- **技术栈**: Qt5 Widgets，QStackedWidget 四页布局（主控页 / 视频管理页 / 融合管理页 / 数据回溯页），全屏无边框
- **共享标题栏**: `titleBar`（温度/CPU/RGA/NPU + 标题 + 时钟）位于根布局 QStackedWidget 上方，三页共享。hwLabel 和 clockLabel 均为 280px 确保标题居中
- **双路预览**: 左右并排 `previewLabel0` / `previewLabel1`，两个独立 PreviewWorker 各自运行在独立 QThread，通过 lambda 捕获 camNum 将 `frameReady` 信号路由到对应 label。每路预览可独立开启/关闭
- **双路控制**: 每路相机独立 6 按钮（预览切换、推流、录像、暂停、OSD、EIS），全局系统按钮一键启停两路
- **实时预览**: PreviewWorker 子线程通过 `try_get_preview(camNum, 200)` 拉取 RGB888 帧，DMA-BUF virtAddr 零拷贝 QImage → Qt::QueuedConnection 信号槽投递至主线程
- **推流控制**: 调用 SentinelStreamer API 启停 RTSP 推流，StreamerCallback → QMetaObject::invokeMethod 跨线程通知 UI。两路独立 RTSP URL
- **录像控制**: 按相机独立启停 MP4 录像，实时显示录制时长（QTimer 每秒更新），时间戳文件名。双路均支持 1080p/720p 录制分辨率（底部栏 per-camera QComboBox 选择），Web 远程切换双向同步
- **NPU 推理管理**: `SentinelYoloInfer` 实例懒加载，融合或 OSD 首次需要时创建，两路都关闭且融合未启用时自动销毁。融合和 OSD 各自绑定独立 provider 回调
- **OSD 叠加控制**: 每路相机独立 OSD 按钮（Qt + Web 双端同步），绑定 `StreamOsdProvider` 回调到 streamer，运行时切换 OSD 模式
- **视频管理**: QTableWidget 列表展示录制文件（libavformat 读分辨率/时长），支持删除
- **融合目标跟踪**: 第三页独立子页面，集成 SentinelLslidarer + LidarCameraFusion：
  - **俯视图**: `TopDownView` 自定义 QWidget，paintEvent 绘制距离网格、目标（按 TrackState 着色）、速度箭头、告警脉冲圈、中文图例
  - **参数面板**: 9 个跟踪器参数 + 每路相机 4 个内参，QLineEdit + QDoubleValidator/QIntValidator，每个参数带 `?` 帮助按钮（点击在对应 section 下方显示说明，4 秒自动隐藏）
  - **虚拟键盘**: `VirtualKeyboard` 4×4 数字键盘，默认隐藏，eventFilter 检测 FocusIn 自动弹出 / FocusOut 自动隐藏。`keyboardContainer` 位于根布局（QStackedWidget 同级），实现跨页面（融合页 + 回溯页）键盘共享
  - **融合启停**: `on_btn_fusion_toggle_()` 控制完整生命周期（lidar start → fusion start → FusionWorker 轮询），失败自动回滚
  - **参数热更新**: `editingFinished` 触发 `configure_tracker()` + `update_camera_intrinsics()` 实时推送，无需重启融合
  - **告警输出**: 三层输出 — 俯视图红色脉冲圈 + 状态栏告警计数 + 终端 stderr `[FusionWarning]` 日志
  - **NPU 推理**: 融合线程通过 `DetectionProvider` 回调从 `SentinelYoloInfer` 获取真实 YOLO 检测结果（替换假检测），仅保留 person (classId=0) 且置信度 ≥ 0.75 的检测框
  - **数据回溯**: 第四页独立子页面，通过 RecordBufferPool 实现历史帧查询：
    - **手动回溯**: 秒数输入框 + 相机选择器 + 回溯按钮，`on_btn_backtrack_()` 触发。NvmeWorker QThread 持续写入 NV12 帧 → NVMeDataManager → NVMe SSD。调用 `export_trigger_video_clip()` 扫描磁盘导出 MP4。"全部"模式两路各导出一个视频。Web 支持 POST 触发 + 播放 + 删除。存储格式 NV12 直存，导出时 memcpy 直送 MPP 零 swscale
    - **自动告警回溯**: 融合告警回调触发 `on_fusion_alert_backtrack_()`，根据告警时间戳和配置的回溯秒数终端打印回溯范围。双路相机同时回溯
    - **双向 Web↔Qt 同步**: dirty flag 机制 — 回溯参数/l融合参数/录分辨率通过 status JSON 推送同步，用户正在修改时暂停覆盖
- **硬件监控**: 标题栏实时显示温度（thermal_zone0）、CPU（/proc/stat）、RGA/NPU 逐核利用率（debugfs）及日期时间。紧凑格式（无 `%` 符号）
- **系统暂停**: `camera_pause(camNum, paused)` 暂停 RGA 处理，硬件流保持，避免 STREAMOFF 重建管线
- **线程模型**: 两个 PreviewWorker + 一个 FusionWorker 各自独立 QThread + std::atomic<bool>；LidarCameraFusion 内部 std::thread；主线程处理 UI 和定时器。FusionWorker 100ms 轮询 + 目标变化去重。析构逆序释放（FusionWorker → fusion → lidar → preview → visioner/streamer）
- **状态栏**: 底部自动合并显示两路相机状态（`CAM0: xxx | CAM1: xxx`），全局消息直接显示
- **Web 远程控制**: 嵌入 WebServer（cpp-httplib），提供 REST API + WebSocket 实时推送。浏览器打开 `http://<IP>:8080` 即可远程操控。推流视频通过 iframe 嵌入 MediaMTX WebRTC 播放器（端口 8889）
- **配置**: `config.ini` 分 `[Camera0]`/`[Camera1]`/`[Lidar]`/`[Fusion]`/`[Record]`/`[WebServer]`/`[Backtrack]`/`[EIS]` 八节，USB 相机支持 1080p（MJPG 模式）

关键文件: `widget.h/cpp/ui`（主界面）、`preview_worker.h/cpp`（预览线程）、`fusion_worker.h/cpp`（融合轮询线程）、`top_down_view.h/cpp`（俯视图组件）、`virtual_keyboard.h/cpp`（虚拟键盘）、`main.cpp`（入口）、`config.ini`（配置，`[Backtrack]` 节）、`build.sh`（构建脚本）

## 关键约定

- 所有组件 C++14 标准，非 ROS，零外部运行时依赖
- 交叉编译器指向 aarch64，在 x86 开发机上运行编译
- 设备名默认值：雷达 `/dev/sentinel_lidar`，ISP 相机 `/dev/video11`，USB 相机 `/dev/video21`（均支持命令行覆盖）
- DMA 缓冲区遵循严格的"获取-使用-归还"生命周期，未归还将导致内核态内存枯竭和丢帧
- 时间戳统一使用 `CLOCK_MONOTONIC`，不同组件间通过此时间域实现传感器融合对齐
- 各开发者在自己的分支上工作，勿交叉修改他人负责的组件
- 用户说"上传代码到远程仓库"时：git add 所有修改过的源码/文档文件，用中文写 commit message，commit 后不 push（用户自己手动 push）

## 编码规范

以下规范适用于本项目所有 C++ 组件。

### 命名

| 类别 | 约定 | 示例 |
|------|------|------|
| 文件名 | `snake_case` | `lidar_camera_fusion.h`, `demo_single.cpp` |
| 类 / 结构体 | `PascalCase` | `LidarCameraFusion`, `YoloBBox`, `FusionResult` |
| 公共方法 | `snake_case` | `fuse_data()`, `get_closest_frame()` |
| 私有方法 | `snake_case` + 尾部 `_` | `transform_point_()`, `reader_loop_()` |
| 私有成员变量 | 尾部 `_` | `config_`, `running_`, `head_` |
| 结构体成员 / 公共成员 | `camelCase` | `x1`, `y1`, `classId`, `dmaFd` |
| 编译期常量 | `k` + `PascalCase` | `kMaxLidarPoints`, `kPacketLength` |
| 宏 | `UPPER_SNAKE_CASE` | `LIDAR_CAMERA_FUSION_H` |
| 局部变量 / 参数 | `camelCase` | `srcWidth`, `nPoints`, `bestIdx` |
| 全局变量 | `g` + `PascalCase` | `gRunning`, `g_is_running` |

### 格式

- **缩进**: 4 空格，禁用 tab
- **行宽**: 建议 ≤ 100 字符
- **大括号**: K&R 风格（左括号与声明同行），函数体左括号同行
- **Include guard**: `#ifndef` / `#define` / `#endif`，尾部加 `// GUARD_NAME` 注释。不使用 `#pragma once`

### 注释

- **README / 文档**: 简体中文
- **公共 API Doxygen**: `/** @brief ... */` 块注释，中文为主
- **行内注释**: `//` 或 `///<`（成员变量），中文或英文均可
- **内部实现注释**: `//`，简明解释非显而易见的逻辑
- **不使用** `@file`、`@author`、`@date` 等占位标签

### 错误处理与日志

- 公共方法通过 `bool` 返回值表示成功/失败
- 不使用 C++ 异常（嵌入式目标平台约束）
- 不使用 `assert()` / `static_assert()`
- 诊断输出统一使用 `fprintf(stderr, "[ComponentName] ...")`，带组件名前缀
- 系统调用失败时附 `strerror(errno)` 信息

### 内存管理

- 优先预分配，避免运行时动态分配
- 使用 `new (std::nothrow)` + 显式空指针检查
- RAII 析构函数清理所有资源
- DMA 缓冲区遵循"获取-使用-归还"生命周期
- 非平凡类删除拷贝构造和拷贝赋值 (`= delete`)

### 线程安全

- 组件内部默认不使用锁，设计为单线程顺序调用
- 需要跨线程共享状态时，使用 `std::atomic` + 显式 `memory_order`
- 阻塞队列使用 `std::mutex` + `std::condition_variable`

### 构建

- C++14 标准，CMake ≥ 3.4.1
- 每个组件编译为静态库（`.a`），附带独立 demo 可执行文件
- 产物输出到组件 `install/` 目录
- 交叉编译目标: `aarch64-buildroot-linux-gnu`
- x86 本地编译仅用于 demo 测试，不产生安装包

## 文档规范

每个组件应包含以下四类文档，分层递进。所有文档使用简体中文。

### README.md — 项目概览

面向**使用者**（接入方 / 维护者），回答"这是什么、怎么用"。

结构：
- 功能概述（bullet list，一项一行）
- 构建与部署（交叉编译命令、依赖关系）
- 快速上手（最小可运行代码示例）
- 配置文件说明（如有）

写法：
- 极简，不展开实现细节
- 代码示例可直接复制运行
- 使用 emoji 标题（✨🛠️🚀📖📊⚠️）增强可读性

参考：`sentinel-visioner/README.md`

### docs/IMPLEMENTATION.md — 实现文档

面向**开发者**（接手维护 / 二次开发），回答"怎么实现的、架构是什么"。

结构：
- 架构总览（ASCII 框图，展示组件内部模块和数据流）
- 线程模型（每个线程的职责、生命周期、同步机制）
- 核心实现细节（关键数据流分步骤展开，带代码片段）
- 配置或文件格式说明

写法：
- 用 ASCII 框图代替文字描述模块关系
- 关键路径给出完整调用链
- 点出容易踩坑的细节（如跨线程回调、析构顺序）

参考：`sentinel-streamer/docs/IMPLEMENTATION.md`、`SentinelQT/docs/IMPLEMENTATION.md`

### BUG_RECORD.md — 问题记录

面向**全团队**（经验沉淀 / 面试准备），回答"踩过什么坑、怎么修的"。

格式（每条严格遵循）：
```markdown
## N. 简短标题

**现象**: 一句话描述用户/开发者看到什么

**原因**: 根因分析，说清为什么

**解决**: 修复方案，关键代码片段或操作步骤
```

写法：
- 编号递增，不重排
- 每条独立完整，不看上下文也能理解
- 修复方案精确到代码行或具体命令

参考：`sentinel-streamer/BUG_RECORD.md`、`SentinelQT/BUG_RECORD.md`

### docs/LEARNING_GUIDE.md — 学习指南

面向**面试 / 新人**（快速建立系统认知），回答"设计为什么这样做、亮点在哪里"。

结构（三层递进）：
1. **第一层：做了什么** — 一句话概括、架构图、关键代码（面试讲项目用）
2. **第二层：为什么这么设计** — 设计决策表格（方案对比 + 教训），每个决策包含"我们选了什么 / 为什么不用另一种 / 教训"
3. **第三层：能讲清 bug** — 选 3 个最有代表性的 bug，附 **面试话术**（中文口述版）
4. **怎么对着代码学** — 跟一遍数据流的关键函数入口表

写法：
- 面试导向，每个决策、每个 bug 都配上"面试话术"
- 设计决策用对比表格（我们的方案 vs 常见替代方案）
- 关键代码要精简（3-5 行），面试能张口就来
- 结尾附"重点函数入口"表格，方便新人定位代码

参考：`sentinel-streamer/docs/LEARNING_GUIDE.md`、`SentinelQT/docs/LEARNING_GUIDE.md`

### docs/DEMO-INSTRUCTIONS.md — 演示程序说明

面向**测试者 / 使用者**，回答"有哪些 Demo、怎么跑、预期结果是什么"。

结构：
- 环境准备与通用编译要求（硬件平台、内核版本、依赖库、设备节点）
- 每个 Demo 独立一节：
  - 源文件（精确到 `.cpp`）
  - 演示目标（验证什么功能）
  - 线程架构说明（启动哪些线程、各自职责）
  - 运行步骤（编译命令 + 板端命令行参数）
  - 预期输出（终端打印的关键日志示例）
- 末尾附新增 Demo 文档模板

写法：
- 板端命令直接给出可用参数和默认值
- 终端输出用代码块展示关键行（心跳日志、FPS、延迟数据）
- 环境依赖写在最前面统一说明，每个 Demo 不重复
- 无 Demo 的组件（如纯 UI）可不写此文档

参考：`sentinel-visioner/docs/DEMO-INSTRUCTIONS.md`、`sentinel-streamer/docs/DEMO-INSTRUCTIONS.md`

## 飞书文档

项目核心技术手册：https://my.feishu.cn/docx/BcgLdpq2bo4qz9xxGCNc2M2Ynog

### 文档结构

6 个 H1 章节（按组件），每章 9 个 H2 小节：

| 节号 | 节名 | 内容 |
|------|------|------|
| 1 | 组件概览 | 一句话概括 + 功能列表 + 在系统中的角色 |
| 2 | 架构设计 | ASCII 框图 + 模块关系 + 线程模型表格 |
| 3 | 核心数据流 | 一帧数据的完整生命周期，分步展开 |
| 4 | 关键代码示例 | 最小可运行代码（构造→启动→使用→停止→清理） |
| 5 | 设计决策剖析 | 3-5 个决策对比表格 + 面试话术 |
| 6 | 性能基准数据 | CPU/内存/延迟/FPS 表格 |
| 7 | Bug 故事与面试话术 | 精选 3 个最有代表性的 bug（现象/原因/解决/面试话术） |
| 8 | API 接口参考 | 公共方法表格 |
| 9 | 编译部署与系统集成 | 交叉编译命令 + 依赖 + 组件间关系 + 注意事项 callout |

### 更新文档注意事项

1. **认证**：写入操作需要 user 身份（`--as user`），需先完成飞书授权。若报 `need_user_authorization`，执行：
   ```bash
   lark-cli auth login --domain docs --no-wait --json
   # 将输出的 verification_url 发给用户，授权完成后执行：
   lark-cli auth login --device-code <device_code>
   ```
2. **读取文档**：优先使用局部读取策略
   ```bash
   # 先看目录
   lark-cli docs +fetch --api-version v2 --as user --doc "<url>" --scope outline --max-depth 2
   # 精读某章节
   lark-cli docs +fetch --api-version v2 --as user --doc "<url>" --scope section --start-block-id <标题id>
   ```
3. **更新策略**：精准编辑优于全量覆盖
   - 修改单个段落：`block_replace --block-id <id> --content "<p>新内容</p>"`
   - 追加新章节：`block_insert_after --block-id -1 --content "<h1>新章节</h1>..."`
   - 全文重写（当前 5 章全部重建时）：`overwrite --content @file.xml`
4. **格式**：使用 XML 格式，支持 callout/table/grid/code block 等富文本元素
   - 核心结论 → `<callout emoji="💡" background-color="light-blue" border-color="blue">`
   - 注意事项 → `<callout emoji="⚠️" background-color="light-red" border-color="red">`
   - 设计决策对比 → `<table>` + 表头 `background-color="light-gray"`
   - 代码示例 → `<pre lang="cpp" caption="说明">`
5. **风格统一**：每个 H2 下至少 1 个非纯文本 block；连续纯文本 ≤ 3 段；不同 H2 间用 `<hr/>` 分隔；开头用 callout front-load 结论
6. **内容来源**：每章 9 节内容从对应组件的 README.md、docs/IMPLEMENTATION.md、docs/LEARNING_GUIDE.md、BUG_RECORD.md 中提取精炼
7. **新增组件章节时**：严格按照上述 9 节结构创建 H1 章节，内容从组件文档中提取，缺少 LEARNING_GUIDE 的组件（如 sentinel-lslidarer）需从 README + CLAUDE.md 推导设计决策和 bug 故事
