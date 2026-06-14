# sentinel-yolo-infer

`sentinel-yolo-infer` 是 RK3588-Omni-Sentinel 平台上的 YOLOv8 RKNN 推理模块。它不是重新实现 YOLO 算法，而是把 Rockchip 官方 YOLOv8 RKNN 推理流程封装成可接入 `sentinel-visioner` 的工程化 C++ 推理类。

模块负责从 `sentinel-visioner` 获取每路摄像头对应的 640×640 RGB888 NPU 小图 DMA buffer，基于 RKNN Runtime 进行零拷贝 YOLOv8 推理，并将检测结果分别推送到两个结果队列：

- `fusionQueue`：供融合模块消费；
- `osdQueue`：供画面叠加 OSD / 绘制检测框消费。

---

## 1. 项目定位

在整体系统中，本模块位于 `sentinel-visioner` 与上层融合/显示业务之间：

```text
V4L2 摄像头
    ↓
sentinel-visioner
    ↓  wait_get_npu / try_get_npu
640×640 RGB888 NPU 小图 DMA buffer
    ↓
sentinel-yolo-infer
    ↓  RKNN YOLOv8 推理
YoloBBox 检测框
    ↓
    ├── fusionQueue  → 融合模块
    └── osdQueue     → OSD / 画框模块
```

它主要解决以下工程问题：

1. 多摄像头场景下按 `camNum` 创建独立推理线程；
2. 每路推理线程拥有独立 RKNN context；
3. 从 `sentinel-visioner` 获取 NPU 小图并在推理完成后释放；
4. 基于 `dmaFd` 导入 RKNN input tensor，避免 CPU 图像拷贝；
5. 将同一帧检测结果同时分发给融合队列和 OSD 队列；
6. 为单路与双路摄像头提供可运行 Demo。

---

## 2. 目录结构

```text
APP/
├── 3rdparty/
│   ├── rknpu2/
│   │   ├── include/rknn_api.h
│   │   └── lib/librknnrt.so
│   ├── ffmpeg/
│   └── librga/
├── resources/
│   └── models/
│       └── yolov8n.rknn
├── sentinel-visioner/
└── sentinel-yolo-infer/
    ├── CMakeLists.txt
    ├── build.sh
    ├── README.md
    ├── BUG_RECORD.md
    ├── include/
    │   └── SentinelYoloInfer.h
    ├── src/
    │   ├── SentinelYoloInfer.cpp
    │   ├── Yolov8RknnEngine.cpp
    │   ├── Yolov8RknnEngine.h
    │   ├── demo_yolo_infer.cpp
    │   └── demo_yolo_infer_dual.cpp
    └── docs/
        ├── DEMO-INSTRUCTIONS.md
        ├── IMPLEMENTATION.md
        ├── LEARNING_GUIDE.md
        └── TODO.md
```

---

## 3. 核心接口

### 3.1 检测框结构

```cpp
struct YoloBBox {
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
    uint32_t classId;
    float confidence;
    uint64_t timestampNs;
};

using YoloBBoxList = std::vector<YoloBBox>;
```

坐标位于 `sentinel-visioner` 输出的 NPU 小图坐标系中，当前为 640×640 RGB888 letterbox 图像坐标系。

### 3.2 推理类典型用法

```cpp
SentinelVisioner visioner;

visioner.add_camera("/dev/video21", 640, 480, 6, 0, CameraType::USB_CAM);
visioner.camera_stream_ctrl(0, true);

SentinelYoloInferConfig cfg;
cfg.modelPath = "./models/yolov8n.rknn";
cfg.boxThreshold = 0.25f;
cfg.nmsThreshold = 0.45f;
cfg.waitTimeoutMs = 200;
cfg.pushEmptyResult = true;

SentinelYoloInfer infer(&visioner, cfg);
infer.create_infer_thread(0);

YoloBBoxList osdBoxes;
if (infer.try_get_osd_result(0, osdBoxes, 100)) {
    // draw boxes
}

YoloBBoxList fusionBoxes;
if (infer.try_get_fusion_result(0, fusionBoxes, 100)) {
    // fusion
}
```

---

## 4. 模型要求

当前推理代码适配的是 Rockchip 官方 `rknn_model_zoo/examples/yolov8` 的 YOLOv8 Detect 优化版模型，要求如下：

| 项目 | 要求 |
|---|---|
| 平台 | RK3588 |
| 模型格式 | `.rknn` |
| 模型类型 | YOLOv8 Detect |
| 输入尺寸 | 640×640×3 |
| 输入格式 | RGB888 / UINT8 |
| 输出结构 | 9 outputs，3 个尺度分支，每个分支包含 box / score / score_sum |
| 量化类型 | int8 |
| 典型日志 | `model input: 640x640x3, outputs=9, quant=true` |

当前测试模型由 Rockchip 官方 `rknn_model_zoo` 生成，存放在：

```text
APP/resources/models/yolov8n.rknn
```

板端测试部署路径为：

```text
/userdata/sentinel-yolo-infer-test/models/yolov8n.rknn
```

模型转换命令见 `docs/DEMO-INSTRUCTIONS.md`。

---

## 5. 编译方式

本工程与 `sentinel-visioner` 保持相同的交叉编译风格。典型目录结构如下：

```text
APP/
├── 3rdparty/rknpu2/include/rknn_api.h
├── 3rdparty/rknpu2/lib/librknnrt.so
├── sentinel-visioner/
└── sentinel-yolo-infer/
```

在虚拟机交叉编译环境中执行：

```bash
cd ~/RK3588-Omni-Sentinel/Software/APP/sentinel-yolo-infer
chmod +x build.sh
rm -rf build install
./build.sh
```

编译产物包括：

```text
build/sentinel_yolo_infer_demo
build/sentinel_yolo_infer_dual_demo
install/
```

注意：交叉编译生成的是 AArch64 程序，在 x86_64 虚拟机上不能直接运行，也不能使用本机 `ldd` 正确检查。可用：

```bash
file build/sentinel_yolo_infer_demo
readelf -d build/sentinel_yolo_infer_demo | grep NEEDED
```

真实运行测试必须在 RK3588 板子上执行。

---

## 6. 部署路径

推荐在 RK3588 板子上使用固定测试目录：

```text
/userdata/sentinel-yolo-infer-test/
├── sentinel_yolo_infer_demo
├── sentinel_yolo_infer_dual_demo
├── lib/
│   ├── librknnrt.so
│   ├── libavcodec.so.60
│   ├── libavutil.so.58
│   ├── libswresample.so.4
│   └── librga.so
├── models/
│   └── yolov8n.rknn
└── logs/
```

运行前需要设置：

```bash
export LD_LIBRARY_PATH=/userdata/sentinel-yolo-infer-test/lib:$LD_LIBRARY_PATH
```

---

## 7. 已验证结果

### 7.1 单路 USB 摄像头

```bash
./sentinel_yolo_infer_demo \
  ./models/yolov8n.rknn \
  /dev/video21 \
  0 \
  USB \
  640 \
  480 \
  6
```

验证结果：

- `/dev/video21` USB 摄像头打开成功；
- MJPG 640×480 解码成功；
- RKNN 模型加载成功；
- 推理线程启动成功；
- 持续输出 `cam=0 boxes=...`。

### 7.2 单路 ISP 摄像头

```bash
./sentinel_yolo_infer_demo \
  ./models/yolov8n.rknn \
  /dev/video11 \
  1 \
  ISP \
  1920 \
  1080 \
  6
```

验证结果：

- `/dev/video11` ISP mainpath 摄像头打开成功；
- RKNN 模型加载成功；
- 推理线程启动成功；
- 实测出现 `cam=1 boxes=1/2` 检测结果。

### 7.3 双路摄像头双推理线程

```bash
./sentinel_yolo_infer_dual_demo \
  ./models/yolov8n.rknn \
  /dev/video21 \
  /dev/video11 \
  640 \
  480 \
  1920 \
  1080 \
  6
```

验证结果：

- USB `/dev/video21` 配置为 `camNum=0`；
- ISP `/dev/video11` 配置为 `camNum=1`；
- 两路采集线程均启动；
- 两个 `Yolov8RknnEngine` 均初始化成功；
- 两个推理线程均启动；
- 日志持续输出 `[USB] cam=0 boxes=...` 和 `[ISP] cam=1 boxes=...`；
- USB 侧和 ISP 单路测试均出现实际检测框输出。

因此，当前版本已经完成单路与双路推理可行性验证。

---

## 8. 与 sentinel-visioner 的必要配合修改

测试过程中发现，如果 `sentinel-visioner` 在生成 NPU 小图后直接释放 `targetNpuBuf`，则 `sentinel-yolo-infer` 的 `try_get_npu()` 永远拿不到图，推理线程不会产生结果。

因此需要在 `sentinel-visioner/src/sentinel-visioner.cpp` 中将 NPU 成功分支改为：

```cpp
if (npuOk) {
    ctx->npuTaskQueue.push(targetNpuBuf);
} else {
    ctx->npuRgbPool->release_buffer(targetNpuBuf);
}
```

这项修改应当保留在 `sentinel-visioner` 中。否则本模块无法从 `wait_get_npu / try_get_npu` 获取 NPU 小图。

---

## 9. 已知非阻塞现象

当前 YOLO Demo 未消费 `sentinel-visioner` 的 preview 队列，因此运行时会出现：

```text
[Thread] Warning: preview pool empty! Dropping frame.
```

该现象说明 preview buffer pool 被占满，影响的是预览链路，不影响当前 YOLO 推理链路。若未来需要同时显示预览画面，应在 Demo 或上层业务中消费 preview 队列，或在 `sentinel-visioner` 增加关闭 preview 输出的开关。
