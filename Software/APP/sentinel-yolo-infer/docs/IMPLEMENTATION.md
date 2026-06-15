# Sentinel YOLO Infer — 技术实现文档

## 1. 概述

`sentinel-yolo-infer` 是 RK3588-Omni-Sentinel 项目中的 YOLOv8 RKNN 推理模块。它建立在 `sentinel-visioner` 的摄像头采集、RGA 转换和 DMA buffer 管理能力之上，实现从 NPU 小图到检测框结果的实时推理链路。

核心职责：

1. 基于 `camNum` 创建多路推理线程；
2. 每路线程独立加载 RKNN 模型并维护独立 RKNN context；
3. 调用 `sentinel-visioner` 的 `wait_get_npu / try_get_npu` 获取 NPU 小图；
4. 使用 `rknn_create_mem_from_fd` 导入 DMA-BUF，实现输入侧零拷贝；
5. 完成 YOLOv8 后处理，生成 `YoloBBox`；
6. 将检测框分别推入融合队列和 OSD 队列；
7. 推理完成后调用 `release_npu` 归还 DMA buffer。

---

## 2. 总体架构

```text
                   ┌────────────────────────────┐
                   │        V4L2 Camera          │
                   │  /dev/video21 / video11     │
                   └──────────────┬─────────────┘
                                  │
                                  ▼
                   ┌────────────────────────────┐
                   │      sentinel-visioner      │
                   │ V4L2 DQBUF + MJPG/ISP 输入  │
                   │ RGA → 640×640 RGB888 小图   │
                   │ push → npuTaskQueue         │
                   └──────────────┬─────────────┘
                                  │
                       try_get_npu / wait_get_npu
                                  │
                                  ▼
                   ┌────────────────────────────┐
                   │     SentinelYoloInfer       │
                   │  每 camNum 一个推理线程      │
                   └──────────────┬─────────────┘
                                  │
                   ┌──────────────┴─────────────┐
                   │                            │
                   ▼                            ▼
        ┌─────────────────────┐      ┌─────────────────────┐
        │ Yolov8RknnEngine     │      │ Yolov8RknnEngine     │
        │ camNum = 0           │      │ camNum = 1           │
        │ RKNN context 0       │      │ RKNN context 1       │
        └──────────┬──────────┘      └──────────┬──────────┘
                   │                            │
                   ▼                            ▼
             YoloBBoxList                 YoloBBoxList
                   │                            │
                   ├──────── fusionQueue ◀─────┤
                   └──────── osdQueue    ◀─────┘
```

---

## 3. 模块组成

| 文件 | 职责 |
|---|---|
| `include/SentinelYoloInfer.h` | 对外接口、配置结构、检测框结构、推理类声明 |
| `src/SentinelYoloInfer.cpp` | 多线程管理、NPU buffer 获取与释放、结果队列分发 |
| `src/Yolov8RknnEngine.h` | RKNN YOLOv8 引擎接口 |
| `src/Yolov8RknnEngine.cpp` | RKNN 初始化、zero-copy 输入、输出收集、YOLOv8 后处理 |
| `src/demo_yolo_infer.cpp` | 单路摄像头推理 Demo |
| `src/demo_yolo_infer_dual.cpp` | 双路 USB + ISP 推理 Demo |
| `docs/DEMO-INSTRUCTIONS.md` | 模型生成、部署、测试和基准记录 |
| `BUG_RECORD.md` | 问题记录 |
| `README.md` | 对外展示和快速上手 |

---

## 4. 核心数据结构

### 4.1 `YoloBBox`

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
```

语义说明：

| 字段 | 含义 |
|---|---|
| `x1, y1` | 左上角像素坐标，包含 |
| `x2, y2` | 右下角像素坐标，不包含 |
| `classId` | COCO 类别 ID |
| `confidence` | 置信度 |
| `timestampNs` | 当前检测框对应的图像帧时间戳，CLOCK_MONOTONIC ns |

当前坐标系为 640×640 NPU 小图坐标系，不是原始相机 1920×1080 或 640×480 坐标系。若后续 OSD 要画到原始画面，需要进行 letterbox 逆变换。

### 4.2 `SentinelYoloInferConfig`

```cpp
struct SentinelYoloInferConfig {
    std::string modelPath;
    float boxThreshold = 0.25f;
    float nmsThreshold = 0.45f;
    int waitTimeoutMs = 200;
    bool pushEmptyResult = true;
};
```

重点：

- `modelPath` 必须指向 RK3588 int8 `.rknn` 模型；
- `boxThreshold` 可用于调试检测灵敏度；
- `pushEmptyResult=true` 时，即使无目标也推送空 `vector`，便于 OSD 保持帧同步；
- `waitTimeoutMs>0` 时使用 `try_get_npu`，便于线程检查退出标志。

### 4.3 `InferThreadContext`

每路摄像头对应一个内部上下文：

```cpp
struct InferThreadContext {
    int camNum;
    std::atomic<bool> running;
    std::thread worker;
    std::unique_ptr<Yolov8RknnEngine> engine;
    ThreadSafeQueue<YoloBBoxList> fusionQueue;
    ThreadSafeQueue<YoloBBoxList> osdQueue;
};
```

设计目的：

- `camNum` 区分不同摄像头；
- `engine` 独立持有 RKNN context；
- `fusionQueue` 和 `osdQueue` 分离，避免两个消费者抢同一份结果；
- `running` 控制线程退出。

---

## 5. 推理线程生命周期

### 5.1 创建线程

```cpp
bool SentinelYoloInfer::create_infer_thread(int camNum);
```

流程：

```text
检查 visioner_ 是否为空
检查 modelPath 是否为空
查找该 camNum 是否已有运行线程
创建 InferThreadContext
创建 Yolov8RknnEngine
engine->init(modelPath, boxThreshold, nmsThreshold)
启动 std::thread
写入 contexts_
```

成功日志：

```text
[Yolov8RknnEngine] model input: 640x640x3, outputs=9, quant=true
[SentinelYoloInfer] infer thread started, camNum=0
```

### 5.2 推理循环

核心逻辑：

```text
while running:
    从 sentinel-visioner 获取 NPU 小图
    如果超时，continue
    使用 NpuBufferGuard 托管 buffer
    调用 Yolov8RknnEngine::inferFromDmaBuffer
    得到 YoloBBoxList
    push 到 fusionQueue
    push 到 osdQueue
    作用域结束，自动 release_npu
```

关键点：`release_npu` 必须在推理完成后调用，否则 `sentinel-visioner` 的 `npuRgbPool` 会被耗尽。

### 5.3 停止线程

```cpp
void SentinelYoloInfer::stop_infer_thread(int camNum);
void SentinelYoloInfer::stop_all();
```

流程：

1. 从 `contexts_` 中移除上下文；
2. 设置 `running=false`；
3. `join()` 工作线程；
4. 调用 `engine->release()`；
5. 销毁 RKNN context 与 output mem。

---

## 6. DMA buffer 生命周期

这是整个模块最容易出错的部分。

正确生命周期如下：

```text
sentinel-visioner:
    targetNpuBuf = npuRgbPool->get_buffer()
    RGA 写入 640×640 RGB888
    npuTaskQueue.push(targetNpuBuf)

sentinel-yolo-infer:
    npuBuf = try_get_npu(camNum)
    rknn_create_mem_from_fd(npuBuf->dmaFd, ...)
    rknn_set_io_mem(...)
    rknn_run(...)
    post_process(...)
    release_npu(camNum, npuBuf)
```

错误做法：

```cpp
if (npuOk) {
    ctx->npuRgbPool->release_buffer(targetNpuBuf);
}
```

这会导致 `try_get_npu()` 永远拿不到 NPU 小图。

最终必要修改：

```cpp
if (npuOk) {
    ctx->npuTaskQueue.push(targetNpuBuf);
} else {
    ctx->npuRgbPool->release_buffer(targetNpuBuf);
}
```

---

## 7. RKNN 引擎实现

### 7.1 初始化

`Yolov8RknnEngine::init` 负责：

1. 读取 `.rknn` 文件；
2. 调用 `rknn_init`；
3. 查询输入输出数量；
4. 查询 input/output tensor attr；
5. 判断模型尺寸和量化状态；
6. 创建输出 tensor memory。

关键校验：

```text
model input: 640x640x3
outputs=9
quant=true
```

当前代码只支持 int8 量化模型。如果 `quant=false`，说明模型生成错误。

### 7.2 输入侧零拷贝

核心接口：

```cpp
bool Yolov8RknnEngine::inferFromDmaBuffer(
    int dmaFd,
    void* virtAddr,
    int bufferSize,
    int width,
    int height,
    uint64_t timestampNs,
    std::vector<YoloBBox>& out
);
```

逻辑：

```text
检查 width/height 是否匹配模型输入
sync dma buf
rknn_create_mem_from_fd
rknn_set_io_mem
rknn_run
collect outputs
postProcess
destroy input mem
```

输入图像来自 `sentinel-visioner`，应为：

```text
640×640 RGB888 UINT8
```

模型由 `rknn_model_zoo` 生成时配置了归一化参数，Runtime 会按模型配置处理输入。

### 7.3 输出与后处理

当前适配 Rockchip 官方 YOLOv8 优化版输出：

```text
outputs = 9
3 个分支
每个分支包含 box / score / score_sum
```

后处理主要步骤：

1. 反量化 int8 输出；
2. DFL 解码 bbox；
3. 按 `boxThreshold` 筛选候选框；
4. 按类别执行 NMS；
5. 坐标裁剪到 640×640；
6. 写入 `YoloBBox`。

---

## 8. 队列设计

同一帧推理结果会复制到两个队列：

```cpp
ctx->fusionQueue.push(boxes);
ctx->osdQueue.push(boxes);
```

设计原因：

| 队列 | 消费者 | 设计目的 |
|---|---|---|
| `fusionQueue` | 目标融合 / 多传感器融合 | 用于业务算法 |
| `osdQueue` | OSD 绘制 / UI 显示 | 用于画面叠加 |

若两个消费者共用同一个队列，先取走结果的消费者会导致另一个消费者拿不到该帧结果，因此必须分离。

---

## 9. 双路推理实现

双路 Demo 的结构：

```text
camNum=0 → /dev/video21 → USB_CAM → infer thread 0
camNum=1 → /dev/video11 → ISP_CAM → infer thread 1
```

主程序执行：

```cpp
visioner.add_camera("/dev/video21", 640, 480, 6, 0, CameraType::USB_CAM);
visioner.add_camera("/dev/video11", 1920, 1080, 6, 1, CameraType::ISP_CAM);

visioner.camera_stream_ctrl(0, true);
visioner.camera_stream_ctrl(1, true);

infer.create_infer_thread(0);
infer.create_infer_thread(1);
```

读取结果：

```cpp
infer.try_get_osd_result(0, usbBoxes, 10);
infer.try_get_osd_result(1, ispBoxes, 10);
```

实测结果显示双线程均可持续输出：

```text
[USB] cam=0 boxes=...
[ISP] cam=1 boxes=...
```

---

## 10. 与 resources 的关系

本模块不保存大模型文件，模型统一存放在：

```text
APP/resources/models/yolov8n.rknn
```

该模型由 Rockchip 官方 `rknn_model_zoo` 生成，转换参数为：

```text
platform = rk3588
quant = i8
input = yolov8n.onnx
output = yolov8n.rknn
```

板端部署时复制到：

```text
/userdata/sentinel-yolo-infer-test/models/yolov8n.rknn
```

这样可以避免把模型权重混入推理代码仓库，也方便后续替换不同版本模型。

---

## 11. 当前已验证状态

| 能力 | 状态 |
|---|---|
| RKNN 模型生成 | 通过 |
| USB 单路推理 | 通过 |
| ISP 单路推理 | 通过 |
| 双路推理线程 | 通过 |
| DMA buffer 自动释放 | 通过 |
| fusion / osd 双队列分发 | 通过 |
| 实际检测框输出 | 通过 |
| preview 队列消费 | 未覆盖，非当前模块职责 |

---

## 12. 已知限制

1. 当前检测框坐标仍处于 640×640 NPU 小图坐标系；
2. Demo 未消费 preview 队列，会产生 `preview pool empty`；
3. 尚未做 10 分钟以上长稳测试；
4. 尚未实现自动设备节点发现；
5. 当前后处理只适配 Rockchip 官方优化版 YOLOv8 Detect，不适配 YOLOv8 Seg/Pose/Cls 或普通 Ultralytics 单输出 ONNX。
