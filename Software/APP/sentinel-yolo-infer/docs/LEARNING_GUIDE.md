# Sentinel YOLO Infer — 学习指南

## 目标

本文面向项目其他成员，用于快速理解 `sentinel-yolo-infer` 是什么、为什么要这样写、如何学习、如何测试，以及遇到问题时怎么定位。

读完后应能回答：

1. 推理类在整个系统中负责什么；
2. 为什么不能直接用官方 YOLOv8 demo；
3. 什么是 RKNN 模型，为什么要用 `rknn_model_zoo` 生成；
4. NPU 小图 DMA buffer 是如何流转的；
5. 多摄像头多推理线程如何实现；
6. 当前测试结果说明了什么。

---

## 第一层：先能说清“这个模块做了什么”

### 一句话概括

> `sentinel-yolo-infer` 是 RK3588 上的 YOLOv8 RKNN 推理封装模块，它从 `sentinel-visioner` 获取 640×640 RGB888 NPU 小图 DMA buffer，基于 RKNN Runtime 完成 YOLOv8 推理，并把检测框结果分别输出给融合模块和 OSD 模块。

### 和官方 YOLOv8 demo 的区别

Rockchip 官方 YOLOv8 demo 通常是：

```text
读取图片 / 单帧
→ 输入 RKNN
→ 得到检测框
→ 打印或画图
```

本工程需要的是：

```text
多路摄像头实时输入
→ sentinel-visioner 生成 NPU 小图
→ 每路摄像头一个推理线程
→ 零拷贝 RKNN 推理
→ 结果进入 fusionQueue 和 osdQueue
```

所以必须封装成工程类，而不是直接把官方 demo 当作主程序使用。

---

## 第二层：理解整体数据流

### 核心流程图

```text
/dev/video21 或 /dev/video11
        ↓
sentinel-visioner 采集线程
        ↓
MJPG/ISP 图像处理
        ↓
RGA 转成 640×640 RGB888
        ↓
npuTaskQueue
        ↓
SentinelYoloInfer::create_infer_thread(camNum)
        ↓
Yolov8RknnEngine::inferFromDmaBuffer()
        ↓
YOLOv8 RKNN 输出
        ↓
YoloBBoxList
        ↓
fusionQueue + osdQueue
```

### 最关键的一句话

`sentinel-visioner` 是图像生产者，`sentinel-yolo-infer` 是 NPU 小图消费者。

生产者必须：

```cpp
ctx->npuTaskQueue.push(targetNpuBuf);
```

消费者用完必须：

```cpp
visioner_->release_npu(camNum, npuBuf);
```

---

## 第三层：理解核心类

### 1. `SentinelYoloInfer`

它负责线程管理和结果队列：

```cpp
SentinelYoloInfer infer(&visioner, cfg);
infer.create_infer_thread(0);
infer.create_infer_thread(1);
```

每个 `camNum` 都有自己的：

```text
推理线程
RKNN context
fusionQueue
osdQueue
```

### 2. `Yolov8RknnEngine`

它负责真正的 RKNN 调用：

```text
rknn_init
rknn_query
rknn_create_mem_from_fd
rknn_set_io_mem
rknn_run
postProcess
```

学习这个类时重点看三件事：

1. 模型初始化是否成功；
2. DMA-BUF 是否成功导入 RKNN；
3. 输出后处理是否匹配模型结构。

### 3. `YoloBBox`

它是最终输出：

```cpp
struct YoloBBox {
    uint32_t x1, y1, x2, y2;
    uint32_t classId;
    float confidence;
    uint64_t timestampNs;
};
```

后续融合和 OSD 都围绕这个结构展开。

---

## 第四层：为什么模型不能随便拿

当前 C++ 后处理不是适配所有 YOLOv8 模型，而是适配 Rockchip 官方 YOLOv8 优化版输出结构。

正确模型应满足：

```text
RK3588
YOLOv8 Detect
输入 640×640×3
outputs=9
quant=true
int8
```

初始化时必须看到：

```text
[Yolov8RknnEngine] model input: 640x640x3, outputs=9, quant=true
```

如果看到：

```text
outputs=1
quant=false
```

大概率就是模型不匹配。

---

## 第五层：学习路线

### Step 1：先看 `sentinel-visioner`

必须先理解：

| 内容 | 重点 |
|---|---|
| V4L2 | 摄像头如何出帧 |
| RGA | 如何转成 640×640 RGB888 |
| DmaBufferPool | DMA buffer 如何复用 |
| ThreadSafeQueue | 生产者消费者如何通信 |
| `wait_get_npu` | 推理模块如何拿图 |
| `release_npu` | 推理完成后如何归还 buffer |

重点文件：

```text
sentinel-visioner/include/sentinel-visioner.h
sentinel-visioner/src/sentinel-visioner.cpp
sentinel-visioner/include/ThreadSafeQueue.h
```

### Step 2：再看推理类接口

重点文件：

```text
sentinel-yolo-infer/include/SentinelYoloInfer.h
```

重点理解：

```cpp
create_infer_thread(camNum)
try_get_fusion_result(camNum, ...)
try_get_osd_result(camNum, ...)
stop_infer_thread(camNum)
```

### Step 3：看推理线程实现

重点文件：

```text
sentinel-yolo-infer/src/SentinelYoloInfer.cpp
```

看懂：

1. `InferThreadContext`；
2. `NpuBufferGuard`；
3. `infer_thread_loop_`；
4. 为什么结果要 push 两个队列。

### Step 4：看 RKNN 引擎

重点文件：

```text
sentinel-yolo-infer/src/Yolov8RknnEngine.cpp
```

按顺序看：

1. `init`；
2. `queryModelInfo_`；
3. `inferFromDmaBuffer`；
4. `postProcess_`。

不需要一开始就完全看懂 DFL 和 NMS，先知道它们属于 YOLOv8 后处理即可。

### Step 5：跑 Demo

先单路，再双路：

```bash
./sentinel_yolo_infer_demo ./models/yolov8n.rknn /dev/video21 0 USB 640 480 6

./sentinel_yolo_infer_demo ./models/yolov8n.rknn /dev/video11 1 ISP 1920 1080 6

./sentinel_yolo_infer_dual_demo ./models/yolov8n.rknn /dev/video21 /dev/video11 640 480 1920 1080 6
```

---

## 第六层：面试/汇报时怎么讲

### 项目贡献说法

> 我负责实现 RK3588 上的 YOLOv8 推理模块，将 Rockchip 官方 RKNN YOLOv8 示例改造成了可接入现有视觉流水线的 C++ 推理类。该模块支持按摄像头编号创建多路推理线程，每路线程独立持有 RKNN context，从 `sentinel-visioner` 获取 NPU DMA 小图进行零拷贝推理，并将检测结果分别推送给融合队列和 OSD 队列。最终在 RK3588 板端完成了 USB + ISP 双摄像头双推理线程测试。

### 技术亮点说法

1. **零拷贝输入**：使用 DMA-BUF fd 导入 RKNN input tensor，避免 CPU memcpy；
2. **多线程扩展**：每个 `camNum` 对应独立推理线程和 RKNN context；
3. **队列解耦**：融合和 OSD 使用两个结果队列，互不抢数据；
4. **生命周期安全**：使用 RAII guard 确保 NPU buffer 一定释放；
5. **模型匹配**：基于 Rockchip `rknn_model_zoo` 生成 `rk3588 i8` 模型。

### 踩坑说法

> 初始测试时推理线程一直没有结果，后来定位到 `sentinel-visioner` 在 NPU 小图生成成功后直接 release 了 buffer，没有 push 到 `npuTaskQueue`，导致推理模块拿不到图。修复为 `ctx->npuTaskQueue.push(targetNpuBuf)` 后，单路和双路推理均正常输出结果。

---

## 第七层：常见问题速查

### Q1：为什么 `cam=0 boxes=0` 也算正常？

因为这说明推理线程已经正常运行，只是当前帧没有检测到满足阈值的目标。真正失败通常是 `rknn_init failed`、`try_get_npu timeout` 或程序崩溃。

### Q2：为什么会有 `preview pool empty`？

因为当前 YOLO demo 没有消费 preview 队列，preview buffer 池被占满。它影响预览链路，不影响当前推理链路。

### Q3：为什么要用 `rknn_model_zoo`？

因为当前 C++ 后处理适配 Rockchip 官方优化版 YOLOv8 输出结构，需要 `outputs=9` 和 `quant=true`。

### Q4：为什么双路推理要两个 RKNN context？

每路摄像头帧流独立，推理线程独立。使用独立 context 避免跨线程共享 RKNN context 的线程安全问题，也方便后续按摄像头管理资源。

### Q5：检测框坐标为什么最大是 640？

因为检测结果当前处于 NPU 小图坐标系，而不是原始图像坐标系。画回原图需要做 letterbox 逆变换。

---

## 第八层：推荐学习资料

1. `sentinel-visioner/docs/LEARNING_GUIDE.md`
2. `sentinel-yolo-infer/docs/IMPLEMENTATION.md`
3. Rockchip `rknn_model_zoo/examples/yolov8`
4. RKNN Runtime API：`rknn_init`、`rknn_query`、`rknn_create_mem_from_fd`
5. YOLOv8 后处理：DFL、NMS、置信度筛选
6. Linux DMA-BUF 基础概念
7. V4L2 + RGA 图像流基础

---

## 第九层：学习验收

读完本模块后，成员应能独立完成：

- [ ] 解释 `SentinelYoloInfer` 在系统中的位置；
- [ ] 说明为什么 NPU buffer 必须由 `sentinel-yolo-infer` 释放；
- [ ] 说明 `fusionQueue` 和 `osdQueue` 为什么要分开；
- [ ] 生成 `rk3588 i8 yolov8n.rknn`；
- [ ] 在板端部署并运行单路 demo；
- [ ] 在板端运行 USB + ISP 双路 demo；
- [ ] 根据日志判断是模型问题、摄像头问题、动态库问题还是队列问题。
