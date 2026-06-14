# TODO — sentinel-yolo-infer 后续优化方向

本文档记录当前 `sentinel-yolo-infer` 在可行性测试通过后的后续优化方向。当前单路 USB、单路 ISP、双路 USB+ISP 双推理线程均已跑通，以下内容不影响基础推理链路结论。

---

## 1. 消除 `preview pool empty` 警告

**现状**

当前 YOLO Demo 只消费 NPU 推理队列，不消费 preview 队列，因此运行中持续出现：

```text
[Thread] Warning: preview pool empty! Dropping frame.
```

**影响**

不影响 YOLO 推理链路，但会刷屏，且预览链路无法正常验证。

**优化方案**

方案 A：在 demo 中增加 preview 消费线程：

```text
try_get_preview(camNum, timeoutMs)
→ 不做显示，仅立即 release_preview
```

方案 B：在 `sentinel-visioner` 中增加开关：

```cpp
enablePreview = false;
```

YOLO-only 测试时只生成 NPU 小图，不生成 preview 图。

方案 C：真实系统中由 OSD/显示模块正常消费 preview 队列。

---

## 2. 检测框坐标从 640×640 映射回原始画面

**现状**

`YoloBBox` 当前坐标位于 NPU 小图坐标系：

```text
640×640 RGB888 letterbox
```

如果直接画到 1920×1080 或 640×480 原图上，会出现位置不匹配。

**优化方案**

记录 `sentinel-visioner` 生成 NPU 小图时的 letterbox 参数：

```text
scale
padX
padY
srcWidth
srcHeight
```

在 OSD 前做逆变换：

```text
x_orig = (x_npu - padX) / scale
y_orig = (y_npu - padY) / scale
```

---

## 3. 增加长稳测试脚本

**现状**

当前测试主要证明单路与双路可行，但尚未形成 10 分钟、30 分钟级别稳定性压测脚本。

**优化方案**

新增脚本：

```text
tools/run_single_usb_test.sh
tools/run_single_isp_test.sh
tools/run_dual_stability_test.sh
```

记录：

- 运行时间；
- `boxes` 输出频率；
- 是否崩溃；
- 是否死锁；
- RKNN 是否报错；
- 内存是否增长；
- fd 是否泄漏。

---

## 4. 增加设备节点自动发现

**现状**

当前测试命令硬编码：

```text
USB: /dev/video21
ISP: /dev/video11
```

不同板卡、重启顺序或外设插拔后，节点编号可能变化。

**优化方案**

解析：

```bash
v4l2-ctl --list-devices
```

或使用 Media Controller API，从设备名自动选择：

```text
USB 2.0 Camera → USB_CAM
rkisp_mainpath → ISP_CAM
```

---

## 5. 增加模型版本管理

**现状**

当前模型为：

```text
resources/models/yolov8n.rknn
```

仅通过文件名区分，不够明确。

**优化方案**

建议模型命名加入平台、量化类型和来源：

```text
yolov8n_rk3588_i8_rknn_model_zoo.rknn
```

同时记录一个模型说明文件：

```text
resources/models/MODEL_INFO.md
```

内容包括：

- ONNX 来源；
- rknn-toolkit2 版本；
- 转换命令；
- 量化数据集；
- 输入输出结构；
- 适配的 C++ 后处理版本。

---

## 6. 推理性能统计

**现状**

当前日志主要显示 RGA 耗时和 boxes 输出，没有单独统计 RKNN 推理耗时。

**优化方案**

在 `Yolov8RknnEngine::inferFromDmaBuffer` 中增加计时：

```text
rknn_set_io_mem 耗时
rknn_run 耗时
collect output 耗时
postprocess 耗时
总推理耗时
```

输出示例：

```text
[InferPerf] cam=0 rknn_run=8ms post=2ms total=11ms
```

---

## 7. 队列背压与丢帧策略

**现状**

如果推理线程处理慢于摄像头/RGA 生产速度，`npuTaskQueue` 可能积压，导致结果延迟变大。

**优化方案**

为 NPU 队列增加容量限制或“只保留最新帧”策略：

```text
队列满 → 丢弃旧帧
队列满 → 丢弃新帧
队列满 → 阻塞生产者
```

实际项目中推荐“只保留最新帧”，因为实时检测更关注最新画面而不是历史帧。

---

## 8. RKNN 多核调度与性能调优

**现状**

当前每路推理线程独立 RKNN context，但尚未明确配置 NPU core mask。

**优化方案**

如果 RKNN Runtime 支持，可尝试：

```text
cam0 → NPU core 0
cam1 → NPU core 1
```

或使用自动调度模式。需要实测双路 FPS 和延迟后决定。

---

## 9. 降低误检与阈值调参

**现状**

测试中 USB 侧出现 `cls=15`，ISP 侧出现 `cls=41/63/68/69` 等检测框，证明模型输出可用，但尚未评估误检率。

**优化方案**

增加配置项或命令行参数：

```text
boxThreshold
nmsThreshold
targetClasses
```

便于快速测试：

```bash
./sentinel_yolo_infer_demo model dev cam USB 640 480 6 --conf 0.35
```

---

## 10. 完善 Demo 退出和日志控制

**现状**

当前 Demo 使用 `Ctrl+C` 退出，日志中 RGA 和 preview warning 输出较多。

**优化方案**

- 增加运行时长参数；
- 增加日志等级；
- 增加只打印每 N 帧结果；
- 退出时打印统计摘要。

---

## 11. 支持更多模型

**现状**

当前仅适配 YOLOv8 Detect，且要求 Rockchip 官方优化版 9 输出 int8 模型。

**优化方向**

后续可扩展：

- YOLOv8s / YOLOv8m；
- YOLOv5；
- YOLOv8 Pose；
- YOLOv8 Seg；
- 自定义类别模型。

每种模型都应在 `MODEL_INFO.md` 中记录输出结构，并在 `Yolov8RknnEngine` 中明确对应后处理。

---

## 12. 对接真实 OSD 与融合模块

**现状**

当前 demo 只是打印队列输出。

**优化方向**

- OSD 模块消费 `osdQueue` 并将框画到 preview / 原始画面；
- 融合模块消费 `fusionQueue` 并做跨摄像头目标融合；
- 建立 result timestamp 与视频帧 timestamp 的同步策略。

---

## 当前优先级建议

| 优先级 | 任务 |
|---|---|
| P0 | 保留 `sentinel-visioner` 的 `npuTaskQueue.push` 修复 |
| P0 | 补充长稳测试 |
| P1 | 消费或关闭 preview 队列，消除 warning |
| P1 | 增加 RKNN 推理耗时统计 |
| P1 | 坐标映射回原图 |
| P2 | 自动发现摄像头节点 |
| P2 | 模型版本管理 |
| P3 | 多模型支持 |
