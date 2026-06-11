# sentinel-yolo-infer

`sentinel-yolo-infer` 是基于 RK3588 / RKNN YOLOv8 的多摄像头推理模块，用来接入 `sentinel-visioner` 的 NPU 小图流水线。

它的职责不是重新实现 YOLO 算法，而是把官方 YOLOv8 RKNN 示例封装成工程可用的推理类：

```text
SentinelVisioner 摄像头采集/RGA处理
        ↓
wait_get_npu / try_get_npu 获取 640×640 RGB888 DMA-BUF 小图
        ↓
SentinelYoloInfer 零拷贝 RKNN 推理
        ↓
YoloBBoxList
        ├── fusionQueue：融合模块使用
        └── osdQueue：OSD 叠加模块使用
```

## 目录要求

推荐目录结构：

```text
workspace/
├── sentinel-visioner/
└── sentinel-yolo-infer/
```

`sentinel-visioner` 会被本工程的 `CMakeLists.txt` 自动作为兄弟目录加入：

```cmake
add_subdirectory(${SENTINEL_VISIONER_DIR} ...)
```

因此，在上面这种结构下，编译 `sentinel-yolo-infer` 时通常不需要再手动传 `-DSENTINEL_VISIONER_DIR=...`。

如果你的 `sentinel-visioner` 不在兄弟目录，才需要：

```bash
./build.sh -DSENTINEL_VISIONER_DIR=/path/to/sentinel-visioner
```

## RKNN Runtime 说明

`rknpu2/` 这个目录名不是必须的。

本工程真正需要的是：

```text
rknn_api.h
librknnrt.so
```

如果板子系统里已经安装了 RKNN Runtime，并且能在 `/usr/include`、`/usr/local/include`、`/usr/lib`、`/usr/local/lib` 等路径找到它们，可以直接：

```bash
./build.sh
```

如果你保留了 Rockchip 官方 `rknpu2` 目录，并且放在：

```text
workspace/
├── rknpu2/
├── sentinel-visioner/
└── sentinel-yolo-infer/
```

也可以直接 `./build.sh`，CMake 会自动尝试寻找：

```text
../rknpu2/runtime/Linux/librknn_api/include/rknn_api.h
../rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so
```

如果你的 RKNN Runtime 在其他目录，可以传：

```bash
./build.sh -DRKNN_ROOT=/path/to/rknpu2
```

或者显式传：

```bash
./build.sh \
  -DRKNN_API_INCLUDE_DIR=/path/to/librknn_api/include \
  -DRKNNRT_LIB=/path/to/librknnrt.so
```

## 重要：适配新版 sentinel-visioner

你上传的新版 `sentinel-visioner` 中，NPU RGA 转换成功后目前有这段逻辑：

```cpp
if (npuOk) {
    // TODO: NPU 推理接入后改为 npuTaskQueue.push(targetNpuBuf)
    ctx->npuRgbPool->release_buffer(targetNpuBuf);
}
```

这会导致 NPU 小图被立即归还，`wait_get_npu()` / `try_get_npu()` 队列中没有数据，`SentinelYoloInfer` 推理线程会一直等不到帧。

接入本推理类时，应改为：

```cpp
if (npuOk) {
    ctx->npuTaskQueue.push(targetNpuBuf);
}
```

推理类用完后会调用：

```cpp
visioner->release_npu(camNum, npuBuf);
```

所以不会造成 DMA buffer 泄漏。

可以用脚本检查：

```bash
cd sentinel-yolo-infer
./tools/check_sentinel_visioner_compat.sh ../sentinel-visioner
```

如果提示需要修复，可以执行：

```bash
./tools/patch_sentinel_visioner_enable_npu_queue.sh ../sentinel-visioner
```

## 编译

最简单方式：

```bash
cd workspace/sentinel-yolo-infer
./build.sh
```

如果 RKNN 不在默认路径：

```bash
./build.sh -DRKNN_ROOT=/path/to/rknpu2
```

如果 `sentinel-visioner` 不在兄弟目录：

```bash
./build.sh -DSENTINEL_VISIONER_DIR=/path/to/sentinel-visioner
```

## 单摄像头测试

```bash
./build/sentinel_yolo_infer_demo ./yolov8n.rknn /dev/video0 0 USB 640 480 6
```

参数含义：

```text
./sentinel_yolo_infer_demo <model.rknn> <video_device> <cam_num> <ISP|USB> [width] [height] [buffer_count]
```

正常输出类似：

```text
cam=0 boxes=2
  cls=0 conf=0.86 box=(123,80,300,420) ts=123456789000
```

## 核心接口

```cpp
SentinelVisioner visioner;
// visioner.add_camera(...)
// visioner.camera_stream_ctrl(...)

SentinelYoloInferConfig cfg;
cfg.modelPath = "./yolov8n.rknn";
cfg.boxThreshold = 0.25f;
cfg.nmsThreshold = 0.45f;
cfg.waitTimeoutMs = 200;

SentinelYoloInfer infer(&visioner, cfg);
infer.create_infer_thread(0);

YoloBBoxList fusionBoxes;
if (infer.try_get_fusion_result(0, fusionBoxes, 100)) {
    // 给融合模块
}

YoloBBoxList osdBoxes;
if (infer.try_get_osd_result(0, osdBoxes, 100)) {
    // 给 OSD 叠加模块
}
```

## 结果结构

```cpp
struct YoloBBox {
    uint32_t x1, y1;
    uint32_t x2, y2;
    uint32_t classId;
    float confidence;
    uint64_t timestampNs;
};
```

坐标默认位于 `sentinel-visioner` 输出的 640×640 RGB888 NPU 小图坐标系。`timestampNs` 由 `DmaBuffer_t::timestampUs * 1000` 得到；如果 timestamp 为空，则退化为当前 `CLOCK_MONOTONIC` 时间。

## 关于 official_yolov8_reference

新版压缩包已经移除 `official_yolov8_reference/`。它只是开发时参考官方 YOLOv8 示例的备份，不参与编译，不是运行所需文件。
