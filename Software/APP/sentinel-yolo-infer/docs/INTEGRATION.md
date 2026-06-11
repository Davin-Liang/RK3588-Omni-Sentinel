# 集成说明

## 1. `rknpu2/` 不是固定要求

`sentinel-yolo-infer` 依赖 RKNN Runtime，但不依赖目录名叫 `rknpu2`。

必须能找到：

- `rknn_api.h`
- `librknnrt.so`

如果它们已经安装到系统路径，直接 `./build.sh` 即可。否则传 `-DRKNN_ROOT` 或分别传 `-DRKNN_API_INCLUDE_DIR`、`-DRKNNRT_LIB`。

## 2. `sentinel-visioner` 默认自动接入

推荐结构：

```text
workspace/
├── sentinel-visioner/
└── sentinel-yolo-infer/
```

这种结构下，`sentinel-yolo-infer/CMakeLists.txt` 会自动：

```cmake
add_subdirectory(../sentinel-visioner ...)
```

所以不用手动传 sentinel 路径。

## 3. 新版 sentinel-visioner 必须打开 NPU 队列输出

你上传的新版 `sentinel-visioner/src/sentinel-visioner.cpp` 中，NPU RGA 输出当前被立即归还：

```cpp
if (npuOk) {
    // TODO: NPU 推理接入后改为 npuTaskQueue.push(targetNpuBuf)
    ctx->npuRgbPool->release_buffer(targetNpuBuf);
}
```

这会导致推理线程拿不到 NPU 小图。接入 `SentinelYoloInfer` 时必须改成：

```cpp
if (npuOk) {
    ctx->npuTaskQueue.push(targetNpuBuf);
}
```

随后推理类在每帧处理完成后会自动：

```cpp
visioner_->release_npu(camNum, npuBuf);
```

## 4. 检查/修复脚本

```bash
cd sentinel-yolo-infer
./tools/check_sentinel_visioner_compat.sh ../sentinel-visioner
./tools/patch_sentinel_visioner_enable_npu_queue.sh ../sentinel-visioner
```

## 5. 运行 demo

```bash
./build/sentinel_yolo_infer_demo ./yolov8n.rknn /dev/video0 0 USB 640 480 6
```
