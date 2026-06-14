# BUG_RECORD — sentinel-yolo-infer 问题记录

本文档记录 `sentinel-yolo-infer` 开发、集成、模型生成和板端测试过程中遇到的问题。记录格式尽量保持“现象 → 原因 → 解决 → 状态”，便于后续复现、归档和交接。

---

## 1. x86_64 虚拟机上使用 ldd 检查 ARM64 demo 显示 `not a dynamic executable`

**现象**

在虚拟机中执行：

```bash
ldd build/sentinel_yolo_infer_demo
```

输出：

```text
not a dynamic executable
```

进一步检查：

```bash
file build/sentinel_yolo_infer_demo
uname -m
readelf -d build/sentinel_yolo_infer_demo | grep NEEDED
readelf -l build/sentinel_yolo_infer_demo | grep interpreter
```

结果显示：

```text
ELF 64-bit LSB pie executable, ARM aarch64, dynamically linked
uname -m = x86_64
interpreter /lib/ld-linux-aarch64.so.1
```

**原因**

`sentinel_yolo_infer_demo` 是通过交叉编译器生成的 AArch64 目标程序，而当前检查环境是 x86_64 虚拟机。x86_64 环境下的 `ldd` 不能正确检查 ARM64 可执行文件。

**解决**

虚拟机上只做交叉编译和静态检查：

```bash
file build/sentinel_yolo_infer_demo
readelf -d build/sentinel_yolo_infer_demo | grep NEEDED
```

真实 `ldd` 检查和运行测试应在 RK3588 板端执行。但当前 Buildroot 板端未内置 `ldd`，因此最终采用 `file + 直接运行` 的方式判断动态库是否齐全。

**状态**

非代码问题，属于测试环境使用不当。

---

## 2. APP 目录中缺少 `.rknn` 模型

**现象**

在 `APP/` 目录下执行：

```bash
find . -name "*.rknn" -type f
```

无输出。

**原因**

`sentinel-yolo-infer` 是推理封装模块，不负责自动生成模型文件。RK3588 NPU 运行需要 `.rknn` 模型，不能直接使用 `.pt` 或 `.onnx`。

**解决**

使用 Rockchip 官方 `rknn_model_zoo/examples/yolov8` 生成适配当前推理代码的模型：

```bash
cd ~/RK3588-Omni-Sentinel/Software/tools/rknn_model_zoo/examples/yolov8/python
python3 convert.py ../model/yolov8n.onnx rk3588 i8 ../model/yolov8n.rknn
```

生成后复制到：

```text
APP/resources/models/yolov8n.rknn
/userdata/sentinel-yolo-infer-test/models/yolov8n.rknn
```

**状态**

已解决。模型已生成并部署，板端显示模型文件大小约 4.2 MB。

---



## 3. RKNN 转换时报 `onnx.mapping` 缺失

**现象**

执行：

```bash
python3 convert.py ../model/yolov8n.onnx rk3588 i8 ../model/yolov8n.rknn
```

报错：

```text
AttributeError: module 'onnx' has no attribute 'mapping'
```

**原因**

当前 Python 环境中的 `onnx` 版本与 `rknn-toolkit2 2.3.2` 的 `load_onnx` 逻辑不兼容。工具内部使用 `onnx.mapping`，而较新版本 ONNX 中该接口发生变化。

**解决**

参考 `rknn_model_zoo/docs/requirements_cp38.txt`，降级关键依赖：

```bash
pip3 uninstall -y onnx onnxruntime onnxoptimizer protobuf numpy

pip3 install --user \
  protobuf==3.20.3 \
  numpy==1.26.4 \
  onnx==1.14.1 \
  onnxoptimizer==0.2.7 \
  onnxruntime==1.16.0
```

重新转换成功，日志显示：

```text
I rknn building done.
--> Export rknn model
done
```

**注意**

安装时 pip 可能提示 `rknn-toolkit2 2.3.2 requires onnx>=1.16.1` 等依赖冲突，但实测 `onnx==1.14.1` 可解决 `onnx.mapping` 问题并完成转换。

**状态**

已解决。属于模型转换环境依赖版本问题。

---


## 4. `sentinel-visioner` 未将 NPU 小图推入 `npuTaskQueue`

**现象**

摄像头采集和 RGA 转换持续运行，但 YOLO demo 没有输出：

```text
cam=0 boxes=...
```

只看到：

```text
[time] RGA (NPU + Preview): ...
[Thread] Warning: preview pool empty! Dropping frame.
```

**原因**

`sentinel-visioner/src/sentinel-visioner.cpp` 中，NPU 图像生成成功后直接释放了 `targetNpuBuf`：

```cpp
if (npuOk) {
    ctx->npuRgbPool->release_buffer(targetNpuBuf);
}
```

导致 `sentinel-yolo-infer` 调用 `try_get_npu()` 时拿不到 NPU 小图。

**解决**

修改为：

```cpp
if (npuOk) {
    ctx->npuTaskQueue.push(targetNpuBuf);
} else {
    ctx->npuRgbPool->release_buffer(targetNpuBuf);
}
```

由 `sentinel-yolo-infer` 在推理完成后通过：

```cpp
visioner_->release_npu(camNum, npuBuf);
```

归还 DMA buffer。

**状态**

已解决。该修改属于 `sentinel-visioner` 必要配合修改，应保留。

---

## 5. `preview pool empty` 持续出现

**现象**

运行单路或双路 YOLO demo 时持续出现：

```text
[Thread] Warning: preview pool empty! Dropping frame.
```

**原因**

当前 YOLO demo 只消费 NPU 推理队列，不消费 `sentinel-visioner` 的 preview 队列。`previewPool` 中的 buffer 被填满后，捕获线程无法继续获取空闲 preview buffer，因此丢弃预览帧。

**影响**

该问题影响预览链路，不影响当前 YOLO 推理链路。因为测试日志已经持续输出：

```text
cam=0 boxes=...
[USB] cam=0 boxes=...
[ISP] cam=1 boxes=...
```

**解决方向**

后续可选方案：

1. 在 demo 中增加 preview 消费线程；
2. 在 `sentinel-visioner` 中增加关闭 preview 输出的开关；
3. 在真实业务中由 OSD/显示模块正常消费 preview 队列。

**状态**

非阻塞问题，待后续优化。

---

## 6. ISP 摄像头设置帧率时报 `VIDIOC_S_PARM failed`

**现象**

双路测试中 `/dev/video11` 输出：

```text
VIDIOC_S_PARM failed: Inappropriate ioctl for device (Ignore if not supported)
```

**原因**

`/dev/video11` 对应 RKISP mainpath 节点不支持当前设置帧率参数的 ioctl。日志中已经说明该问题可忽略。

**影响**

非致命。后续日志显示：

```text
Camera 1 (/dev/video11) added successfully.
Camera 1 capture thread STARTED.
[SentinelYoloInfer] infer thread started, camNum=1
[ISP] cam=1 boxes=...
```

说明 ISP 摄像头实际可用。

**状态**

已记录，非阻塞问题。


