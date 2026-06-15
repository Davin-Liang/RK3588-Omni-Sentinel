# Sentinel YOLO Infer - Demo 与基准测试说明

本文档记录 `sentinel-yolo-infer` 的模型生成、板端部署、单路推理测试、双路推理测试和当前实测基准数据。

---

## 1. 测试环境

### 1.1 开发与编译环境

| 项目 | 内容 |
|---|---|
| 编译主机 | x86_64 Ubuntu 虚拟机 |
| 交叉编译目标 | ARM AArch64 |
| 交叉编译工具链 | `aarch64-buildroot-linux-gnu` |
| C++ 标准 | C++14 |
| 构建系统 | CMake + build.sh |
| 工程目录 | `~/RK3588-Omni-Sentinel/Software/APP` |

### 1.2 板端运行环境

| 项目 | 内容 |
|---|---|
| 板卡 | RK3588 |
| 系统 | Buildroot |
| 运行用户 | root |
| 板端 IP | `192.168.0.232` |
| 测试目录 | `/userdata/sentinel-yolo-infer-test` |
| 模型路径 | `/userdata/sentinel-yolo-infer-test/models/yolov8n.rknn` |
| 动态库路径 | `/userdata/sentinel-yolo-infer-test/lib` |

### 1.3 摄像头节点

通过：

```bash
v4l2-ctl --list-devices
```

确认节点如下：

```text
USB 2.0 Camera: USB Camera:
    /dev/video21
    /dev/video22

rkisp_mainpath:
    /dev/video11
    /dev/video12
    /dev/video13
    /dev/video14
    /dev/video15
    /dev/video16
    /dev/video17
```

当前测试选择：

| 摄像头 | 设备节点 | camNum | 类型 | 分辨率 |
|---|---|---:|---|---|
| USB 摄像头 | `/dev/video21` | 0 | USB | 640×480 |
| ISP 摄像头 | `/dev/video11` | 1 | ISP | 1920×1080 |

---

## 2. RKNN 模型生成记录

### 2.1 模型来源

使用 Rockchip 官方 `rknn_model_zoo/examples/yolov8` 示例生成 `yolov8n.rknn`，而不是使用 Ultralytics 原始 `yolov8n.pt` 直接导出。

原因：

- 当前 C++ 后处理逻辑适配 Rockchip 官方优化版 YOLOv8 输出结构；
- 模型应为 `outputs=9`；
- 当前零拷贝推理路径要求 `quant=true`；
- 需要生成 `rk3588 + i8` 模型。

### 2.2 获取官方 ONNX

```bash
cd ~/RK3588-Omni-Sentinel/Software/tools/rknn_model_zoo/examples/yolov8/model
chmod +x download_model.sh
./download_model.sh
ls -lh yolov8n.onnx
```

若网络无法下载，可手动下载后放入：

```text
rknn_model_zoo/examples/yolov8/model/yolov8n.onnx
```

### 2.3 Python 依赖版本

转换过程中曾出现：

```text
AttributeError: module 'onnx' has no attribute 'mapping'
```

最终可用版本为：

```bash
pip3 install --user \
  protobuf==3.20.3 \
  numpy==1.26.4 \
  onnx==1.14.1 \
  onnxoptimizer==0.2.7 \
  onnxruntime==1.16.0
```

检查：

```bash
python3 -c "import onnx; print('onnx =', onnx.__version__)"
python3 -c "import numpy; print('numpy =', numpy.__version__)"
python3 -c "import google.protobuf; print('protobuf =', google.protobuf.__version__)"
python3 -c "from rknn.api import RKNN; print('rknn-toolkit2 import ok')"
```

### 2.4 转换命令

```bash
cd ~/RK3588-Omni-Sentinel/Software/tools/rknn_model_zoo/examples/yolov8/python

python3 convert.py ../model/yolov8n.onnx rk3588 i8 ../model/yolov8n.rknn
```

正常结束日志：

```text
I rknn building ...
I rknn building done.
done
--> Export rknn model
done
```

转换时出现以下 warning 是预期现象：

```text
The default input dtype of 'images' is changed from 'float32' to 'int8'
The default output dtype ... is changed from 'float32' to 'int8'
```

当前 C++ 代码要求 int8 量化模型，因此该 warning 与预期一致。

### 2.5 模型复制

虚拟机：

```bash
mkdir -p ~/RK3588-Omni-Sentinel/Software/APP/resources/models

cp ~/RK3588-Omni-Sentinel/Software/tools/rknn_model_zoo/examples/yolov8/model/yolov8n.rknn \
   ~/RK3588-Omni-Sentinel/Software/APP/resources/models/yolov8n.rknn
```

板端：

```bash
scp ~/RK3588-Omni-Sentinel/Software/APP/resources/models/yolov8n.rknn \
    root@192.168.0.232:/userdata/sentinel-yolo-infer-test/models/
```

板端检查：

```bash
ls -lh /userdata/sentinel-yolo-infer-test/models/yolov8n.rknn
```

实测结果：

```text
-rw------- 1 root root 4.2M ... /userdata/sentinel-yolo-infer-test/models/yolov8n.rknn
```

---

## 3. 板端部署

### 3.1 创建目录

```bash
ssh root@192.168.0.232

mkdir -p /userdata/sentinel-yolo-infer-test/models
mkdir -p /userdata/sentinel-yolo-infer-test/lib
mkdir -p /userdata/sentinel-yolo-infer-test/logs
```

### 3.2 复制程序

在虚拟机执行：

```bash
cd ~/RK3588-Omni-Sentinel/Software/APP

scp sentinel-yolo-infer/build/sentinel_yolo_infer_demo \
    root@192.168.0.232:/userdata/sentinel-yolo-infer-test/

scp sentinel-yolo-infer/build/sentinel_yolo_infer_dual_demo \
    root@192.168.0.232:/userdata/sentinel-yolo-infer-test/
```

### 3.3 复制动态库

```bash
scp 3rdparty/rknpu2/lib/librknnrt.so \
    root@192.168.0.232:/userdata/sentinel-yolo-infer-test/lib/

scp 3rdparty/ffmpeg/lib/*.so* \
    root@192.168.0.232:/userdata/sentinel-yolo-infer-test/lib/

scp 3rdparty/librga/lib/librga.so \
    root@192.168.0.232:/userdata/sentinel-yolo-infer-test/lib/
```

如果 `librga.so` 路径不同，先查：

```bash
find 3rdparty -name "librga.so*"
```

### 3.4 板端启动前设置

```bash
cd /userdata/sentinel-yolo-infer-test
chmod +x ./sentinel_yolo_infer_demo
chmod +x ./sentinel_yolo_infer_dual_demo
export LD_LIBRARY_PATH=/userdata/sentinel-yolo-infer-test/lib:$LD_LIBRARY_PATH
```

当前 Buildroot 系统无 `ldd`，因此依赖检查以“直接运行是否缺库”为准。

---

## 4. Demo 01：USB 单路推理测试

### 4.1 运行命令

```bash
cd /userdata/sentinel-yolo-infer-test
export LD_LIBRARY_PATH=/userdata/sentinel-yolo-infer-test/lib:$LD_LIBRARY_PATH

./sentinel_yolo_infer_demo \
  ./models/yolov8n.rknn \
  /dev/video21 \
  0 \
  USB \
  640 \
  480 \
  6 2>&1 | tee logs/yolo_test_usb_video21_after_npu_queue_patch.log
```

### 4.2 预期日志

```text
[USB Cam] MJPG mode, 640x480 bufferCount=6
Current FPS set to: 120.101
Camera 0 (/dev/video21) added successfully.
Camera 0 capture thread STARTED.
[Yolov8RknnEngine] model input: 640x640x3, outputs=9, quant=true
[SentinelYoloInfer] infer thread started, camNum=0
cam=0 boxes=0
```

### 4.3 实测结论

USB 单路链路已打通：

```text
/dev/video21 → MJPG 解码 → RGA NPU 小图 → npuTaskQueue → RKNN YOLOv8 → cam=0 boxes=...
```

后续双路测试中 USB 侧出现了实际检测框：

```text
[USB] cam=0 boxes=1
  cls=15 conf=0.414537 box=(286,85,640,479)
```

---

## 5. Demo 02：ISP 单路推理测试

### 5.1 运行命令

```bash
cd /userdata/sentinel-yolo-infer-test
export LD_LIBRARY_PATH=/userdata/sentinel-yolo-infer-test/lib:$LD_LIBRARY_PATH

./sentinel_yolo_infer_demo \
  ./models/yolov8n.rknn \
  /dev/video11 \
  1 \
  ISP \
  1920 \
  1080 \
  6 2>&1 | tee logs/yolo_test_isp_video11_single.log
```

### 5.2 预期日志

```text
Camera 1 (/dev/video11) added successfully.
Camera 1 capture thread STARTED.
[Yolov8RknnEngine] model input: 640x640x3, outputs=9, quant=true
[SentinelYoloInfer] infer thread started, camNum=1
cam=1 boxes=...
```

### 5.3 实测结果

实测出现多次检测框：

```text
cam=1 boxes=1
  cls=41 conf=0.530762 box=(565,141,640,272)

cam=1 boxes=2
  cls=63 conf=0.670232 box=(266,160,640,492)
  cls=41 conf=0.348676 box=(564,142,640,279)
```

### 5.4 结论

ISP 单路推理链路可用，且已出现实际检测结果。

---

## 6. Demo 03：USB + ISP 双路推理线程测试

### 6.1 运行命令

```bash
cd /userdata/sentinel-yolo-infer-test
export LD_LIBRARY_PATH=/userdata/sentinel-yolo-infer-test/lib:$LD_LIBRARY_PATH

./sentinel_yolo_infer_dual_demo \
  ./models/yolov8n.rknn \
  /dev/video21 \
  /dev/video11 \
  640 \
  480 \
  1920 \
  1080 \
  6 2>&1 | tee logs/yolo_test_dual_usb21_isp11.log
```

参数说明：

| 参数 | 含义 |
|---|---|
| `./models/yolov8n.rknn` | RK3588 int8 YOLOv8n 模型 |
| `/dev/video21` | USB 摄像头 |
| `/dev/video11` | ISP 摄像头 |
| `640 480` | USB 输入分辨率 |
| `1920 1080` | ISP 输入分辨率 |
| `6` | buffer 数量 |

### 6.2 预期日志

```text
Camera 0 (/dev/video21) added successfully.
Camera 1 (/dev/video11) added successfully.
Camera 0 capture thread STARTED.
Camera 1 capture thread STARTED.
[Yolov8RknnEngine] model input: 640x640x3, outputs=9, quant=true
[Yolov8RknnEngine] model input: 640x640x3, outputs=9, quant=true
[SentinelYoloInfer] infer thread started, camNum=0
[SentinelYoloInfer] infer thread started, camNum=1
[DualDemo] started. USB camNum=0, ISP camNum=1
[USB] cam=0 boxes=...
[ISP] cam=1 boxes=...
```

### 6.3 实测结果

双路测试中出现：

```text
[USB] cam=0 boxes=0
[ISP] cam=1 boxes=0
[USB] cam=0 boxes=1
  cls=15 conf=0.414537 box=(286,85,640,479)
```

同时两个推理线程均成功启动，两个 RKNN 模型实例均显示：

```text
model input: 640x640x3, outputs=9, quant=true
```

### 6.4 结论

双路摄像头 + 双推理线程可行性测试通过。测试验证了：

1. `SentinelVisioner` 可以同时接入 USB 和 ISP 两路摄像头；
2. `SentinelYoloInfer` 可以按 `camNum` 创建两路独立推理线程；
3. 两路线程可以分别读取对应摄像头的 NPU 小图；
4. 两路线程均可输出独立结果队列；
5. 至少 USB 侧在双路场景下出现实际检测框，ISP 侧在单路测试中出现实际检测框。

---

## 7. 当前基准数据

| 项目 | 实测值 / 现象 |
|---|---|
| RKNN 模型大小 | 约 4.2 MB |
| 模型输入 | 640×640×3 |
| 模型输出 | 9 outputs |
| 模型量化 | `quant=true` |
| USB 输入 | `/dev/video21`，MJPG 640×480 |
| USB 当前 FPS 设置 | `120.101` |
| ISP 输入 | `/dev/video11`，1920×1080 |
| RGA 单帧耗时 | 常见 0–6 ms |
| USB 双路检测输出 | 出现 `boxes=1`，`conf≈0.25–0.41` |
| ISP 单路检测输出 | 出现 `boxes=1/2`，最高 `conf≈0.67` |
| 双路线程 | camNum=0 / camNum=1 均启动成功 |
| 非阻塞 warning | `preview pool empty`，不影响推理链路 |

---

## 8. 测试通过标准

若满足以下条件，可认为当前推理类可行性测试通过：

- [x] RKNN 模型生成成功；
- [x] 模型能够在板端加载；
- [x] `model input: 640x640x3, outputs=9, quant=true`；
- [x] 单路 USB 推理线程启动成功；
- [x] 单路 ISP 推理线程启动成功；
- [x] 双路推理线程同时启动成功；
- [x] USB 与 ISP 均能输出对应 `camNum` 的结果；
- [x] 至少一路出现 `boxes > 0`；
- [x] 程序运行过程中未出现 `rknn_init failed` / `rknn_run failed` / 崩溃。

当前测试结论：通过。
