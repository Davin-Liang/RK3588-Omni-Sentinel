# SentinelVisioner

**SentinelVisioner** 是一个专为瑞芯微（Rockchip）边缘计算平台（如 RK3568 / RK3588）设计的高性能、零拷贝多路视觉流水线框架。

本库基于 C++14 编写，深度整合了 **V4L2 (Video for Linux 2)** 的 DMA-BUF 导出模式（`VIDIOC_EXPBUF`）与瑞芯微 **RGA (2D Graphics Acceleration)** 硬件加速模块。实现了从底层摄像头捕获、格式转换（NV12 到 RGB888）、等比例缩放（Letterbox）、画中画拷贝到视频防抖（平移补偿）的全链路  **纯硬件零拷贝** 。本框架是接入高算力 NPU (如 YOLO 推理)、多路算法分发或硬件编码推流前最理想的高效数据前处理底座。

---

## ✨ 核心架构与特性

* **多路并发与一转多架构** ：基于 `epoll` 监听底层 V4L2 节点，单路物理视频流输入后，通过 RGA 硬件瞬间裂变为三路独立数据流（NPU 专用小图、720P OSD 渲染图、1080P 原始推流大图），互不干扰。
* **极致零拷贝 (Zero-Copy)** ：应用层不涉及任何内存映射 (`mmap`) 与 CPU 像素搬运，百兆级别的高清视频流转仅依靠轻量级的 DMA 文件描述符 (`dmaFd`) 传递。
* **硬件级 ISP 与 RGA 联动** ：
* **智能缩放与转换** ：纯硬件完成 `YCrCb_420_SP` (NV12) 到 `RGB_888` 的转换与等比例缩放。
* **边缘填充 (Letterbox)** ：自动进行灰边 Padding 防脏数据。
* **EIS 电子防抖接入点** ：原生预留有符号横纵坐标偏移量接口，无缝对接外部 IMU 陀螺仪数据进行像素级平移补偿。
* **池化生命周期管理** ：针对不同分辨率内置三个独立的 `DmaBufferPool`，不仅防止了内存碎片化，更通过严格的“借出-归还”机制彻底根绝了 Fd 句柄与 DMA 内存泄漏。
* **休眠级线程安全队列** ：内置基于条件变量（Condition Variable）的 `ThreadSafeQueue`，在无数据时彻底挂起消费者线程，告别自旋锁，CPU 空闲占用率降至  **0.0%** 。

---

## 🛠️ 环境依赖

在编译本工程之前，请确保环境中包含以下组件：

1. **CMake** (>= 3.4.1)
2. **交叉编译工具链** (如 `aarch64-buildroot-linux-gnu`)
3. **瑞芯微 RGA 库** (`librga` / `im2d_api`)
4. **DmaBufferPool & ThreadSafeQueue** (本项目的基础依赖组件)

---

## 🚀 编译指南

本工程支持交叉编译模式。项目根目录下提供了一键编译脚本。

### 1. 配置交叉编译器路径

在编译脚本中，将 `TOOL_CHAIN` 修改为你本机的实际 SDK 路径：

**Bash**

```
# 修改为你实际的交叉编译工具链路径
TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
```

### 2. 执行编译命令

使用以下脚本进行构建、编译并安装（输出在 `install/` 目录）：

**Bash**

```
mkdir -p build/
cd build/
cmake ..
make -j4
make install
```

---

## 📖 快速上手与并发示例

`SentinelVisioner` 的标准生命周期为： **注册设备 -> 开启流 -> 消费者异步阻塞拉取 -> 处理后归还内存 -> 关闭流** 。

以下展示了如何拉起“NPU推理+OSD绘制”与“原始视频推流”两个并发消费者线程：

**C++**

```
#include <iostream>
#include <thread>
#include "sentinel-visioner.h"

// 消费者 1：负责 NPU 推理 与 OSD 画框
void npu_osd_consumer_thread(SentinelVisioner* visioner, int camNum) {
    while (true) { // (实际应用中替换为全局运行标志)
        // 1. 阻塞等待：获取打包好的 NPU RGB888小图 和 720P OSD底图
        NpuOSD task = visioner->wait_get_npuOSD(camNum);
        if (task.npuImage == nullptr) continue; // 退出或虚假唤醒拦截

        // 2. 硬件加速送进 NPU 运算
        // auto results = do_yolo_inference(task.npuImage->dmaFd);

        // 3. 将推理结果绘制在独立的 720P NV12 图像上
        // imfill_boxes(task.osdImage->dmaFd, results);

        // 4. 【必须】交还 DMA 内存，避免内存干涸
        visioner->release_npuOSD(camNum, &task);
    }
}

// 消费者 2：负责原始高分辨率图像编码推流
void stream_consumer_thread(SentinelVisioner* visioner, int camNum) {
    while (true) {
        // 1. 阻塞等待：获取 1080P NV12 原图的独立零拷贝副本
        DmaBuffer_t* origImage = visioner->wait_get_orig_copy_buffer(camNum);
        if (origImage == nullptr) continue;

        // 2. 压入 MPP 硬件编码器 (H264/H265)
        // mpp_encode_push(origImage->dmaFd, origImage->timestampUs);

        // 3. 【必须】交还拷贝大图的 DMA 内存
        visioner->release_orig_copy_buffer(camNum, origImage);
    }
}

int main() {
    SentinelVisioner visioner;
    int camNum = 0;
    std::string devName = "/dev/video11"; // ISP 输出节点
  
    // 1. 初始化 1080P 摄像机并预分配三大内存池 (缓冲块数=8)
    if (!visioner.add_camera(devName, 1920, 1080, 8, camNum)) return -1;

    // 2. 开启硬件视频流与 epoll 捕获分发守护线程
    if (!visioner.camera_stream_ctrl(camNum, true)) return -1;

    // 3. 拉起下游双链路异步消费者
    std::thread npu_thread(npu_osd_consumer_thread, &visioner, camNum);
    std::thread stream_thread(stream_consumer_thread, &visioner, camNum);

    // 主线程保持运行...
    std::this_thread::sleep_for(std::chrono::seconds(60));

    // 4. 优雅回收资源
    visioner.camera_stream_ctrl(camNum, false);
    // (需配合唤醒消费者线程逻辑退出)
    npu_thread.join();
    stream_thread.join();

    return 0;
}
```

---

## 📊 实测基准数据 (Benchmarks)

以下数据基于 RK3568/RK3588 平台实测，记录了在  **双路异步队列满载运行** （1080P输入 -> NPU 640x640分支 + OSD 720P分支 + 1080P推流拷贝分支）下的极高并发性能：

* **端到端延迟 (Latency)** :  **稳定在 64 ms 左右** （该延迟严格涵盖了 V4L2 硬件曝光捕获、连续 3 次 RGA 硬件调度处理及线程间安全通信开销。内部周转极速，绝无内存阻塞）。
* **CPU 线程负载 (基于 `top -H`)** :
* **捕获与调度分发线程** ：单核峰值仅  **8.7%** ，繁重像素运算已完全卸载至 RGA。
* **双消费者业务线程** ：空闲时占用严格为  **0.0%** ，条件变量休眠调度完美。
* **零拷贝内存特征** : `RES` (常驻物理内存) 仅 1.7 MB，而 `VIRT` (虚拟内存) 高达 264.2 MB，证明了庞大的多路并发图像池仅在内核态驻留，并以 Fd 形式在用户态极速穿梭。
* **RGA 硬件利用率** : 主调度器 `scheduler[0] (rga3)` 单帧连续处理三次重负载任务，峰值负载仅占  **5%** 。并发潜力巨大。
* **Fd 防漏监控** : 经高频压测监控 `ls /proc/PID/fd | wc -l`，进程全局打开文件描述符数量严格恒定在  **38 个** ，未见任何递增泄漏，生命周期闭环绝对安全。

> 具体展示请查看 docs/ 文件夹下的DEMO-INSTRUCTIONS.md。

---

## ⚠️ 避坑与注意事项

1. **绝对的归还机制** ：无论是在发生错误分支、丢弃数据分支，还是处理完成分支，都 **必须** 调用对应的 `release_` 接口归还 `DmaBuffer_t`。内存池的枯竭将导致底层 V4L2 引擎因无可用缓冲块而发生致命级 `Drop Frame`。
2. **带框推流策略（OSD）** ：NPU 推理使用的 `RGB888` 缓冲区块通常为满足检测模型强制加入了 Letterbox 灰边，不应直接用于界面展示。若需带框视频推流，请在附带的 `task.osdImage` (干净的 720P) 底图上进行框体绘制（如调用 `imfill`）。
3. **驱动日志拦截** ：若发生 `[RGA Error] Invalid DMA fd` 报错，通常意味着底层视频流启动失败或捕获了坏帧。程序已内置防雪崩机制，会立刻切断后续处理并归还错乱内存，请优先排查硬件接线与 V4L2 `VIDIOC_S_FMT` 协商结果。
