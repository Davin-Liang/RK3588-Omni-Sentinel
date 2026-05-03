# SentinelVisioner

**SentinelVisioner** 是一个专为瑞芯微（Rockchip）边缘计算平台设计的高性能多路 MIPI 摄像头采集库。

本库基于 C++14 编写，深度整合了 **V4L2 (Video for Linux 2)** 的 DMA-BUF 导出模式（`VIDIOC_EXPBUF`）与瑞芯微 **RGA (2D Graphics Acceleration)** 硬件加速模块。实现了从摄像头捕获、格式转换（NV12 到 RGB888）、等比例缩放（Letterbox）到视频防抖（平移补偿）的全链路 **纯硬件零拷贝** ，将 CPU 占用降至最低，是接入 NPU (如 YOLO 推理) 或硬件编码器前置数据处理的理想框架。

---

# ✨ 核心特性

* **多路并发支持** ：基于 `epoll` 与独立线程池，轻松管理和监听多路摄像头。
* **极致零拷贝 (Zero-Copy)** ：应用层不涉及任何内存映射 (`mmap`) 与 CPU 数据搬运，数据流转仅靠文件描述符 (`dmaFd`)。
* **硬件级 ISP 与 RGA 联动** ：
* 纯硬件完成 `YCrCb_420_SP` (NV12) 到 `RGB_888` 的色彩空间转换。
* 自带硬件级 Letterbox（等比例缩放与灰边填充）。
* 预留电子防抖 (EIS) 接口，支持动态的有符号像素级平移补偿。
* **高性能内存池** ：内置独立的 `DmaBufferPool`，防止高帧率下的内存碎片化。
* **线程安全队列** ：提供非轮询（Condition Variable）的安全出队接口，彻底解放消费者线程（推理/推流引擎）的 CPU 占用。

---

## 🛠️ 环境依赖

在编译本工程之前，请确保环境中包含以下组件：

1. **CMake** (>= 3.4.1)
2. **交叉编译工具链** (如 `aarch64-buildroot-linux-gnu`)
3. **瑞芯微 RGA 库** (`librga` / `im2d_api`)
4. **DmaBufferPool** (本项目的依赖模块)
5. *可选：OpenCV（仅用于 Demo 测试显示）*

---

## 🚀 编译指南

本工程支持交叉编译模式。项目根目录下提供了一键编译脚本（如 `build.sh`）。

### 1. 配置交叉编译器路径

在编译脚本中，将 `TOOL_CHAIN` 修改为你本机的实际 SDK 路径：

**Bash**

```
# 修改为你实际的交叉编译工具链路径
TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
```

### 2. 执行编译命令

使用以下脚本进行构建、编译并安装（安装目录默认为工程根目录下的 `install/`）：

**Bash**

```
set -e

TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
GCC_COMPILER=$TOOL_CHAIN/bin/aarch64-buildroot-linux-gnu

export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

# 创建并进入构建目录
mkdir -p build/
cd build/

# 生成 Makefile
cmake ..

# 多线程编译
make -j4

# 安装动态库与可执行文件
make install
cd -

echo "Build Success! Output is in install/ directory."
```

---

## 📖 使用说明 (快速上手)

`SentinelVisioner` 的使用遵循 **“注册设备 -> 开启流 -> 消费队列 -> 归还内存 -> 关闭流”** 的标准生命周期。

### 示例代码 (Demo)

以下示例展示了如何在独立的推理线程中接入该库：

**C++**

```
#include <iostream>
#include <string>
#include <thread>
#include "SentinelVisioner.h"

// 假设这是你的 YOLO 推理业务线程
void consumer_thread_func(SentinelVisioner* visioner, int camNum) {
    std::cout << "Consumer thread started for Camera " << camNum << std::endl;
    bool is_running = true;

    while (is_running) {
        // 1. 阻塞等待：从安全队列获取经过 RGA 硬件处理好的 RGB888 DMA 内存块
        // 这里的线程会休眠，直到 V4L2 产生新数据并经 RGA 转换完成
        DmaBuffer_t* readyBuffer = visioner->wait_get_rga_buffer(camNum);

        if (readyBuffer != nullptr) {
            // -------------------------------------------------------------
            // 2. 业务处理 (完全拥有该 Buffer 的使用权)
            // 此时 targetBuffer 中装的是 640x640 RGB888 的数据
            // 你可以直接将 readyBuffer->dmaFd 交给 NPU、GPU 或编码器
          
            // do_yolo_inference(readyBuffer->dmaFd, readyBuffer->width, readyBuffer->height);
            // -------------------------------------------------------------

            // 3. 【极度重要】：业务使用完毕后，必须将其归还给内存池！
            // 否则内存池干涸会导致底层丢帧
            visioner->release_rga_buffer(camNum, readyBuffer);
        }
    }
}

int main() {
    SentinelVisioner visioner;

    std::string devName = "/dev/video0";
    int camNum = 0;
  
    // 1. 注册并添加摄像头
    // 参数: 设备节点名, 原始图像宽, 原始图像高, 内部流转Buffer数量, 摄像机编号
    // 此过程会自动向 V4L2 申请内存、导出 DMA fd，并预分配好 RGA 目标内存池 (640x640)
    if (!visioner.add_camera(devName, 1920, 1080, 4, camNum)) {
        std::cerr << "Failed to add camera!" << std::endl;
        return -1;
    }

    // 2. 开启视频流 (内部会自动拉起 epoll 捕获线程)
    if (!visioner.camera_stream_ctrl(camNum, true)) {
        std::cerr << "Failed to start camera stream!" << std::endl;
        return -1;
    }

    // 3. 拉起下游的消费者线程 (如 NPU 推理)
    std::thread consumer(consumer_thread_func, &visioner, camNum);

    // 主线程保持运行...
    std::this_thread::sleep_for(std::chrono::seconds(60));

    // 4. 关闭系统
    std::cout << "Shutting down..." << std::endl;
    // (关闭消费者线程的逻辑省略)
  
    visioner.camera_stream_ctrl(camNum, false);
    consumer.join();

    return 0;
}
```

## ⚠️ 注意事项与避坑指南

1. **绝对的归还机制** ：下游业务（如推理线程）在调用 `wait_get_rga_buffer` 获取到 `DmaBuffer_t*` 并在硬件层使用完毕后，**必须**调用 `release_rga_buffer()`。由于整个管线是零拷贝的，不归还 Buffer 会导致 RGA 无目标内存可用，触发 `Drop Frame`。
2. **在原始大图上绘制 OSD** ：NPU 推理所需的小尺寸 RGB888 Buffer（通常带有灰色 Padding 填充边） **不适合直接用于推流或显示** 。如果需要画框，请将 NPU 输出的坐标映射回 1080P/NV12 的原始大图上，并使用 RGA 的 `imfill` 或 `imblend` 在大图上绘制后推流。
3. **系统权限** ：操作 `/dev/videoX` 和底层 DMA API 通常需要 `root` 权限或相应的 `video` 用户组权限。
