# DmaBufferPool: 高性能零拷贝 DMA 内存池

## 📖 简介

`DmaBufferPool` 是专为嵌入式异构计算平台（如 RK3568）设计的高性能、线程安全的物理内存管理模块。

在边缘端 AI 视觉流媒体架构中，本模块通过提前分配连续的 DMA 物理内存块，并采用**“空闲链表 (Free List)”**机制进行管理，实现了跨硬件加速器之间数据传递的 **100% 零 CPU 拷贝**与  **O(1) 复杂度的极速分配** 。

## ✨ 核心特性

* **极致性能** ：基于单向空闲链表实现，获取和释放内存块的时间复杂度均为严格的 O(1)，全程无遍历。
* **绝对的内存安全** ：
* **全量花名册 (Full Roster)** ：底层维护 `allBuffers_` 记录所有已分配的物理内存。即使上层业务发生异常崩溃，析构时依然能无死角地回收所有底层 DRM/DMA 资源。
* **异常回滚** ：在初始化分配阶段如果物理内存耗尽，会自动回滚并清理已分配的碎片内存。
* **并发安全 (Thread-Safe)** ：内置 `std::mutex` 互斥锁，完美支持多线程架构。
* **良好的工程兼容性 (C++14)** ：
* 完全兼容 C++14 标准，适应老版本交叉编译工具链（如 GCC 6.3.1）。
* 采用 `enum class` 强类型枚举定义图像格式（YUV422, RGB888, NV12）。
* CMake 脚本原生支持“独立编译 (Standalone)”与“子工程嵌套 (Sub-Project)”两种模式。

## 🛠️ 环境依赖

* **C++ 标准** : 强制要求 C++14 (`CMAKE_CXX_STANDARD 14`)
* **构建工具** : CMake 3.4.1 或更高版本
* **交叉编译器** : `aarch64-linux-gnu-gcc` (推荐 Linaro GCC 6.3.1-2017.05)
* **底层依赖** : 依赖同级或父级目录下的 `allocator_obj` 库实现 DMA 底层分配。

## 📦 交叉编译与构建

本项目提供了一键式交叉编译脚本。该脚本会自动配置目标板的 GCC/G++ 环境变量，并执行 CMake 构建和安装。

**执行编译：**

**Bash**

```
# 赋予执行权限并运行编译脚本
chmod +x build.sh
./build.sh
```

**编译产物：**
编译成功后，将在项目根目录生成 `install` 文件夹，包含：

* 静态库文件：`libdma_buffer_pool_lib.a`
* 相关的头文件。

## ⚙️ CMake 引入说明 (作为子模块)

本工程的 CMakeLists.txt 已做了防御性设计（处理了菱形依赖）。如果您有一个顶级的超级工程（如 `test_all`），可以直接通过 `add_subdirectory` 将其作为模块引入：

**CMake**

```
# 在顶层 CMakeLists.txt 中：

# 1. 引入 dma-buffer-pool 目录
add_subdirectory(path/to/dma-buffer-pool)

# 2. 将其链接到您的业务模块或可执行文件
add_executable(my_vision_app main.cpp)
target_link_libraries(my_vision_app PUBLIC dma_buffer_pool_lib)
```

*注：作为子工程运行时，本模块会自动使用父工程的 `CMAKE_INSTALL_PREFIX` 路径。*

## 🚀 业务代码接入指引

### 1. 初始化内存池

在主控线程或类构造阶段分配内存池。由于 C++14 没有 `[[nodiscard]]`， **请开发者务必手动检查 `alloc_pool` 的返回值** ，防止物理内存耗尽导致的空指针崩溃。

**C++**

```
#include "dma-buffer-pool.h"
#include <iostream>

DmaBufferPool yoloPool;

// 预分配 3 块 640x640 的 RGB888 格式物理内存
bool success = yoloPool.alloc_pool(3, 640, 640, BufferFormat::RGB888);
if (!success) {
    std::cerr << "致命错误：DMA 物理内存池分配失败！" << std::endl;
    // 执行退出或降级逻辑
}
```

### 2. 获取空闲内存 (Get Buffer)

在数据生产端极速弹出一块空闲内存。

**C++**

```
DmaBuffer_t* buf = yoloPool.get_buffer();

if (buf != nullptr) {
    // 成功获取！
    // 将图像数据写入 buf->virtAddr，或把 buf->dmaFd 传给 RGA 硬件
    // ...
} else {
    // 内存池已枯竭，建议执行丢帧策略
    std::cerr << "Warning: Pool empty, dropping frame!" << std::endl;
}
```

### 3. 归还内存 (Release Buffer)

在数据消费端（如 YOLO 推理完成）将内存还给池子。

**C++**

```
// 极速归还内存（头插法放回空闲链表）
yoloPool.release_buffer(buf);
```
