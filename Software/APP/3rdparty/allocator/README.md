# allocator_obj 内存分配模块

本模块提供基于 DMA（Direct Memory Access）的内存分配功能。它通过 `libdrm` 进行底层内存管理，并与瑞芯微的 NPU (`librknnrt`) 和 2D 硬件加速器 (`librga`) 深度集成，非常适合在 RK3568 等 SoC 上实现图像/张量数据的“零拷贝”（Zero-Copy）硬件级流转。

## 📦 模块功能与特性

* **目标类型** : CMake `OBJECT` 库。编译后仅生成对象文件（`.o`），不生成独立的库文件。
* **核心源码** : `dma_alloc.cpp` / `dma_alloc.h`
* **已配置的依赖** :
* **3rdparty** : 自动引入并链接了第三方模块中的 `${LIBRKNNRT}` (NPU 推理库)。
* **RGA** : 导出了 `${LIBRGA_INCLUDES}` 的头文件搜索路径。
* **DRM** : 集成了上层目录的 `libdrm` 头文件路径。

---

## 🚀 如何在主项目中使用

### 第一步：引入本模块目录

在你的主（或上层）项目的 `CMakeLists.txt` 中，通过 `add_subdirectory` 引入包含此 CMakeLists 的文件夹。

**CMake**

```
# 假设当前模块放在主项目的 modules/allocator 目录下
add_subdirectory(modules/allocator)
```

### 第二步：将 OBJECT 库链接到你的目标程序

从 CMake 3.12 开始，你可以像链接普通库一样，使用 `target_link_libraries` 直接链接 OBJECT 库。CMake 会自动提取 `.o` 文件并合并到你的目标中，同时继承所有的 `PUBLIC` 属性（如头文件路径和 RKNN 的链接关系）。

**CMake**

```
# 定义你的主程序或主动态库
add_executable(my_ai_app main.cpp)

# 直接链接 allocator_obj
target_link_libraries(my_ai_app PRIVATE 
    allocator_obj 
)
```

**上述链接操作会自动为 `my_ai_app` 带来以下效果：**

1. 自动把 `dma_alloc.cpp` 编译出的 `.o` 文件打包进 `my_ai_app`。
2. 自动包含本模块提供的 `include` 目录（你可以直接 `#include "dma_alloc.h"`）。
3. 自动包含 `libdrm` 和 RGA 的头文件目录。
4. 自动将 `my_ai_app` 链接到 `librknnrt.so`。

---

## ⚠️ 注意事项

1. **头文件路径的相对位置** ：
   本模块的 CMakeLists 使用了 `../libdrm/include` 和 `../../3rdparty/`。请确保你的整个项目目录树结构严格遵循此相对关系：
   **Plaintext**

```
   project_root/
   ├── 3rdparty/             # 你之前配置的第三方库目录
   ├── libdrm/               # DRM 库的头文件目录
   ├── allocator/        # 本模块目录
   	├── CMakeLists.txt
        ├── dma_alloc.cpp
        └── include/
              └── dma_alloc.h
```

1. **第三方库的构建目录** ：
   脚本中 `add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/ 3rdparty.out)` 指定了第二个参数 `3rdparty.out`。因为 `3rdparty` 在当前源码树外部（由 `../` 可知），CMake 要求显式指定它的构建输出目录，以避免污染源码树。
