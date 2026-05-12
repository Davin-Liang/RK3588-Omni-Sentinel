# LidarCameraFusion — 视觉-雷达数据融合

基于 **RK3588 / 脱离 ROS / C++14** 的激光雷达与相机目标检测空间对齐组件。

## 核心功能

- **外参变换**：将雷达点从雷达坐标系变换到相机坐标系（4×4 齐次变换，z=0 优化）
- **内参投影**：将 3D 相机坐标点投影到 2D 像素平面（针孔相机模型）
- **Bbox 归属判定**：确定每个 YOLO 检测框覆盖了哪些雷达点——首次命中即跳出
- **两趟扫描**：一趟变换+分类，二趟计数排序写出——热路径零堆分配
- **预分配缓冲区**：构造时一次性分配 ~12 KB，fuse_data 热路径零堆分配
- **多相机累积**：同一雷达帧可通过多次 `fuse_data()` 累积多个相机的检测结果，FOV 不重叠场景下无交叉泄漏

## 环境依赖

1. CMake >= 3.4.1
2. aarch64 交叉编译器 (`aarch64-buildroot-linux-gnu`)
3. C++14 标准库（`<cstdint>`, `<vector>`, `<new>`）
4. `sentinel_lslidarer.h`（使用 `LidarPoint`, `LidarFrame`）

**无 ROS2 / PCL / Eigen / Boost 依赖。**

## 构建

### 交叉编译（ARM64 目标平台）

```bash
./build.sh
```

可通过环境变量覆盖工具链路径：
```bash
CROSS_COMPILE_PATH=/your/toolchain/path ./build.sh
```

### x86 本地编译（仅 Demo 测试）

```bash
mkdir -p build/ && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

产物：
| 文件 | 说明 |
|------|------|
| `install/lib/liblidar_camera_fusion_lib.a` | 静态库 |
| `install/lidar_camera_fusion_demo` | 测试 Demo |
| `install/include/` | 头文件 |

## 快速开始

### 单相机

```cpp
#include "lidar_camera_fusion.h"

// 1. 创建实例（自动预分配缓冲区）
LidarCameraFusion fusion;

// 2. 配置相机参数
CameraConfig cam;
cam.fx = 400.0f;  cam.fy = 400.0f;      // 焦距（像素）
cam.cx = 320.0f;  cam.cy = 240.0f;      // 主点（像素）
cam.imgWidth  = 640;
cam.imgHeight = 480;
// 设置外参 4×4 矩阵（行主序）
// cam.tLidarToCam[0..15] = ...

// 3. 获取 YOLO 检测结果
std::vector<YoloBBox> detections = yolo_infer(...);

// 4. 获取雷达帧
LidarFrame frame;
frame.points = pre_allocated_points_buffer;
lidar.get_closest_frame(cameraTsNs, frame);

// 5. 融合
fusion.reset();
fusion.fuse_data(detections, cameraTsNs, frame, cam);

// 6. 读取结果
const FusionResult& r = fusion.result();
for (uint32_t i = 0; i < r.bboxCount; ++i) {
    uint32_t count = r.bboxPointCounts[i];
    // 计算偏移: offset = (i==0 ? 0 : offset + r.bboxPointCounts[i-1])
    // r.bboxPointIndices[offset .. offset+count] 即第 i 个 bbox 的雷达点索引
}
```

### 双相机累积

```cpp
CameraConfig frontCam, rearCam;
// ... 分别设置两个相机的外参和内参 ...

fusion.reset();
fusion.fuse_data(frontDetections, cameraTsNs, lidarFrame, frontCam);
fusion.fuse_data(rearDetections,  cameraTsNs, lidarFrame, rearCam);

const FusionResult& r = fusion.result();
// r.bboxCount = frontDetections.size() + rearDetections.size()
// bbox 0..nFront-1 来自前相机, bbox nFront.. 来自后相机
```


## 限制条件

- 仅支持单线激光雷达（N10Plus），雷达点 z 坐标恒为 0
- bbox 判定采用"首次命中"策略：一个雷达点若投影到多个 bbox 重叠区域，归入先匹配的 bbox
- `reset()` 必须在每帧新的雷达帧融合前调用；同一雷达帧的多次 `fuse_data()` 之间不要调用 `reset()`
- 结果在下一次 `reset()` 调用时被覆盖，需及时消费
- YOLO 检测结果的数据结构定义在此组件中，待 YOLO 模块正式实现后可统一
