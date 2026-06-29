# sentinel-visioner 视觉为主 + IMU 辅助 EIS 接入说明

## 1. 方案目标

旧版防抖主要依赖外部 IMU 回调直接给出 `offsetX / offsetY`。这种方案对 IMU 与相机坐标映射要求很高，在“IMU 水平放置、双相机竖直放置、镜头朝左右”的结构中较难调试。

新版方案改为：

```text
视觉帧间运动估计为主
IMU 震动强度辅助调节平滑强度
```

视觉侧直接根据画面估计 `dx / dy / dtheta`，因此不需要先调通 `signX / signY / swapXY`。

## 2. 新增代码

新增文件：

```text
include/vision_eis.hpp
src/vision_eis.cpp
```

修改文件：

```text
include/sentinel-visioner.h
src/sentinel-visioner.cpp
src/demo1.cpp
src/demo3.cpp
CMakeLists.txt
```

## 3. 新增 API

```cpp
bool set_visual_eis_config(int camNum, const VisionEisConfig& config);
bool enable_visual_eis(int camNum, bool enable);
void set_imu_assist_callback(std::function<bool(uint64_t timestampUs,
                                                int camNum,
                                                VisionImuAssistState& state)> callback);
```

其中 `set_imu_assist_callback()` 是可选的。如果不设置，视觉 EIS 仍然可以独立运行，只是不会根据 IMU 震动等级自适应调整 alpha。

## 4. 捕获线程工作方式

捕获线程采用一帧延迟闭环：

```text
第 t 帧：使用第 t-1 帧估计得到的 offset 进行 RGA 防抖处理
第 t 帧 raw preview：送入 LK 光流模块，估计第 t+1 帧可用的 offset
```

这样可以避免“必须先处理完当前帧才能补偿当前帧”的循环依赖。

## 5. RGA 补偿方式

旧版 `rga_process_to_rgb_()` 通过移动目标 `drect` 实现 offset。但当 16:9 图像 letterbox 到 640x640 时，水平方向没有 padding，导致水平 offset 容易被夹成 0。

新版在 EIS 激活时使用：

```text
源图轻微 zoom + 移动源图裁剪窗口 + 输出居中 letterbox
```

这种方式为 X/Y 两个方向都预留了裁剪余量，更适合 EIS。

## 6. 单路相机测试

```bash
./sentinel_visioner_demo1 /dev/video11 60 1
```

参数含义：

```text
/dev/video11  摄像头节点
60            运行 60 秒
1             开启视觉 EIS，0 表示关闭
```

运行日志中会出现：

```text
[Visual EIS Cam 0] reliable=1 offset=(x,y) dxdy=(dx,dy) pts=... inliers=... alpha=... cost=...ms
```

## 7. 双路相机测试

```bash
./sentinel_visioner_demo3 /dev/video11 /dev/video21 60 1
```

注意：两路相机分别创建 `VisionEisStabilizer`，不能共用上一帧和轨迹状态。

## 8. IMU 如何接入

上层可以把 ICM45686 输出转换成：

```cpp
VisionImuAssistState state;
state.gyroNorm = ...;
state.gyroRms = ...;
state.vibrationLevel = ...; // 0低 1中 2高
```

然后：

```cpp
visioner.set_imu_assist_callback([](uint64_t timestampUs, int camNum,
                                    VisionImuAssistState& state) {
    // 从 IMU 读取线程获取最新震动状态
    state.gyroRms = latest.gyroRms;
    state.vibrationLevel = latest.vibrationLevel;
    return true;
});
```

## 9. 效果评价

仍然使用 `raw.mp4 / eis.mp4` 的画面指标：

```text
Translation RMS
Horizontal std
Vertical std
Rotation RMS
```

视觉 EIS 的过程指标建议同时记录：

```text
tracked_points
inlier_count
visual_reliable_rate
gyro_rms
vibration_level
used_alpha
avg_cost_ms
```
