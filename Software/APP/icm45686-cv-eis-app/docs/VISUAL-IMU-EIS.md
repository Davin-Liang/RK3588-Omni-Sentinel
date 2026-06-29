# 视觉为主 + IMU 辅助 EIS 实现说明

## 1. 为什么改成这个方案

旧方案是纯 IMU 开环补偿：

```text
ICM45686 gyro -> 积分角度 -> 按相机焦距换算 offsetX/offsetY -> RGA 平移
```

这种方案依赖 IMU 与相机坐标系严格对应。当前设备中 ICM45686 与 RK3588 水平放置，而两个相机竖直放置、镜头分别朝左/朝右，实际轴映射较复杂，导致 `signX/signY/swapXY` 难以调通。

新方案改为：

```text
视觉帧间运动估计为主
IMU 震动强度辅助调参
```

也就是说，最终 offset 主要由画面本身的帧间运动决定，IMU 不再直接决定画面平移方向。

## 2. LK 光流 / ORB 是否需要模型

不需要。LK 光流和 ORB 都是传统计算机视觉算法，不是深度学习模型。

本工程第一版使用 LK 光流：

```text
上一帧灰度图 -> 提取角点
当前帧灰度图 -> LK 跟踪角点
点对 -> RANSAC 估计全局仿射运动
得到 dx / dy / dtheta
```

## 3. 代码新增内容

### 3.1 新增 `include/vision_eis.hpp`

定义：

- `VisionImuAssistState`：IMU 辅助状态，包括 `gyroNorm`、`gyroRms`、`vibrationLevel`；
- `VisionEisConfig`：每路相机的视觉 EIS 参数；
- `VisionEisResult`：每帧视觉估计结果；
- `VisionEisStabilizer`：核心视觉防抖类。

### 3.2 新增 `src/vision_eis.cpp`

实现：

1. 帧转灰度和缩放；
2. `goodFeaturesToTrack()` 提取角点；
3. `calcOpticalFlowPyrLK()` 跟踪角点；
4. `estimateAffinePartial2D()` + RANSAC 估计全局运动；
5. 累计运动轨迹；
6. 根据 IMU 震动等级选择轨迹平滑 alpha；
7. 输出 `offsetX / offsetY`。

### 3.3 新增 `src/vision_eis_offline_demo.cpp`

该工具用于 PC 端离线验证：

```bash
./icm45686_vision_eis_offline raw.mp4 eis_visual.mp4
./icm45686_jitter_eval raw.mp4 eis_visual.mp4
```

它可以先不依赖 RK3588、不依赖 `/dev/icm45686`，直接验证视觉主导 EIS 是否能降低画面 RMS 抖动。

## 4. IMU 在新方案中的作用

IMU 输出不再直接作为最终 offset，而是提供：

```text
gyroNorm        当前角速度模长
gyroRms         最近窗口内角速度 RMS
vibrationLevel  0低震动 / 1中震动 / 2高震动
```

视觉 EIS 根据震动等级选择不同平滑强度：

| 震动等级 | 建议 alpha | 含义 |
|---|---:|---|
| 低震动 | 0.30 | 更跟手，少补偿 |
| 中震动 | 0.20 | 常规防抖 |
| 高震动 | 0.12 | 更强平滑 |

注意：这里 `alpha` 越小，轨迹越平滑，防抖越强，但可能带来更明显的滞后。

## 5. 15FPS / 30FPS 双相机如何配置

一个 IMU 仍然只有一套硬件参数：

```text
sampleHz / gyroRange / accelRange / gyroBias 只能统一配置一套
```

但是两路相机必须分别建立视觉 EIS 状态：

```cpp
VisionEisStabilizer cam0Stabilizer(cam0Config);  // 15FPS
VisionEisStabilizer cam1Stabilizer(cam1Config);  // 30FPS
```

推荐起步配置：

```cpp
// 15FPS 相机：更强平滑
cam15.alphaLowVibration  = 0.25f;
cam15.alphaMidVibration  = 0.18f;
cam15.alphaHighVibration = 0.10f;
cam15.maxOffsetPixel = 80;

// 30FPS 相机：响应可以更快
cam30.alphaLowVibration  = 0.30f;
cam30.alphaMidVibration  = 0.22f;
cam30.alphaHighVibration = 0.15f;
cam30.maxOffsetPixel = 80;
```

## 6. sentinel-visioner 集成方式

`sentinel-visioner` 中新增：

```cpp
bool set_visual_eis_config(int camNum, const VisionEisConfig& config);
bool enable_visual_eis(int camNum, bool enable);
void set_imu_assist_callback(...);
```

捕获线程中的工作方式：

```text
当前帧：使用上一帧视觉估计得到的 offset 做 RGA 防抖处理
当前 raw preview：送入 VisionEisStabilizer，估计下一帧 offset
```

这是一帧延迟闭环，避免视觉估计必须等当前帧处理完才能再补当前帧的问题。

## 7. RGA 补偿方式变化

旧逻辑主要移动目标 `drect`。当 16:9 输入缩放到 640x640 时，输出宽度已占满，水平方向没有 padding，导致水平 offset 容易被夹成 0。

新逻辑在 EIS 激活时改为：

```text
源图轻微 zoom -> 移动源图裁剪窗口 -> 输出居中 letterbox
```

这样 X/Y 两个方向都有补偿空间，更接近实际 EIS 的“保留边缘裁剪余量”做法。

## 8. 如何验证效果

先用离线工具验证：

```bash
./icm45686_vision_eis_offline raw.mp4 eis_visual.mp4
./icm45686_jitter_eval raw.mp4 eis_visual.mp4
```

再在板端开启 `sentinel_visioner_demo1` 的视觉 EIS：

```bash
./sentinel_visioner_demo1 /dev/video11 60 1
```

双路相机：

```bash
./sentinel_visioner_demo3 /dev/video11 /dev/video21 60 1
```

最后仍然使用 `raw.mp4 / eis.mp4` 的 Translation RMS、Horizontal std、Vertical std、Rotation RMS 评价防抖效果。
