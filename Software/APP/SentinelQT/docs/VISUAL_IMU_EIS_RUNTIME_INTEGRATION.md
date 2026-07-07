# Visual-Primary + IMU-Assisted EIS 集成说明

本次修改不是把防抖做成离线工具，而是把防抖算法接入到最终运行的 `SentinelQT` 主程序链路中。最终运行方式仍然是：

```bash
./SentinelQT -platform eglfs
```

## 1. 最终实时链路

```text
SentinelQT
  ├─ 按钮/接口启用某路相机 EIS
  ├─ 初始化 ICM45686 Reader，注册 IMU 辅助回调
  ↓
sentinel-visioner 采集线程
  ├─ 从摄像头获取 NV12 帧
  ├─ 转 RGB raw preview
  ├─ 使用 LK 光流 + RANSAC 估计画面 dx / dy / dtheta
  ├─ 读取 IMU gyroRms / vibrationLevel，动态选择平滑 alpha
  ├─ 计算下一帧 offsetX / offsetY
  ├─ 使用上一帧 offset 对当前帧做 RGA 裁剪/平移补偿
  └─ 输出防抖后的 preview / NPU 输入，并把 offset 元数据交给后续推流/录像链路
```

## 2. 关键变化

### SentinelQT

- `Widget::on_btn_eis_()` 不再调用旧的 IMU offset 防抖，而是调用：
  - `visioner_->set_visual_eis_config(camNum, visualEisCfg_[camNum])`
  - `visioner_->enable_visual_eis(camNum, true/false)`
- `init_eis_()` 只负责初始化 ICM45686 并注册 `set_imu_assist_callback()`。
- `imu_assist_callback_()` 输出 `gyroNorm / gyroRms / vibrationLevel`，不再输出 `offsetX / offsetY`。
- `config.ini` 中 `[EIS]` 已改成视觉 EIS 参数，例如 `Cam0ProcessWidth`、`Cam0AlphaLow`、`Cam0MaxOffsetPixel` 等。

### sentinel-visioner

- 新增 `include/vision_eis.hpp` 和 `src/vision_eis.cpp`。
- `CameraContext` 每路相机独立维护 `VisionEisStabilizer`，双相机互不干扰。
- 采集线程中真正执行实时视觉 EIS。
- SentinelQT UI 预览输出现在会显示防抖后的画面。

### icm45686-eis-app

- 新增 `ImuAssistState` 和 `Icm45686Reader::getAssistState()`。
- IMU 模块定位从“直接算 offset”调整为“震动感知与 EIS 辅助”。

## 3. 编译位置

将三个目录放在同一父目录下：

```text
APP/
├── SentinelQT/
├── sentinel-visioner/
└── icm45686-eis-app/
```

然后编译最终界面：

```bash
cd APP/SentinelQT
./build.sh
```

运行：

```bash
./SentinelQT -platform eglfs
```

## 4. OpenCV 说明

LK 光流和 RANSAC 视觉 EIS 依赖 OpenCV，但不是深度学习模型，不需要训练模型。`sentinel-visioner` 会链接项目中的 `../3rdparty/opencv`。

## 5. 调试建议

1. 启动 SentinelQT 后先确认两路相机正常预览。
2. 点击“防抖开”。
3. 观察终端日志：

```text
[Visual EIS Cam 0] reliable=1 offset=(...,...) dxdy=(...,...) pts=... inliers=... alpha=... cost=...ms
```

4. 若 `pts` 或 `inliers` 很低，说明画面纹理不足，需要对准纹理更丰富的场景。
5. 若 offset 经常达到 `MaxOffsetPixel`，说明补偿量被限幅，需要增加裁剪余量或降低震动幅度。
