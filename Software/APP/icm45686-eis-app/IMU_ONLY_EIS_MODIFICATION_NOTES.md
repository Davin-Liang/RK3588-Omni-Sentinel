# IMU-only EIS 修改说明

本版本把防抖入口从“视觉 LK 光流 + IMU 辅助”切回 IMU-only 电子防抖退化版。

## 核心链路

```text
IMU gyro raw
  -> R_B_imu_raw
  -> qRaw_B / qSmooth_B
  -> R_delta_C
  -> H = K * R_delta_C * K^-1
  -> center point displacement
  -> offsetX / offsetY
  -> 当前 RGA crop 链路继续使用
```

当前版本仍输出 offsetX/offsetY 给现有 RGA crop 使用，所以只能主要补偿 pitch/yaw 导致的中心点平移；roll 抖动需要下一步使用 H 做 warpPerspective/affine warp 才能真正补偿。

## 用户实测 IMU 原始坐标到 B 坐标映射

B 坐标定义：

```text
+X_B：图中向右，指向 cam1
+Y_B：垂直图纸平面向外
+Z_B：图中向上
```

采用实测映射：

```text
gyro_B.x = -gyro_raw.y
gyro_B.y = -gyro_raw.x
gyro_B.z =  gyro_raw.z
```

对应配置：

```ini
Cam0RBimu00=0
Cam0RBimu01=-1
Cam0RBimu02=0
Cam0RBimu10=-1
Cam0RBimu11=0
Cam0RBimu12=0
Cam0RBimu20=0
Cam0RBimu21=0
Cam0RBimu22=1
```

cam1 默认使用同一套 IMU raw -> B 映射。

## 相机安装外参

cam0：左侧相机，光轴向左。

```text
IMU -> cam0 光心：(-0.175, 0.000, 0.070) m
R_C0_B: x_C=+Y_B, y_C=-Z_B, z_C=-X_B
```

cam1：右侧相机，光轴向右。

```text
IMU -> cam1 光心：(0.010, 0.000, 0.070) m
R_C1_B: x_C=-Y_B, y_C=-Z_B, z_C=+X_B
```

## 杆臂补偿

已在配置和结构体中保留 `t_B / EnableLeverArm / NominalDepthMeter`，但当前实现默认不启用杆臂补偿。原因是杆臂补偿需要假设场景深度，深度不准时可能变差。建议先用纯旋转 EIS 跑通，再测试 cam0 的杆臂补偿。

## 调试日志

开启 `Cam0ImuOnlyDebug=true` 后会打印：

```text
[IMU-only EIS Cam 0] offset=(x,y) gyroRaw=(...) gyroB=(...) gyroCam=(...) roll=... cost=... samples=...
```

单轴验证预期：

- cam0 yaw：offsetX 明显变化，offsetY/roll 较小；
- cam0 pitch：offsetY 明显变化，offsetX/roll 较小；
- cam0 roll：roll 明显变化，offsetX/offsetY 不应是主变化。

如果 roll 抖动是主要抖动来源，仅靠 offsetX/offsetY 的 RGA crop 结果不会明显改善，需要下一阶段接入 H warp。
