# EIS 画面抖动 RMS 评估说明

## 1. 评估目的

`icm45686_eis_demo` 能证明 IMU 读取、EIS offset 计算和 CPU 开销正常，但它不能直接证明“画面真的变稳”。  
要评估真实防抖效果，需要对原始视频和防抖后视频分别计算相邻帧之间的全局运动，并比较平移抖动 RMS。

本工程新增 `icm45686_jitter_eval` 离线评估工具，用于计算：

```text
Translation RMS = sqrt(mean(dx^2 + dy^2))
Horizontal std  = std(dx)
Vertical std    = std(dy)
Rotation RMS    = sqrt(mean(dtheta^2))
Reduction       = (Raw - EIS) / Raw * 100%
```

## 2. 输入数据

需要准备两段同一场景下的视频：

```text
raw.mp4  原始未防抖视频
eis.mp4  防抖后视频
```

推荐测试场景：

1. 相机对准静态标定板、墙面纹理、AprilTag 或固定工业设备；
2. 给设备施加轻微震动；
3. 同时或先后录制 raw 和 eis 两路视频；
4. 两段视频帧率、分辨率、时长尽量保持一致。

## 3. 编译依赖

该工具依赖 OpenCV。  
如果 CMake 找不到 OpenCV，会跳过 `icm45686_jitter_eval`，不影响 `icm45686_app` 和 `icm45686_eis_demo` 编译。

Ubuntu PC 上可安装：

```bash
sudo apt-get install libopencv-dev
```

然后执行：

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

如果是在 RK3588 交叉编译环境中，需要确保 SDK/sysroot 中包含 OpenCV 开发库，否则建议将 raw/eis 视频拷贝到 PC 上离线评估。

## 4. 运行方法

```bash
./icm45686_jitter_eval raw.mp4 eis.mp4
```

只分析前 900 帧：

```bash
./icm45686_jitter_eval raw.mp4 eis.mp4 900
```

## 5. 输出示例

```text
================ EIS Jitter Evaluation ================
Valid frame pairs: raw=899/899, eis=899/899

Metric                              Raw            EIS      Reduction
--------------------------------------------------------------------------
Translation RMS                  8.600px       2.400px        72.09%
Horizontal std                   6.200px       1.800px        70.97%
Vertical std                     5.100px       1.500px        70.59%
Rotation RMS                     0.1200deg     0.0400deg      66.67%
--------------------------------------------------------------------------
Result: EIS reduces frame-to-frame translation jitter.
========================================================
```

## 6. 指标解释

| 指标 | 含义 |
|---|---|
| `Translation RMS` | 相邻帧全局平移抖动均方根，越小越稳定 |
| `Horizontal std` | 水平方向帧间位移标准差，越小越稳定 |
| `Vertical std` | 垂直方向帧间位移标准差，越小越稳定 |
| `Rotation RMS` | 相邻帧旋转抖动均方根，越小越稳定 |
| `Reduction` | 防抖后相对于原始视频的降低率，越大越好 |

## 7. 注意事项

1. 这个评估方法最适合静态场景。如果画面中有大面积运动目标，光流会受到干扰。
2. 视频中需要有足够纹理。如果是纯白墙或纹理很少，特征点不足，评估会失败。
3. 如果防抖后指标反而变差，需要检查 offset 符号、焦距参数、裁剪余量、IMU与相机时间同步。
4. 如果工具提示 valid frame pairs 很少，说明视频纹理不足或运动过大，需要更换测试场景。
5. 如果代码中启用了图像缩放，dx/dy 是缩放后图像上的像素位移。如果要得到原分辨率像素值，可删除 `preprocess_frame()` 中的缩放逻辑。
