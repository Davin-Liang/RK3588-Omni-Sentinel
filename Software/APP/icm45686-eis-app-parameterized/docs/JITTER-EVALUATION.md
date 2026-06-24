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

其中 `Raw` 表示原始未防抖视频的抖动指标，`EIS` 表示防抖后视频的抖动指标。

## 2. 输入数据要求

需要准备两段视频：

```text
raw.mp4  原始未防抖视频
eis.mp4  防抖后视频
```

为了保证评估结果公平，`raw.mp4` 和 `eis.mp4` 应尽量来自**同一次采集、同一相机、同一场景、同一帧序列**。最理想的生成方式是：同一个原始相机帧同时分成两路处理。

```text
Camera Frame i
    ├── Raw branch：offsetX = 0, offsetY = 0
    │       └── raw_frame_i
    └── EIS branch：offsetX = IMU-EIS计算值, offsetY = IMU-EIS计算值
            └── eis_frame_i
```

也就是说：

```text
raw_frame_i 和 eis_frame_i 必须尽量一一对应。
```

如果 `raw.mp4` 和 `eis.mp4` 是两次不同录制得到的，震动强度、光照、人员位置和设备姿态都可能不同，最终 `Reduction` 只能作为主观参考，不适合作为严谨指标。

推荐测试场景：

1. 相机对准静态标定板、墙面纹理、AprilTag 或固定工业设备；
2. 给设备施加轻微、可重复的工业振动；
3. 同时保存 raw 和 eis 两路视频，或者保存 raw 视频与对应的 `offset.csv` 后离线生成 eis 视频；
4. 两段视频的帧率、分辨率、帧数和起止时间应保持一致。

## 3. 单 IMU 与双相机参数配置原则

本项目中可能同时存在两路相机，例如：

```text
Camera0：15 FPS
Camera1：30 FPS
IMU：ICM45686，一个物理传感器
```

需要注意：**一个 IMU 不能同时配置两套 `gyroRange`、`sampleHz`、`accelRange`。**这些参数是 IMU 传感器或 IMU 读取线程的全局参数，同一时刻只能有一套。

### 3.1 IMU 全局参数只能配置一套

下面这些参数属于 `ImuConfig`，应在 IMU Reader 启动前统一设置：

| 参数 | 是否能按相机分别设置 | 说明 |
|---|---|---|
| `sampleHz` | 不能 | 一个 IMU 读取线程统一采样，例如 100Hz / 200Hz |
| `gyroRange` | 不能 | 陀螺仪量程是 IMU 寄存器配置，例如 ±250 / ±500 / ±1000 / ±2000 dps |
| `accelRange` | 不能 | 加速度计量程是 IMU 寄存器配置，例如 ±2G / ±4G / ±8G / ±16G |
| `gyroBiasX/Y/Z` | 通常不能 | 同一个 IMU 的静态零偏是一套，启动时静置标定得到 |
| IMU ODR / 滤波带宽 | 不能 | 如果驱动后续支持，也是传感器级配置 |

推荐做法是用一套较稳妥的 IMU 全局配置同时服务两路相机：

```cpp
ImuConfig imuConfig;
imuConfig.sampleHz = 200.0f;        // 统一 IMU 读取频率，兼顾 15FPS 和 30FPS
imuConfig.gyroRange = 1;            // 1 表示 ±500 dps，适合一般工业振动起步测试
imuConfig.accelRange = 1;           // 1 表示 ±4G
imuConfig.enableGyroBiasCalib = true;
imuConfig.biasCalibMs = 2000;       // 启动后静置 2 秒标定陀螺仪零偏

reader.configure(imuConfig);
reader.start(imuConfig.sampleHz);
```

如果现场震动很小，可以使用 `gyroRange = 0`，即 ±250 dps，以获得更高的小信号分辨率。  
如果现场震动较剧烈，出现陀螺仪饱和或 offset 异常跳变，可以提高到 `gyroRange = 2` 或 `gyroRange = 3`。

### 3.2 两路相机可以分别配置 EisCameraConfig

虽然 IMU 硬件参数只能一套，但**同一份 IMU 数据可以同时提供给 15FPS 和 30FPS 两个相机使用**。每路相机应单独配置 `EisCameraConfig`。

下面这些参数属于每路相机自己的防抖算法参数，可以分别设置：

| 参数 | 是否能按相机分别设置 | 说明 |
|---|---|---|
| `frameRate` | 能 | 例如一路 15FPS，另一路 30FPS |
| `focalX/focalY` | 能 | 两路镜头焦距或分辨率可能不同 |
| `halfWindowMs` | 能 | 15FPS 可以稍大，30FPS 可以稍小 |
| `maxOffsetPixel` | 能 | 由各自图像分辨率和裁剪余量决定 |
| `timeOffsetMs` | 能 | 两路相机采集延迟可能不同 |
| `signX/signY` | 能 | 前后相机安装方向可能不同 |
| `swapXY` | 能 | IMU 轴和图像轴映射可能不同 |
| `smoothingAlpha` | 能 | 15FPS 可更平滑，30FPS 可更灵敏 |

示例配置如下：

```cpp
// 15FPS 相机：帧间隔约 66.7ms，可以使用稍大的积分窗口和更强平滑
EisCameraConfig cam15Config;
cam15Config.camId = 0;
cam15Config.frameRate = 15.0f;
cam15Config.focalX = 1200.0f;
cam15Config.focalY = 1200.0f;
cam15Config.halfWindowMs = 25;
cam15Config.maxOffsetPixel = 120;
cam15Config.timeOffsetMs = 0.0f;
cam15Config.signX = -1.0f;
cam15Config.signY = 1.0f;
cam15Config.swapXY = false;
cam15Config.enableSmoothing = true;
cam15Config.smoothingAlpha = 0.35f;

// 30FPS 相机：帧间隔约 33.3ms，窗口略小，响应更快
EisCameraConfig cam30Config;
cam30Config.camId = 1;
cam30Config.frameRate = 30.0f;
cam30Config.focalX = 1200.0f;
cam30Config.focalY = 1200.0f;
cam30Config.halfWindowMs = 20;
cam30Config.maxOffsetPixel = 120;
cam30Config.timeOffsetMs = 0.0f;
cam30Config.signX = -1.0f;
cam30Config.signY = 1.0f;
cam30Config.swapXY = false;
cam30Config.enableSmoothing = true;
cam30Config.smoothingAlpha = 0.25f;
```

调用方式：

```cpp
int32_t offsetX15 = 0;
int32_t offsetY15 = 0;
int32_t offsetX30 = 0;
int32_t offsetY30 = 0;

// frameTimestampNs 应由对应相机帧提供，最好使用曝光中心时间戳。
// Demo 阶段没有真实相机时间戳时，可用 now - halfWindowMs 作为近似。
stabilizer.calculate_eis_offset(cam15Config, cam15FrameTimestampNs, offsetX15, offsetY15);
stabilizer.calculate_eis_offset(cam30Config, cam30FrameTimestampNs, offsetX30, offsetY30);
```

最终结构应理解为：

```text
一个 ICM45686 IMU Reader
    ├── 统一 ImuConfig：sampleHz / gyroRange / accelRange / gyroBias
    ↓
同一份 IMU 时间序列数据
    ├── Camera0：15FPS 的 EisCameraConfig → offsetX15 / offsetY15
    └── Camera1：30FPS 的 EisCameraConfig → offsetX30 / offsetY30
```

### 3.3 推荐起步参数

| 对象 | 参数 | 推荐起步值 | 说明 |
|---|---|---:|---|
| IMU | `sampleHz` | 200Hz | 同时兼顾 15FPS 和 30FPS |
| IMU | `gyroRange` | 1，即 ±500dps | 一般工业振动起步值 |
| IMU | `accelRange` | 1，即 ±4G | 比 ±2G 更适合轻微冲击场景 |
| 15FPS 相机 | `halfWindowMs` | 25ms | 获取更多 IMU 样本，画面更平滑 |
| 15FPS 相机 | `smoothingAlpha` | 0.35 | 平滑更明显 |
| 30FPS 相机 | `halfWindowMs` | 20ms | 响应更快 |
| 30FPS 相机 | `smoothingAlpha` | 0.25 | 保持灵敏度 |
| 每路相机 | `focalX/focalY` | 先用 1200 | 后续根据镜头标定修正 |
| 每路相机 | `maxOffsetPixel` | 80~120 | 不能超过视觉链路裁剪余量 |
| 每路相机 | `timeOffsetMs` | 0 起步 | 若画面慢半拍，再单独标定 |

## 4. 编译依赖

该工具依赖 OpenCV。  
如果 CMake 找不到 OpenCV，会跳过 `icm45686_jitter_eval`，不影响 `icm45686_app` 和 `icm45686_eis_demo` 编译。

Ubuntu PC 上可安装：

```bash
sudo apt-get install libopencv-dev
```

PC 端离线评估工具建议在 Ubuntu 上编译：

```bash
mkdir -p build_pc
cd build_pc
cmake ..
make -j4 icm45686_jitter_eval
```

如果使用 `./build.sh` 进行 RK3588 交叉编译，CMake 查找的是 ARM64 Buildroot sysroot 里的 OpenCV，而不是 Ubuntu 主机 OpenCV。如果 sysroot 没有 OpenCV，`icm45686_jitter_eval` 会被跳过，这是正常现象。

板端实时 IMU 程序仍然使用：

```bash
./build.sh
```

用于生成：

```text
icm45686_app
icm45686_eis_demo
```

## 5. 运行方法

```bash
./icm45686_jitter_eval raw.mp4 eis.mp4
```

只分析前 900 帧：

```bash
./icm45686_jitter_eval raw.mp4 eis.mp4 900
```

## 6. 输出示例

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

## 7. 指标解释

| 指标 | 含义 |
|---|---|
| `Translation RMS` | 相邻帧全局平移抖动均方根，越小越稳定 |
| `Horizontal std` | 水平方向帧间位移标准差，越小越稳定 |
| `Vertical std` | 垂直方向帧间位移标准差，越小越稳定 |
| `Rotation RMS` | 相邻帧旋转抖动均方根，越小越稳定 |
| `Reduction` | 防抖后相对于原始视频的降低率，越大越好 |

## 8. 注意事项

1. 这个评估方法最适合静态场景。如果画面中有大面积运动目标，光流会受到干扰。
2. 视频中需要有足够纹理。如果是纯白墙或纹理很少，特征点不足，评估会失败。
3. `raw.mp4` 和 `eis.mp4` 应尽量来自同一次采集。如果两段视频不是同一时间段，只能做主观参考。
4. 如果防抖后指标反而变差，需要检查 offset 符号、焦距参数、裁剪余量、IMU 与相机时间同步。
5. 如果工具提示 valid frame pairs 很少，说明视频纹理不足或运动过大，需要更换测试场景。
6. 如果代码中启用了图像缩放，dx/dy 是缩放后图像上的像素位移。如果要得到原分辨率像素值，可删除 `preprocess_frame()` 中的缩放逻辑。
7. `ImuConfig` 是全局配置，不要试图给 15FPS 和 30FPS 相机分别设置两套 IMU 硬件参数；应给两路相机分别设置 `EisCameraConfig`。

