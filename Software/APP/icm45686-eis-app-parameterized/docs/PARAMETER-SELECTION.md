# ICM45686 EIS 防抖参数选取说明

本文档用于说明 `icm45686-eis-app` 中会影响电子图像防抖（EIS）效果的参数、推荐取值、双相机场景下的配置方法，以及参数调试顺序。当前工程中防抖参数被拆成两类：

```text
ImuConfig        ：IMU 全局配置，影响 IMU 数据本身；
EisCameraConfig  ：单路相机 EIS 配置，影响某一路相机的 offset 计算。
```

这种设计适合双相机场景。例如，一个相机为 15FPS，另一个相机为 30FPS。两路相机可以共享同一个 IMU Reader，但各自使用不同的 `EisCameraConfig`。

---

## 1. 参数分层原则

### 1.1 IMU 全局参数

IMU 全局参数通常在程序启动时设置一次，不建议每帧动态修改。它们影响的是 IMU 采样频率、量程和陀螺仪零偏。

```cpp
struct ImuConfig {
    float sampleHz;              // 应用层IMU读取频率
    uint8_t gyroRange;           // 陀螺仪量程
    uint8_t accelRange;          // 加速度计量程

    bool enableGyroBiasCalib;    // 是否启用启动时静置零偏标定
    uint32_t biasCalibMs;        // 零偏标定时长

    float gyroBiasX;             // X轴陀螺仪零偏
    float gyroBiasY;             // Y轴陀螺仪零偏
    float gyroBiasZ;             // Z轴陀螺仪零偏
};
```

### 1.2 单路相机参数

相机参数每路相机各维护一份。即使两个相机使用同一颗 IMU，它们的帧率、焦距、安装方向和时间延迟也可能不同。

```cpp
struct EisCameraConfig {
    int camId;                   // 摄像头编号
    float frameRate;             // 相机帧率
    float focalX;                // 水平焦距
    float focalY;                // 垂直焦距

    uint32_t halfWindowMs;       // 积分半窗口
    int32_t maxOffsetPixel;      // 最大补偿像素

    float timeOffsetMs;          // 相机与IMU时间偏差修正

    float signX;                 // X方向补偿符号
    float signY;                 // Y方向补偿符号
    bool swapXY;                 // 是否交换X/Y轴映射

    bool enableSmoothing;        // 是否启用offset平滑
    float smoothingAlpha;        // 平滑系数
};
```

---

## 2. 当前算法中各参数的作用

### 2.1 `sampleHz`：IMU 应用层读取频率

`sampleHz` 决定应用层每秒读取多少条 IMU 数据。它会影响每一帧 EIS 计算窗口内可用的 IMU 样本数量。

| 相机帧率 | 帧间隔 | 100Hz IMU 每帧约可覆盖样本 | 建议 |
|---|---:|---:|---|
| 15FPS | 66.7ms | 6~7 条 | 100Hz 可用，推荐 100~200Hz |
| 30FPS | 33.3ms | 3~4 条 | 100Hz 可用，推荐 200Hz 更稳 |

建议：

```text
基础 Demo：100Hz
30FPS 防抖：100Hz 起步，推荐 200Hz
工业高频震动：200Hz 或 400Hz
```

注意：当前 `sampleHz` 是应用层读取 `/dev/icm45686` 的频率。若后续驱动支持 ODR 配置，还应保证芯片 ODR 与应用读取频率匹配。

---

### 2.2 `gyroRange`：陀螺仪量程

`gyroRange` 决定 IMU 能测量的最大角速度。如果实际震动角速度超过当前量程，陀螺仪会饱和，EIS offset 会失真。

| 参数值 | 量程 | 适用场景 |
|---:|---|---|
| 0 | ±250DPS | 轻微震动、静态测试、小角度防抖 |
| 1 | ±500DPS | 一般工业设备震动，推荐起步值 |
| 2 | ±1000DPS | 较剧烈震动 |
| 3 | ±2000DPS | 快速冲击或极剧烈旋转 |

建议：

```text
如果只是桌面静态测试：gyroRange=0
如果安装在工业设备上：gyroRange=1
如果出现gyro数据接近上限或补偿异常：提高到2或3
```

量程越大，小角速度分辨率越低，因此不要盲目设置最大量程。

---

### 2.3 `accelRange`：加速度计量程

当前 EIS offset 主要依赖陀螺仪短时间积分，加速度计不是主要补偿量来源。加速度计主要用于判断静止状态、重力方向、冲击强度和后续姿态融合。

| 参数值 | 量程 | 适用场景 |
|---:|---|---|
| 0 | ±2G | 静态姿态、轻微运动 |
| 1 | ±4G | 一般震动，推荐工业场景起步值 |
| 2 | ±8G | 较强震动 |
| 3 | ±16G | 强冲击或剧烈运动 |

建议：

```text
普通测试：accelRange=0
工业设备震动：accelRange=1
如果加速度模长经常明显超过量程：提高到2或3
```

---

### 2.4 `gyroBiasX/Y/Z`：陀螺仪零偏

陀螺仪静止时不一定严格为 0。若不扣除零偏，积分后 offset 可能缓慢漂移。

当前工程支持两种方式：

```text
1. 手动配置 gyroBiasX/Y/Z；
2. 启动时启用静置标定 enableGyroBiasCalib + biasCalibMs。
```

推荐启动时静置标定：

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 200 30 20 1 1 120 0 -1 1 0 0.30 2000
```

最后的 `2000` 表示标定 2 秒。标定期间必须保持 IMU 静止。

判断是否需要零偏标定：

```text
静止时 gyro norm 不接近 0；
静止时 offset 慢慢漂移；
长时间运行后 max_abs_offset 缓慢变大。
```

---

## 3. 单路相机参数说明

### 3.1 `frameRate`：相机帧率

两路相机帧率不同时，EIS 参数不应该完全相同。

| 相机 | 帧间隔 | 推荐 halfWindowMs |
|---|---:|---:|
| 15FPS | 66.7ms | 20~30ms |
| 30FPS | 33.3ms | 15~20ms |

15FPS 相机帧间隔更长，画面本身更容易看到跳动和运动模糊。EIS 可以补偿帧间位置抖动，但不能消除曝光期间已经产生的模糊。

---

### 3.2 `halfWindowMs`：IMU 积分半窗口

EIS 会在目标帧时间戳附近取一段 IMU 样本进行积分：

```text
[start, end] = [targetTimestamp - halfWindowMs, targetTimestamp + halfWindowMs]
```

对 100Hz IMU，采样间隔约 10ms：

| halfWindowMs | 总窗口 | 100Hz 下样本数 | 说明 |
|---:|---:|---:|---|
| 5ms | 10ms | 约 1 条 | 不够积分，不推荐 |
| 10ms | 20ms | 约 2 条 | 勉强可用 |
| 20ms | 40ms | 约 3~5 条 | 推荐 |
| 30ms | 60ms | 约 5~7 条 | 更平滑，但延迟更大 |

推荐：

```text
30FPS：15~20ms
15FPS：20~30ms
```

如果 `used_samples < 2`，说明窗口太小或时间戳不对；如果防抖有明显迟滞，说明窗口可能过大或时间同步偏差未补偿。

---

### 3.3 `focalX / focalY`：相机焦距

EIS 将角度变化转换为像素偏移：

```text
offsetX ≈ focalX × thetaY
offsetY ≈ focalY × thetaX
```

因此焦距直接影响补偿幅度。

| 焦距设置 | 现象 |
|---|---|
| 太小 | offset 太小，视觉上防抖不明显 |
| 太大 | offset 太大，画面过补偿甚至更抖 |
| 正确 | 轻微转动时 offset 与画面抖动幅度匹配 |

Demo 中默认 `1200/1200 pixel` 只是测试值。正式项目应根据相机内参填写真实焦距，或者根据视场角和分辨率估算。

---

### 3.4 `maxOffsetPixel`：最大补偿像素

`maxOffsetPixel` 限制输出 offset 范围：

```text
offsetX ∈ [-maxOffsetPixel, maxOffsetPixel]
offsetY ∈ [-maxOffsetPixel, maxOffsetPixel]
```

它必须小于视觉链路的裁剪余量。否则会出现黑边或裁剪越界。

| 裁剪余量 | 推荐 maxOffsetPixel |
|---:|---:|
| 32px | 20~30px |
| 64px | 40~60px |
| 128px | 80~120px |
| 256px | 150~220px |

如果实际抖动需要 150px 补偿，但 `maxOffsetPixel=50`，防抖会补不全。如果设置过大但视觉链路没有裁剪余量，会导致画面边界问题。

---

### 3.5 `timeOffsetMs`：相机与 IMU 时间偏移

这是实际防抖效果中非常关键的参数。若 IMU 数据和相机帧时间不同步，即使 offset 幅值正确，也会出现补偿滞后或超前。

```text
targetTimestamp = frameTimestamp + timeOffsetMs
```

现象判断：

| 现象 | 可能原因 |
|---|---|
| 防抖像“慢半拍” | timeOffsetMs 需要调小或调大 |
| 开启后反而更抖 | 时间错位或方向错误 |
| offset 有变化但画面改善不明显 | 时间戳未对齐 |

建议调试方法：

```text
从 0ms 开始；
按 -30ms、-20ms、-10ms、0ms、+10ms、+20ms、+30ms 扫描；
选择画面RMS抖动最低的一组。
```

15FPS 和 30FPS 相机的 pipeline 延迟可能不同，因此 `timeOffsetMs` 应每路相机单独标定。

---

### 3.6 `signX / signY / swapXY`：轴向与符号

IMU 坐标系、相机坐标系、图像坐标系不一定一致。当前默认映射为：

```text
thetaY -> offsetX
thetaX -> offsetY
signX = -1
signY = +1
```

如果发现开启 EIS 后画面更抖，优先检查符号：

```text
signX 从 -1 改为 +1；
signY 从 +1 改为 -1；
必要时启用 swapXY。
```

推荐调试方式：

```text
1. 固定相机，对准清晰纹理或标定板；
2. 只绕一个方向缓慢转动；
3. 观察 raw/eis 或 offset变化；
4. 若补偿方向反了，切换 sign；
5. 若X/Y方向对应不上，启用 swapXY。
```

双相机前后安装方向不同，`signX/signY/swapXY` 通常需要分别配置。

---

### 3.7 `smoothingAlpha`：offset 平滑系数

当前支持一阶低通平滑：

```text
smooth = alpha * current + (1 - alpha) * previous
```

| alpha | 效果 |
|---:|---|
| 0 | 关闭平滑 |
| 0.1~0.2 | 很平滑，但延迟较明显 |
| 0.25~0.5 | 推荐范围 |
| 0.7~1.0 | 响应快，但抖动抑制弱 |

建议：

```text
30FPS：0.25~0.35
15FPS：0.30~0.50
```

如果画面残余高频抖动明显，可以减小 alpha；如果防抖明显滞后，可以增大 alpha 或关闭平滑。

---

## 4. 双相机推荐初始配置

### 4.1 全局 IMU 配置

```cpp
ImuConfig imuConfig;
imuConfig.sampleHz = 200.0f;
imuConfig.gyroRange = 1;       // ±500DPS
imuConfig.accelRange = 1;      // ±4G
imuConfig.enableGyroBiasCalib = true;
imuConfig.biasCalibMs = 2000;
```

说明：

```text
200Hz 比 100Hz 有更高时间分辨率；
±500DPS 适合一般工业震动；
±4G 给加速度计留出一定震动余量；
启动2秒零偏标定减少静态漂移。
```

### 4.2 15FPS 相机

```cpp
EisCameraConfig cam15;
cam15.camId = 0;
cam15.frameRate = 15.0f;
cam15.focalX = 1200.0f;
cam15.focalY = 1200.0f;
cam15.halfWindowMs = 25;
cam15.maxOffsetPixel = 120;
cam15.timeOffsetMs = 0.0f;
cam15.signX = -1.0f;
cam15.signY = 1.0f;
cam15.swapXY = false;
cam15.enableSmoothing = true;
cam15.smoothingAlpha = 0.35f;
```

### 4.3 30FPS 相机

```cpp
EisCameraConfig cam30;
cam30.camId = 1;
cam30.frameRate = 30.0f;
cam30.focalX = 1200.0f;
cam30.focalY = 1200.0f;
cam30.halfWindowMs = 20;
cam30.maxOffsetPixel = 120;
cam30.timeOffsetMs = 0.0f;
cam30.signX = -1.0f;
cam30.signY = 1.0f;
cam30.swapXY = false;
cam30.enableSmoothing = true;
cam30.smoothingAlpha = 0.25f;
```

> 上述焦距、符号和最大补偿量只是起步值。正式项目应结合真实相机内参、安装方向和裁剪余量标定。

---

## 5. Demo 命令示例

### 5.1 默认 30FPS 测试

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20
```

### 5.2 30FPS 工业震动起步配置

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 200 30 20 1 1 120 0 -1 1 0 0.25 2000
```

参数含义：

```text
200       IMU读取频率200Hz
30        模拟30FPS相机
20        半窗口20ms
1         gyroRange=±500DPS
1         accelRange=±4G
120       maxOffset=120px
0         timeOffset=0ms
-1 1      signX=-1, signY=+1
0         不交换X/Y轴
0.25      启用平滑，alpha=0.25
2000      启动时静置标定2秒
```

### 5.3 15FPS 相机起步配置

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 200 15 25 1 1 120 0 -1 1 0 0.35 2000
```

差异：

```text
frameRate=15
halfWindowMs=25
smoothingAlpha=0.35
```

---

## 6. 参数调试顺序

建议按以下顺序调试，不要一次性同时改所有参数。

### 第一步：确认 IMU 数据正常

运行基础 Demo：

```bash
/usr/bin/icm45686_app
```

确认：

```text
accel norm ≈ 9.8~10.0
gyro norm 静止时接近 0
temp 稳定
```

### 第二步：确认 EIS 链路成功

运行：

```bash
/usr/bin/icm45686_eis_demo /dev/icm45686 30 1200 1200 100 30 20
```

确认：

```text
success_rate≈100%
used_samples≥3
failed_imu=0
avg_cost很低
```

### 第三步：调 `gyroRange` 和 `sampleHz`

若震动较强：

```text
gyroRange 从0调到1；
sampleHz 从100调到200；
若仍饱和，再考虑gyroRange=2。
```

### 第四步：调 `signX/signY/swapXY`

若开启防抖后更抖，优先调方向：

```text
先改signX；
再改signY；
最后尝试swapXY。
```

### 第五步：调 `focalX/focalY`

若 offset 太小，适当增大焦距；若过补偿，适当减小焦距。

### 第六步：调 `timeOffsetMs`

如果 offset 有变化但画面改善不明显，扫描时间偏移：

```text
-30ms, -20ms, -10ms, 0ms, +10ms, +20ms, +30ms
```

选择画面抖动 RMS 最低的配置。

### 第七步：调 `maxOffsetPixel`

结合视觉链路裁剪余量设置。不要让 `maxOffsetPixel` 大于可用裁剪边界。

### 第八步：调 `smoothingAlpha`

如果 offset 抖动导致画面残余小抖动，降低 alpha；如果延迟明显，增大 alpha。

---

## 7. 常见问题

### 7.1 为什么 15FPS 和 30FPS 不能共用同一个配置？

因为 15FPS 的帧间隔是 66.7ms，30FPS 的帧间隔是 33.3ms。相机曝光时间、pipeline 延迟、帧时间戳和视觉感知效果都不同。尤其是 `halfWindowMs`、`timeOffsetMs` 和 `smoothingAlpha` 建议分开配置。

### 7.2 为什么参数设置后静止时 offset 仍为 0？

这是正常现象。静止时陀螺仪角速度接近 0，积分角度也接近 0，因此像素补偿量应接近 0。

### 7.3 为什么动态测试时 offset 很小？

可能原因：

```text
转动幅度太小；
focalX/focalY 太小；
gyroRange 过大导致小角速度分辨率降低；
halfWindowMs 太小；
单位换算或轴向映射仍需确认。
```

### 7.4 为什么防抖后画面更抖？

优先排查：

```text
signX/signY 是否反了；
swapXY 是否需要开启；
timeOffsetMs 是否错位；
RGA 是否真正使用了最新 offset；
maxOffsetPixel 是否过大导致画面跳动。
```

### 7.5 为什么大震动时防抖失效？

可能超过了算法能力边界：

```text
陀螺仪量程饱和；
maxOffsetPixel 限幅过小；
图像裁剪余量不足；
震动主要是平移而不是旋转；
曝光期间已经产生运动模糊；
rolling shutter 果冻效应无法由全局平移消除。
```

---

## 8. 推荐验收指标

参数调好后，建议使用以下指标评定效果：

| 类别 | 指标 | 说明 |
|---|---|---|
| 链路稳定性 | `success_rate` | EIS计算成功率，应接近100% |
| 样本充足性 | `used_samples` | 每帧积分样本数，建议≥3 |
| 实时性 | `avg_cost/max_cost` | 单帧EIS计算耗时 |
| 资源占用 | `cpu` | 进程CPU占用 |
| 视觉效果 | 帧间位移 RMS 降低率 | 需要 raw/eis 同步视频 |
| AI稳定性 | 检测框中心抖动降低率 | 需要接入视觉检测结果 |

对于当前 IMU 模块负责范围，至少应保证：

```text
IMU读取稳定；
EIS计算成功率接近100%；
动态转动时offset能响应；
CPU开销较低；
参数结构支持15FPS/30FPS分别配置。
```
