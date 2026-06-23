# BUG_RECORD - RK3588 + ICM45686 调试记录

## 1. `/dev/icm45686` 未生成

### 现象

```bash
ls -la /dev/icm45686
ls: cannot access '/dev/icm45686': No such file or directory
```

内核日志曾出现：

```bash
icm45686_spi spi4.0: Failed to get reset GPIO
```

### 原因

驱动最初强制申请 reset GPIO，但硬件未实际连接 RESET，且设备树 GPIO 编号写法存在问题，导致 `probe()` 中断，字符设备未创建。

### 解决

1. 将 reset GPIO 改为 optional；
2. 未连接 RESET 时跳过硬件复位；
3. 若后续连接 RESET，使用实际引出的 GPIO，例如 `GPIO3_B0`。

---

## 2. WHO_AM_I 读取失败

### 现象

```bash
Invalid WHO_AM_I value: 0x00 (expected 0x68)
icm45686_spi spi4.0: Failed to initialize ICM45686
```

### 原因

原驱动使用了旧芯片寄存器定义：

```c
#define ICM45686_REG_WHO_AM_I 0x0F
#define ICM45686_WHO_AM_I_VAL 0x68
```

而 ICM45686 实际为：

```c
#define ICM45686_REG_WHO_AM_I 0x72
#define ICM45686_WHO_AM_I_VAL 0xE9
```

### 解决

修正 WHO_AM_I 地址和值后，驱动初始化成功：

```bash
ICM45686 initialized successfully
icm45686_spi spi4.0: ICM45686 SPI driver loaded successfully
```

---

## 3. 数据全为 0

### 现象

```text
Accel: 0.00 0.00 0.00
Gyro : 0.00 0.00 0.00
Temp : 25.00
```

### 原因

只修正 WHO_AM_I 后，其他寄存器地址仍是旧芯片地址，例如数据寄存器仍从 `0x3B` 读取。

### 解决

修正 ICM45686 的数据寄存器和配置寄存器：

```c
ACCEL_DATA 起始地址 = 0x00
GYRO_DATA 起始地址  = 0x06
TEMP_DATA 起始地址  = 0x0C
PWR_MGMT0           = 0x10
ACCEL_CONFIG0       = 0x1B
GYRO_CONFIG0        = 0x1C
```

---

## 4. 静置数据异常跳变

### 现象

静置状态下，加速度和温度大幅跳变：

```text
Accel: 12.72 17.62 -18.04
Temp : 57.02 / 73.02 / 105.02
```

### 原因

ICM45686 默认数据为 Little Endian，但驱动曾按 Big Endian 解析。

### 解决

改为 Little Endian 组包：

```c
raw = low_byte | (high_byte << 8)
```

修复后静置数据稳定，加速度模长接近 `9.8 m/s²`。

---

## 5. EIS Demo 首次 success=0

### 现象

```text
success=0
used_samples=1
avg_cost=0.000 ms
latest_offset=(0,0)
```

### 原因

100Hz IMU 的采样周期约为 10ms，原 Demo 使用 `halfWindowMs=5ms`，总窗口只有 10ms，通常只能取到 1 条样本，无法做陀螺积分。

### 解决

1. 将 `halfWindowMs` 改为 20ms；
2. 目标帧时间戳使用 `now - halfWindowMs`，保证窗口落在历史 IMU 数据中；
3. 增加 `success_rate / failed_eis / used_samples / max_abs_offset` 等测试指标。

修复后：

```text
success_rate=100.00%
failed_eis=0
used_samples≈4
```

---

## 6. SentinelQT 集成: EIS 回调被 NPU buffer 条件跳过

**现象**: EIS 初始化成功、IMU 正常采样（samples=4），但 `offset` 始终为 0。

**原因**: 回调注入点放在 `capture_thread_func_` 的 `if (targetNpuBuf != nullptr)` 块内。NPU 推理未启动时 `npuRgbPool` 耗尽后此条件恒假，回调永不执行。

**解决**: 将 EIS 偏移计算提升到 `if (targetNpuBuf != nullptr)` 外部，每帧独立执行。偏移值仍仅在有 NPU buffer 时传入 `rga_process_to_rgb_()`。

---

## 7. SentinelQT 集成: Web EIS 按钮路由未注册

**现象**: 网页点击 EIS 按钮返回"网络错误"。

**原因**: cpp-httplib 使用显式路由注册，`web_server.cpp` 中未注册 `/api/v1/cam/{0,1}/eis/start|stop` 四条 POST 路由。

**解决**: 在 `web_server.cpp` 中注册四条 EIS POST 路由。

---

## 8. SentinelQT 集成: `setAxisSign()` 两路相机竞态

**现象**: 两路相机同时开启 EIS 时，采集线程并发调用 `EisStabilizer::setAxisSign()` 修改 `signX_/signY_` 成员（无 mutex 保护）。

**原因**: 方案初版在回调内每帧调用 `setAxisSign()` 适配 per-camera 符号，两路采集线程访问同一 `EisStabilizer` 实例。

**解决**: init 时设 `signX=1.0, signY=1.0`，回调内对 `calculate_eis_offset()` 结果手动乘 per-camera 符号。避免回调内写操作。
