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
