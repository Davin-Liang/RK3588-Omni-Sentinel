# 温控调频系统设计

**日期**: 2026-07-19  
**状态**: 待实现

## 1. 动机

Omni-Sentinel 多传感器全功能运行时，CPU（3 簇 8 核）+ NPU（3 核）同时高负载，YOLO 推理 + LLM 推理 + RGA 图像处理 + MPP 编码持续运行，整板功耗和发热不可忽视。RK3588 内核虽有三层 thermal trip point（75/85/115°C）硬保底，但缺乏在用户态根据业务负载精细调控的手段——当前系统只能眼睁睁看着温度升高，直到内核强制降频才被动响应。

本项目需要一套用户态温控策略，在温度升高时主动、渐进地限制 CPU 和 NPU 的频率上限（不接管 governor），在保证系统安全的前提下尽可能维持关键业务性能。

## 2. 管控范围

基于实际板端 sysfs 探测（elf2-buildroot / RK3588），确定管控以下设备：

| 设备 | sysfs 节点 | 控制方式 | 档位数 |
|------|-----------|---------|--------|
| CPU A55 (核 0-3) | `/sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq` | 写频率上限 | 8 档 (408~1800 MHz) |
| CPU A76 (核 4-5) | `/sys/devices/system/cpu/cpufreq/policy4/scaling_max_freq` | 写频率上限 | 11 档 (408~2304 MHz) |
| CPU A76 (核 6-7) | `/sys/devices/system/cpu/cpufreq/policy6/scaling_max_freq` | 写频率上限 | 11 档 (408~2304 MHz) |
| NPU | `/sys/class/devfreq/fdab0000.npu/max_freq` | 写频率上限 | 8 档 (300~1000 MHz) |

### 不纳入管控

| 设备 | 原因 |
|------|------|
| GPU (Mali) | 项目未使用 OpenGL/Vulkan/OpenCL，无负载 |
| DDR (`dmc`) | 调频收益小，频率变化可能引起系统不稳定 |
| RGA | 仅有时钟门控，无 DVFS 频率节点，由内核驱动自动管理 |
| MPP (VEPU/VDPU) | 同上，仅有时钟门控，自动管理 |

### 控制方式原则

**不接管 governor，只调 `max_freq` 上限。**  
CPU governor 保持 `schedutil`，NPU governor 保持 `rknpu_ondemand`。温控策略只写频率天花板，内核 governor 在允许范围内自行根据负载调节。这样即使温控逻辑有 bug，governor 仍能正常工作，不会导致系统不可用。

## 3. 策略分级

4 级温度梯度 + 每级独立的回滞（hysteresis），避免频率在阈值边界反复跳变：

```
Normal:  T < warmThreshold           → 全速 (max = 硬件上限)
Warm:    T > warmThreshold  → 降频    → T < warmRecover  → 恢复
Hot:     T > hotThreshold   → 降频    → T < hotRecover   → 恢复
Critical: T > critThreshold → 极限降频 → T < critRecover → 恢复
           ↑ 同时触发内核 thermal trip point (soc-thermal: 75/85°C) 硬保底
```

### 默认参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `warmThreshold` | 65°C | 进入 Warm 的温度 |
| `warmRecover` | 60°C | 退出 Warm 的温度（回滞 5°C） |
| `hotThreshold` | 75°C | 进入 Hot 的温度 |
| `hotRecover` | 70°C | 退出 Hot 的温度（回滞 5°C） |
| `critThreshold` | 85°C | 进入 Critical 的温度 |
| `critRecover` | 80°C | 退出 Critical 的温度（回滞 5°C） |

### 默认频率上限

| 等级 | CPU A55 (kHz) | CPU A76 (kHz) | NPU (Hz) |
|------|--------------|---------------|----------|
| Normal | 1,800,000 | 2,304,000 | 1,000,000,000 |
| Warm | 1,400,000 | 1,800,000 | 800,000,000 |
| Hot | 1,000,000 | 1,200,000 | 600,000,000 |
| Critical | 600,000 | 800,000 | 300,000,000 |

### 频率写入策略

- CPU A76 两簇（policy4 + policy6）使用相同的 A76 频率上限，写入两份 sysfs 节点
- 写入前对比当前值，相同时跳过，减少写 sysfs 次数
- 写入失败只打 `[Thermal]` stderr 日志，不崩溃、不回滚

## 4. 组件结构

新增静态库组件 `thermal-controller`，仿照项目现有组件风格：

```
thermal-controller/
├── include/
│   └── thermal_controller.h     # 公共 API
├── src/
│   └── thermal_controller.cpp    # 核心实现
├── CMakeLists.txt
├── README.md
├── BUG_RECORD.md
└── docs/
    └── IMPLEMENTATION.md
```

### ThermalController 类

```cpp
struct ThermalConfig {
    bool enabled = true;
    int  intervalSec = 2;
    bool restoreOnExit = true;

    // 温度阈值 (°C)
    int warmThreshold = 65;
    int warmRecover   = 60;
    int hotThreshold  = 75;
    int hotRecover    = 70;
    int critThreshold = 85;
    int critRecover   = 80;

    // CPU A76 各等级频率上限 (kHz)
    int cpuBigNormal   = 2304000;
    int cpuBigWarm     = 1800000;
    int cpuBigHot      = 1200000;
    int cpuBigCritical = 800000;

    // CPU A55 各等级频率上限 (kHz)
    int cpuLittleNormal   = 1800000;
    int cpuLittleWarm     = 1400000;
    int cpuLittleHot      = 1000000;
    int cpuLittleCritical = 600000;

    // NPU 各等级频率上限 (Hz)
    int npuNormal   = 1000000000;
    int npuWarm     = 800000000;
    int npuHot      = 600000000;
    int npuCritical = 300000000;
};

class ThermalController {
public:
    explicit ThermalController(const ThermalConfig& cfg);
    ~ThermalController();  // 可选恢复 Normal 频率

    // 每 intervalSec 调用一次，采集温度 + 评估策略 + 写入频率
    void tick();

    // 只读查询
    int  currentTempC() const;
    const char* currentLevel() const;   // "Normal" / "Warm" / "Hot" / "Critical"
    int  cpuLittleFreq() const;         // kHz
    int  cpuBigFreq() const;            // kHz
    int  npuFreq() const;               // Hz

    // JSON 状态，供 REST API 使用
    std::string status_json() const;

private:
    ThermalConfig cfg_;
    int  tickCount_ = 0;
    int  tempC_ = -1;
    int  level_ = 0;          // 0=Normal, 1=Warm, 2=Hot, 3=Critical
    int  cpuBigFreq_ = -1;    // kHz, 缓存 policy4 cur_freq
    int  npuFreq_ = -1;       // Hz, 缓存 npu cur_freq

    void read_sensors_();
    void evaluate_and_apply_();
};
```

## 5. 采样子系统

### 温度采集

```
/sys/class/thermal/thermal_zone0/temp  (soc-thermal, 当前唯一使用)
```

后续可按需扩展读取多 zone（bigcore0/1-thermal, littlecore-thermal, center-thermal, gpu-thermal, npu-thermal），取最大值做决策。

### 频率采集（只读）

| 节点 | 用途 |
|------|------|
| `policy0/scaling_cur_freq` | 标题栏显示 CPU A55 当前频率 |
| `policy4/scaling_cur_freq` | 标题栏显示 CPU A76 当前频率 (取 policy4) |
| `fdab0000.npu/cur_freq` | 标题栏显示 NPU 当前频率 |

## 6. SentinelQT 集成

### hwLabel 显示扩展

```
现:  45°C  CPU 30  RGA 5/2/1  NPU 80/75/60
新:  45°C  Warm  CPU 30% 1.8G  RGA 5/2/1  NPU 80/75/60 0.8G
```

- 温度后追加策略等级（Normal/Warm/Hot/Critical），若 280px 吃紧则缩写为 N/W/H/C
- CPU 利用率后追加当前大核频率（简写 `1.8G`）
- NPU 利用率后追加当前频率（简写 `0.8G`），加 `%`
- 各字段间用一个空格分隔（原来用两个），节省 4px 宽度

### 数据源分工

温控类和 QT 各自负责不同的 sysfs 读取，避免同一秒内重复读同一个文件：

| 数据 | 读取者 | 原因 |
|------|--------|------|
| 温度 (`thermal_zone0/temp`) | **ThermalController** `tick()` | 策略决策需要，缓存后 getter 取出 |
| CPU 频率 (`policy4/cur_freq`) | **ThermalController** `tick()` | 策略需要感知当前频率，缓存后 getter 取出 |
| NPU 频率 (`npu/cur_freq`) | **ThermalController** `tick()` | 同上 |
| 策略等级 | **ThermalController** `tick()` | 策略评估产出 |
| CPU 利用率 (`/proc/stat`) | QT `update_hw_usage_()` | 温控类不需要，QT 继续原有 delta 计算 |
| RGA 利用率 (`rkrga/load`) | QT `update_hw_usage_()` | 温控类不需要 |
| NPU 利用率 (`rknpu/load`) | QT `update_hw_usage_()` | 温控类不需要 |

`tick()` 每次内置读取 temp + cur_freq（轻量 fopen/fscanf），无论是否执行策略评估。策略评估按 `intervalSec` 间隔执行，但缓存值每次 tick 都刷新。

### 定时器集成

- 复用现有 `clockTimer_`（1 秒），`update_hw_usage_()` 中调用 `thermalCtrl_->tick()`
- `tick()` 内部用间隔计数，每 `intervalSec` 秒执行一次策略评估和频率写入
- 温度和频率采集每次 tick 都做（轻量 `fopen`/`fscanf`），getter 返回缓存值

### 生命周期

- **构造**: `Widget` 构造时读取 `config.ini [Thermal]`，若 `enabled=true` 则创建 `ThermalController`。构造函数内立即将所有 `max_freq` 写回硬件上限（Normal 值），确保启动时从干净的全速状态开始（即使上次异常退出未恢复也能兜底）
- **运行**: `update_hw_usage_()` 中调用 `tick()`，标题栏显示
- **析构**: `ThermalController` 析构时若 `restoreOnExit=true`，将所有 `max_freq` 写回硬件上限值，确保程序退出后系统恢复全速

### enabled=false 行为

温控关闭时，ThermalController 仍读 sysfs 取温度和频率（标题栏正常显示），但跳过策略评估和频率写入：

| | 读温度/频率 | 标题栏显示 | 写频率上限 |
|------|:-:|:-:|:-:|
| `enabled=true` | ✅ | ✅ | ✅ |
| `enabled=false` | ✅ | ✅ | ❌ |
| 不创建实例 | ❌ | 保持旧格式 | ❌ |

`enabled=false` 时等级字段不显示（没有策略就没有等级），频率照常显示实际值。

### REST API（只读）

```
GET /api/v1/thermal/status
```

返回 JSON：
```json
{
  "ok": true,
  "enabled": true,
  "level": "Normal",
  "temp": 45,
  "freq": {
    "cpuLittle": 1200000,
    "cpuBig": 1800000,
    "npu": 1000000000
  }
}
```

### Web 前端

- 在 `index.html` 的 hwStats 区域同步显示策略等级和频率（复用现有 WebSocket 推送）
- 不提供配置修改入口，只做只读展示
- `get_status_json_()` 扩展 temperature/level/freq 字段

## 7. config.ini 配置节

```ini
[Thermal]
# 温控开关
enabled=true
# 策略评估间隔 (秒)
intervalSec=2
# 程序退出时是否恢复全速
restoreOnExit=true

# 温度阈值 (°C) — 含回滞
warmThreshold=65
warmRecover=60
hotThreshold=75
hotRecover=70
critThreshold=85
critRecover=80

# CPU A76 (big core, policy4 + policy6) 各等级频率上限 (kHz)
cpuBigNormal=2304000
cpuBigWarm=1800000
cpuBigHot=1200000
cpuBigCritical=800000

# CPU A55 (little core, policy0) 各等级频率上限 (kHz)
cpuLittleNormal=1800000
cpuLittleWarm=1400000
cpuLittleHot=1000000
cpuLittleCritical=600000

# NPU 各等级频率上限 (Hz)
npuNormal=1000000000
npuWarm=800000000
npuHot=600000000
npuCritical=300000000
```

### 配置读取与校验

- 所有参数通过 INI 文件解析，带默认值兜底
- 温度阈值需满足 `warmRecover < warmThreshold < hotRecover < hotThreshold < critRecover < critThreshold`，否则打日志并使用默认值
- 各等级频率值满足 `Normal ≥ Warm ≥ Hot ≥ Critical`（非递增则打日志使用默认值）
- 频率值校验在可用档位范围内，超出则 clamp 到最近有效值
- 校验失败不阻止程序启动，打印 stderr 警告后使用默认值

## 8. 错误处理

- 所有 sysfs 读写使用 `fopen`/`fprintf`，失败打 `[Thermal]` stderr 日志，不抛异常
- `tick()` 写频率失败不重试，等下一个周期再写，避免因临时故障导致持续写入
- 文件打开失败（如 devfreq 不存在）静默跳过该设备，不影响其他设备的频率写入
- 构造函数中 sysfs 探测失败将 `enabled` 置为 `false`，后续 `tick()` 立即返回

## 9. 不做的事情

- **不做 Web 热修改**——配置只通过 config.ini 修改，重启生效
- **不接管 governor**——只写 `max_freq`，不写 `scaling_setspeed`
- **不写 `/sys/class/devfreq/.../governor`**——保持内核默认 governor
- **不做 RGA / MPP / GPU 频率管控**——没有独立的 DVFS 节点
- **不做 DDR 频率管控**——收益小且有稳定性风险
- **不做风扇控制**——RK3588 通常无板载风扇，如有需要后续扩展
- **不做温度历史曲线**——第一阶段只做实时监控和策略控制，历史数据可在后续版本追加

## 10. 开发任务清单

| # | 任务 | 涉及文件 | 预估改动量 |
|---|------|---------|-----------|
| 1 | 创建 `thermal-controller` 组件骨架 | `CMakeLists.txt`, `thermal_controller.h/.cpp` | ~400 行 |
| 2 | 实现 sysfs 读写函数 | `thermal_controller.cpp` (包含在 #1 中) | — |
| 3 | 实现 4 级策略评估逻辑 | `thermal_controller.cpp` (包含在 #1 中) | — |
| 4 | SentinelQT 集成：构造/析构/tick | `widget.h`, `widget.cpp` | ~50 行 |
| 5 | hwLabel 扩展显示 | `widget.cpp` `update_hw_usage_()` | ~15 行 |
| 6 | config.ini 追加 `[Thermal]` 节 | `config.ini` | ~30 行 |
| 7 | REST API `/api/v1/thermal/status` | `widget.cpp` `register_routes_()` | ~20 行 |
| 8 | WebSocket 状态推送追加字段 | `widget.cpp` `get_status_json_()` | ~10 行 |
| 9 | Web 前端 hwStats 显示扩展 | `index.html` | ~10 行 |
| 10 | 编写组件文档 | `README.md`, `IMPLEMENTATION.md`, `BUG_RECORD.md` | ~250 行 |

预估总代码量：~800 行（含文档），改动集中在 `thermal-controller/` 新目录 + `SentinelQT/` 5 个文件局部修改。
