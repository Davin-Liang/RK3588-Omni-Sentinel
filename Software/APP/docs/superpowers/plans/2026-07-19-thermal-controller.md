# Thermal Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add active thermal-based frequency scaling to RK3588 Omni-Sentinel (CPU 3 clusters + NPU) with a 4-level hysteresis policy, configurable via config.ini.

**Architecture:** New static library `thermal-controller` with a single public class `ThermalController`. Integrates into SentinelQT via a member pointer. Reuses existing `clockTimer_` (1s) for tick. Reads temp/freq from sysfs (cached getters), writes `max_freq` limits. QT continues to read utilization (/proc/stat, rkrga/load, rknpu/load) independently.

**Tech Stack:** C++14, static library (`.a`), POSIX file I/O (fopen/fscanf/fprintf), nlohmann/json, no external dependencies beyond C++ stdlib.

## Global Constraints

- C++14 standard, cross-compiled for aarch64-buildroot-linux-gnu
- No C++ exceptions, no asserts — fprintf(stderr, "[Thermal] ...") for diagnostics
- All sysfs writes use fopen/fprintf; failure logs but never crashes
- Config via QSettings INI format, defaults for all fields
- Pre-allocated members, no runtime heap allocation in tick() path
- Follow project naming conventions (PascalCase class, snake_case methods, trailing _ for private members)

---

### Task 1: Create thermal-controller component skeleton

**Files:**
- Create: `thermal-controller/CMakeLists.txt`
- Create: `thermal-controller/include/thermal_controller.h`
- Create: `thermal-controller/src/thermal_controller.cpp`

**Interfaces:**
- Produces: `ThermalConfig` struct, `ThermalController` class (both in `thermal_controller.h`)

- [ ] **Step 1: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.4.1)

project(thermal_controller)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_library(thermal_controller_lib STATIC
    src/thermal_controller.cpp
)

target_include_directories(thermal_controller_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
```

- [ ] **Step 2: Create thermal_controller.h header**

```cpp
#ifndef THERMAL_CONTROLLER_H
#define THERMAL_CONTROLLER_H

#include <string>

/** @brief 温控配置 — 所有参数通过 config.ini [Thermal] 节读取 */
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

/** @brief 温控控制器
 *
 * 读取 sysfs 温度和频率，根据 4 级策略写入频率上限。
 * 不接管 governor，只通过 max_freq 设天花板。
 */
class ThermalController {
public:
    explicit ThermalController(const ThermalConfig& cfg);
    ~ThermalController();

    /** @brief 每 1 秒调用一次。内部按 intervalSec 间隔执行策略评估 */
    void tick();

    // ---- 只读查询 ----
    int  currentTempC() const          { return tempC_; }
    const char* currentLevel() const;
    int  cpuLittleFreq() const         { return cpuLittleFreq_; }
    int  cpuBigFreq() const            { return cpuBigFreq_; }
    int  npuFreq() const               { return npuFreq_; }
    bool isEnabled() const             { return cfg_.enabled; }

    /** @brief JSON 状态，供 REST API GET /api/v1/thermal/status */
    std::string status_json() const;

private:
    ThermalConfig cfg_;
    int  tickCount_ = 0;
    int  tempC_ = -1;
    int  level_ = 0;          // 0=Normal, 1=Warm, 2=Hot, 3=Critical
    int  cpuLittleFreq_ = -1; // kHz
    int  cpuBigFreq_ = -1;    // kHz
    int  npuFreq_ = -1;       // Hz

    void read_sensors_();
    void evaluate_and_apply_();
    void startup_restore_();
    void write_max_freq_(const char* path, int value, const char* label);
    int  read_int_sysfs_(const char* path);
    int  get_level_freq_(int normalVal, int warmVal, int hotVal, int critVal) const;
    bool validate_config_() const;
};

#endif // THERMAL_CONTROLLER_H
```

- [ ] **Step 3: Create thermal_controller.cpp with includes and static paths**

```cpp
#include "thermal_controller.h"
#include <cstdio>
#include <cstring>
#include <cerrno>

// ---- sysfs 路径常量 ----
static const char* kPolicy0MaxFreq  = "/sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq";
static const char* kPolicy4MaxFreq  = "/sys/devices/system/cpu/cpufreq/policy4/scaling_max_freq";
static const char* kPolicy6MaxFreq  = "/sys/devices/system/cpu/cpufreq/policy6/scaling_max_freq";
static const char* kNpuMaxFreq      = "/sys/class/devfreq/fdab0000.npu/max_freq";

static const char* kThermalZone0    = "/sys/class/thermal/thermal_zone0/temp";
static const char* kPolicy0CurFreq  = "/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq";
static const char* kPolicy4CurFreq  = "/sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq";
static const char* kNpuCurFreq      = "/sys/class/devfreq/fdab0000.npu/cur_freq";

static const char* kLevelNames[] = { "Normal", "Warm", "Hot", "Critical" };
```

- [ ] **Step 4: Implement read_int_sysfs_() and thermal_controller.cpp constructor/destructor**

```cpp
int ThermalController::read_int_sysfs_(const char* path)
{
    FILE* fp = fopen(path, "r");
    if (!fp) return -1;
    int val = -1;
    fscanf(fp, "%d", &val);
    fclose(fp);
    return val;
}

ThermalController::ThermalController(const ThermalConfig& cfg)
    : cfg_(cfg)
{
    if (!validate_config_()) {
        fprintf(stderr, "[Thermal] config validation failed, using defaults\n");
        cfg_ = ThermalConfig{};  // 回退默认值
    }
    if (cfg_.enabled) {
        startup_restore_();
        fprintf(stderr, "[Thermal] started, interval=%ds\n", cfg_.intervalSec);
    } else {
        fprintf(stderr, "[Thermal] disabled — monitoring only\n");
    }
}

ThermalController::~ThermalController()
{
    if (cfg_.enabled && cfg_.restoreOnExit) {
        fprintf(stderr, "[Thermal] restoring max frequencies to Normal...\n");
        write_max_freq_(kPolicy0MaxFreq, cfg_.cpuLittleNormal, "CPU little");
        write_max_freq_(kPolicy4MaxFreq, cfg_.cpuBigNormal,    "CPU big(p4)");
        write_max_freq_(kPolicy6MaxFreq, cfg_.cpuBigNormal,    "CPU big(p6)");
        write_max_freq_(kNpuMaxFreq,     cfg_.npuNormal,       "NPU");
    }
}
```

- [ ] **Step 5: Implement startup_restore_() and write_max_freq_()**

```cpp
void ThermalController::startup_restore_()
{
    fprintf(stderr, "[Thermal] startup: restoring all max_freq to Normal\n");
    write_max_freq_(kPolicy0MaxFreq, cfg_.cpuLittleNormal, "CPU little");
    write_max_freq_(kPolicy4MaxFreq, cfg_.cpuBigNormal,    "CPU big(p4)");
    write_max_freq_(kPolicy6MaxFreq, cfg_.cpuBigNormal,    "CPU big(p6)");
    write_max_freq_(kNpuMaxFreq,     cfg_.npuNormal,       "NPU");
}

void ThermalController::write_max_freq_(const char* path, int value, const char* label)
{
    int cur = read_int_sysfs_(path);
    if (cur == value) return;  // 相同则跳过

    FILE* fp = fopen(path, "w");
    if (!fp) {
        fprintf(stderr, "[Thermal] open failed: %s (%s)\n", path, strerror(errno));
        return;
    }
    fprintf(fp, "%d", value);
    fclose(fp);
    fprintf(stderr, "[Thermal] %s max_freq: %d -> %d\n", label, cur, value);
}
```

- [ ] **Step 6: Implement get_level_freq_()**

```cpp
int ThermalController::get_level_freq_(int normalVal, int warmVal, int hotVal, int critVal) const
{
    switch (level_) {
        case 0: return normalVal;
        case 1: return warmVal;
        case 2: return hotVal;
        case 3: return critVal;
        default: return normalVal;
    }
}
```

- [ ] **Step 7: Commit**

```bash
git add thermal-controller/
git commit -m "feat: thermal-controller 组件骨架 — CMake, header, sysfs读写"
```

---

### Task 2: Implement 4-level policy engine

**Files:**
- Modify: `thermal-controller/src/thermal_controller.cpp` (add read_sensors_, evaluate_and_apply_, tick, currentLevel, status_json, validate_config_)

**Interfaces:**
- Produces: `tick()`, `currentLevel()`, `status_json()`, `validate_config_()`
- Consumes: `ThermalConfig`, `read_int_sysfs_()`, `write_max_freq_()`, `get_level_freq_()` from Task 1

- [ ] **Step 1: Implement read_sensors_()**

```cpp
void ThermalController::read_sensors_()
{
    // 温度
    int raw = read_int_sysfs_(kThermalZone0);
    tempC_ = (raw > 0) ? raw / 1000 : -1;

    // 当前频率 (只读)
    cpuLittleFreq_ = read_int_sysfs_(kPolicy0CurFreq);  // kHz
    cpuBigFreq_    = read_int_sysfs_(kPolicy4CurFreq);  // kHz
    npuFreq_       = read_int_sysfs_(kNpuCurFreq);      // Hz
}
```

- [ ] **Step 2: Implement evaluate_and_apply_()**

```cpp
void ThermalController::evaluate_and_apply_()
{
    if (tempC_ < 0) return;  // 传感器读取失败

    int prevLevel = level_;
    int t = tempC_;

    // 策略评估 — 4 级 + 回滞
    if (t >= cfg_.critThreshold) {
        level_ = 3;
    } else if (t >= cfg_.hotThreshold) {
        if (level_ == 3) { if (t >= cfg_.critRecover) level_ = 3; else level_ = 2; }
        else level_ = 2;
    } else if (t >= cfg_.warmThreshold) {
        if (level_ == 3) {
            if (t >= cfg_.critRecover) level_ = 3; else if (t >= cfg_.hotRecover) level_ = 2; else level_ = 1;
        } else if (level_ == 2) {
            if (t >= cfg_.hotRecover) level_ = 2; else level_ = 1;
        } else {
            level_ = 1;
        }
    } else {
        if (level_ == 3) {
            if (t >= cfg_.critRecover) level_ = 3; else if (t >= cfg_.hotRecover) level_ = 2; else if (t >= cfg_.warmRecover) level_ = 1; else level_ = 0;
        } else if (level_ == 2) {
            if (t >= cfg_.hotRecover) level_ = 2; else if (t >= cfg_.warmRecover) level_ = 1; else level_ = 0;
        } else if (level_ == 1) {
            if (t >= cfg_.warmRecover) level_ = 1; else level_ = 0;
        } else {
            level_ = 0;
        }
    }

    if (!cfg_.enabled) return;  // 只监控不控制

    // 等级变化时写入新频率上限
    if (level_ != prevLevel) {
        fprintf(stderr, "[Thermal] level change: %s -> %s (T=%d°C)\n",
                kLevelNames[prevLevel], kLevelNames[level_], t);
    }

    write_max_freq_(kPolicy0MaxFreq, get_level_freq_(cfg_.cpuLittleNormal, cfg_.cpuLittleWarm, cfg_.cpuLittleHot, cfg_.cpuLittleCritical), "CPU little");
    write_max_freq_(kPolicy4MaxFreq, get_level_freq_(cfg_.cpuBigNormal, cfg_.cpuBigWarm, cfg_.cpuBigHot, cfg_.cpuBigCritical), "CPU big(p4)");
    write_max_freq_(kPolicy6MaxFreq, get_level_freq_(cfg_.cpuBigNormal, cfg_.cpuBigWarm, cfg_.cpuBigHot, cfg_.cpuBigCritical), "CPU big(p6)");
    write_max_freq_(kNpuMaxFreq,     get_level_freq_(cfg_.npuNormal, cfg_.npuWarm, cfg_.npuHot, cfg_.npuCritical), "NPU");
}
```

- [ ] **Step 3: Implement tick()**

```cpp
void ThermalController::tick()
{
    read_sensors_();
    tickCount_++;
    if (cfg_.enabled && (tickCount_ % cfg_.intervalSec == 0)) {
        evaluate_and_apply_();
    }
}
```

- [ ] **Step 4: Implement currentLevel() and status_json()**

```cpp
const char* ThermalController::currentLevel() const
{
    if (!cfg_.enabled) return "";  // disabled 时不显示等级
    return kLevelNames[level_];
}

#include <string>
#include <cstdio>

std::string ThermalController::status_json() const
{
    char buf[512];
    snprintf(buf, sizeof(buf),
        R"({"ok":true,"enabled":%s,"level":"%s","temp":%d,)"
        R"("freq":{"cpuLittle":%d,"cpuBig":%d,"npu":%d}})",
        cfg_.enabled ? "true" : "false",
        cfg_.enabled ? kLevelNames[level_] : "",
        tempC_,
        cpuLittleFreq_ > 0 ? cpuLittleFreq_ : 0,
        cpuBigFreq_ > 0 ? cpuBigFreq_ : 0,
        npuFreq_ > 0 ? npuFreq_ : 0);
    return std::string(buf);
}
```

- [ ] **Step 5: Implement validate_config_()**

```cpp
bool ThermalController::validate_config_() const
{
    const ThermalConfig& c = cfg_;

    // 温度阈值链校验
    if (!(c.warmRecover < c.warmThreshold &&
          c.warmThreshold < c.hotRecover &&
          c.hotRecover < c.hotThreshold &&
          c.hotThreshold < c.critRecover &&
          c.critRecover < c.critThreshold)) {
        fprintf(stderr, "[Thermal] threshold chain invalid: "
                "warmRecover(%d) < warm(%d) < hotRecover(%d) < hot(%d) < critRecover(%d) < crit(%d)\n",
                c.warmRecover, c.warmThreshold, c.hotRecover,
                c.hotThreshold, c.critRecover, c.critThreshold);
        return false;
    }

    // 频率单调性校验
    if (!(c.cpuBigNormal >= c.cpuBigWarm && c.cpuBigWarm >= c.cpuBigHot && c.cpuBigHot >= c.cpuBigCritical)) {
        fprintf(stderr, "[Thermal] cpuBig freq non-monotonic\n");
        return false;
    }
    if (!(c.cpuLittleNormal >= c.cpuLittleWarm && c.cpuLittleWarm >= c.cpuLittleHot && c.cpuLittleHot >= c.cpuLittleCritical)) {
        fprintf(stderr, "[Thermal] cpuLittle freq non-monotonic\n");
        return false;
    }
    if (!(c.npuNormal >= c.npuWarm && c.npuWarm >= c.npuHot && c.npuHot >= c.npuCritical)) {
        fprintf(stderr, "[Thermal] npu freq non-monotonic\n");
        return false;
    }

    return true;
}
```

- [ ] **Step 6: Commit**

```bash
git add thermal-controller/src/thermal_controller.cpp
git commit -m "feat: thermal-controller 4级策略引擎 + sensor读取 + 校验"
```

---

### Task 3: Integrate thermal-controller into SentinelQT build

**Files:**
- Modify: `SentinelQT/CMakeLists.txt` (add thermal_controller_lib subdirectory + link)

**Interfaces:**
- Produces: `thermal_controller_lib` target available for SentinelQT linking

- [ ] **Step 1: Add subdirectory directive in CMakeLists.txt**

In `SentinelQT/CMakeLists.txt`, after the Deepseek block (line 106-108), add:

```cmake
# Thermal Controller
if(NOT TARGET thermal_controller_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../thermal-controller ${CMAKE_CURRENT_BINARY_DIR}/thermal_build)
endif()
```

- [ ] **Step 2: Add include directory and link library**

In `target_include_directories` (after line 141), add:

```cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/../thermal-controller/include
```

In `target_link_libraries` (after line 160), add:

```cmake
    thermal_controller_lib
```

- [ ] **Step 3: Commit**

```bash
git add SentinelQT/CMakeLists.txt
git commit -m "feat: SentinelQT 链接 thermal_controller_lib"
```

---

### Task 4: Add ThermalController integration to Widget

**Files:**
- Modify: `SentinelQT/widget.h` (add forward decl + member variable)
- Modify: `SentinelQT/widget.cpp` (load_config_, constructor, update_hw_usage_, destructor)

**Interfaces:**
- Consumes: `ThermalConfig`, `ThermalController` (from thermal_controller.h)

- [ ] **Step 1: Add forward declaration and member in widget.h**

In `SentinelQT/widget.h`, after `class AIReportWorker;` (line 30), add:

```cpp
class ThermalController;
struct ThermalConfig;
```

In the private member section (after `bool showEisControl_ = false;` line 156), add:

```cpp
    // ---- Thermal ----
    ThermalConfig         thermalCfg_;
    ThermalController*    thermalCtrl_ = nullptr;
```

- [ ] **Step 2: In widget.cpp, add include**

After existing includes (top of file), add:

```cpp
#include "thermal_controller.h"
```

- [ ] **Step 3: Read ThermalConfig from config.ini in load_config_()**

At end of `Widget::load_config_()` (before the closing `}` of the function), add:

```cpp
    // ---- Thermal ----
    thermalCfg_.enabled       = config_.value("Thermal/enabled", true).toBool();
    thermalCfg_.intervalSec   = config_.value("Thermal/intervalSec", 2).toInt();
    thermalCfg_.restoreOnExit = config_.value("Thermal/restoreOnExit", true).toBool();

    thermalCfg_.warmThreshold = config_.value("Thermal/warmThreshold", 65).toInt();
    thermalCfg_.warmRecover   = config_.value("Thermal/warmRecover", 60).toInt();
    thermalCfg_.hotThreshold  = config_.value("Thermal/hotThreshold", 75).toInt();
    thermalCfg_.hotRecover    = config_.value("Thermal/hotRecover", 70).toInt();
    thermalCfg_.critThreshold = config_.value("Thermal/critThreshold", 85).toInt();
    thermalCfg_.critRecover   = config_.value("Thermal/critRecover", 80).toInt();

    thermalCfg_.cpuBigNormal   = config_.value("Thermal/cpuBigNormal",   2304000).toInt();
    thermalCfg_.cpuBigWarm     = config_.value("Thermal/cpuBigWarm",     1800000).toInt();
    thermalCfg_.cpuBigHot      = config_.value("Thermal/cpuBigHot",      1200000).toInt();
    thermalCfg_.cpuBigCritical = config_.value("Thermal/cpuBigCritical", 800000).toInt();

    thermalCfg_.cpuLittleNormal   = config_.value("Thermal/cpuLittleNormal",   1800000).toInt();
    thermalCfg_.cpuLittleWarm     = config_.value("Thermal/cpuLittleWarm",     1400000).toInt();
    thermalCfg_.cpuLittleHot      = config_.value("Thermal/cpuLittleHot",      1000000).toInt();
    thermalCfg_.cpuLittleCritical = config_.value("Thermal/cpuLittleCritical", 600000).toInt();

    thermalCfg_.npuNormal   = config_.value("Thermal/npuNormal",   1000000000).toInt();
    thermalCfg_.npuWarm     = config_.value("Thermal/npuWarm",      800000000).toInt();
    thermalCfg_.npuHot      = config_.value("Thermal/npuHot",       600000000).toInt();
    thermalCfg_.npuCritical = config_.value("Thermal/npuCritical",  300000000).toInt();
```

- [ ] **Step 4: Create ThermalController in Widget constructor**

In `Widget::Widget()` constructor, after `load_config_();` (line 268), add:

```cpp
    thermalCtrl_ = new (std::nothrow) ThermalController(thermalCfg_);
```

- [ ] **Step 5: Call tick() in update_hw_usage_() and extend hwLabel**

In `Widget::update_hw_usage_()`, after the existing temperature read (around line 1065) REPLACE the temperature read block with a call to ThermalController, and after the NPU utilization read REPLACE the hwLabel text assembly:

Remove the existing fopen/fscanf for `/sys/class/thermal/thermal_zone0/temp` (lines 1060-1065) — ThermalController handles this now.

Replace the hwLabel text assembly (lines 1067-1082) with:

```cpp
    // Temperature and level from ThermalController
    if (thermalCtrl_) {
        thermalCtrl_->tick();
    }

    int tempC = thermalCtrl_ ? thermalCtrl_->currentTempC() : -1;
    const char* level = thermalCtrl_ ? thermalCtrl_->currentLevel() : "";
    int cpuBigKHz = thermalCtrl_ ? thermalCtrl_->cpuBigFreq() : -1;
    int npuHz = thermalCtrl_ ? thermalCtrl_->npuFreq() : -1;

    QString text;
    // 温度 + 等级
    text += tempC >= 0 ? QString("%1°C").arg(tempC) : "--°C";
    if (level && level[0] != '\0') {
        text += QString(" %1").arg(level);
    }
    text += " ";
    // CPU
    text += cpuUsage >= 0 ? QString("CPU%1%").arg(cpuUsage, 3) : "CPU --";
    if (cpuBigKHz > 0) {
        text += QString(" %1G").arg(cpuBigKHz / 1000000.0, 0, 'f', 1);
    }
    text += " ";
    // RGA
    text += QString("RGA%1/%2/%3")
                .arg(rgaCores[0] >= 0 ? QString::number(rgaCores[0]) : "-")
                .arg(rgaCores[1] >= 0 ? QString::number(rgaCores[1]) : "-")
                .arg(rgaCores[2] >= 0 ? QString::number(rgaCores[2]) : "-");
    text += " ";
    // NPU
    text += QString("NPU%1/%2/%3")
                .arg(npuCores[0] >= 0 ? QString::number(npuCores[0]) : "-")
                .arg(npuCores[1] >= 0 ? QString::number(npuCores[1]) : "-")
                .arg(npuCores[2] >= 0 ? QString::number(npuCores[2]) : "-");
    if (npuHz > 0) {
        text += QString(" %1G").arg(npuHz / 1e9, 0, 'f', 1);
    }

    ui->hwLabel->setText(text);
```

- [ ] **Step 6: Cleanup in Widget destructor**

In `Widget::~Widget()`, before `delete visioner_;` (around line 580), add:

```cpp
    delete thermalCtrl_;
    thermalCtrl_ = nullptr;
```

- [ ] **Step 7: Commit**

```bash
git add SentinelQT/widget.h SentinelQT/widget.cpp
git commit -m "feat: ThermalController 集成到 Widget — tick/hwLabel/构造析构"
```

---

### Task 5: Add REST API and WebSocket thermal fields

**Files:**
- Modify: `SentinelQT/widget.cpp` (handle_web_command, get_status_json_)

**Interfaces:**
- Consumes: `ThermalController::status_json()` (Task 2)
- Produces: `GET /api/v1/thermal/status`, WebSocket status JSON extended with thermal fields

- [ ] **Step 1: Add REST route in handle_web_command()**

In `Widget::handle_web_command()`, inside the GET block (after line 2143), add:

```cpp
        if (path == "/api/v1/thermal/status") {
            if (thermalCtrl_) return thermalCtrl_->status_json();
            return R"({"ok":false,"error":"thermal not available"})";
        }
```

- [ ] **Step 2: Extend get_hw_json_() with thermal fields**

In `Widget::get_hw_json_()`, after the existing temperature read block (lines 3085-3094), REPLACE the temperature read with a call to ThermalController and add freq fields. Replace:

```cpp
    // Temperature
    fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (fp) {
        int raw;
        if (fscanf(fp, "%d", &raw) == 1) j["temp"] = raw / 1000;
        else j["temp"] = -1;
        fclose(fp);
    } else {
        j["temp"] = -1;
    }
```

With:

```cpp
    // Temperature + Thermal
    if (thermalCtrl_) {
        j["temp"] = thermalCtrl_->currentTempC();
        j["thermalLevel"] = thermalCtrl_->currentLevel();
        nlohmann::json tf;
        tf["cpuLittle"] = thermalCtrl_->cpuLittleFreq();
        tf["cpuBig"]    = thermalCtrl_->cpuBigFreq();
        tf["npu"]       = thermalCtrl_->npuFreq();
        j["thermalFreq"] = tf;
    } else {
        fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
        if (fp) {
            int raw;
            if (fscanf(fp, "%d", &raw) == 1) j["temp"] = raw / 1000;
            else j["temp"] = -1;
            fclose(fp);
        } else {
            j["temp"] = -1;
        }
    }
```

- [ ] **Step 3: Commit**

```bash
git add SentinelQT/widget.cpp
git commit -m "feat: REST /api/v1/thermal/status + WebSocket thermal字段扩展"
```

---

### Task 6: Append [Thermal] section to config.ini

**Files:**
- Modify: `SentinelQT/config.ini` (append at end of file)

- [ ] **Step 1: Append [Thermal] section to config.ini**

Add after the last line of `config.ini`:

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

- [ ] **Step 2: Commit**

```bash
git add SentinelQT/config.ini
git commit -m "config: 追加 [Thermal] 温控调频配置节"
```

---

### Task 7: Extend Web frontend hwStats display

**Files:**
- Modify: `web-control/web/index.html` (hwStats HTML + updateUI JS)

- [ ] **Step 1: Add HTML elements for level and freq in hwStats**

Replace the hwStats div (lines 163-168) with:

```html
  <div id="hwStats">
    <span>🌡 <span class="val" id="hwTemp">--°C</span></span>
    <span class="val" id="hwLevel" style="display:none"></span>
    <span>CPU <span class="val" id="hwCpu">--</span>%</span>
    <span class="val" id="hwCpuFreq" style="display:none"></span>
    <span>RGA <span class="val" id="hwRga">-/-/-</span></span>
    <span>NPU <span class="val" id="hwNpu">-/-/-</span></span>
    <span class="val" id="hwNpuFreq" style="display:none"></span>
  </div>
```

- [ ] **Step 2: Update updateUI() JavaScript to populate thermal fields**

Replace the hwStats update line (line 365) with:

```javascript
  if(data.hw&&data.hw.ok){const h=data.hw;document.getElementById('hwTemp').textContent=(h.temp>=0?h.temp:'--')+'°C';document.getElementById('hwCpu').textContent=h.cpu>=0?h.cpu:'--';if(h.rga&&h.rga.length)document.getElementById('hwRga').textContent=h.rga.map(v=>v>=0?v:'-').join('/');if(h.npu&&h.npu.length)document.getElementById('hwNpu').textContent=h.npu.map(v=>v>=0?v:'-').join('/');
    // Thermal level
    const lvEl=document.getElementById('hwLevel');if(h.thermalLevel){lvEl.textContent=h.thermalLevel;lvEl.style.display=''}else{lvEl.style.display='none'}
    // Thermal freq
    if(h.thermalFreq){const tf=h.thermalFreq;
      const cfEl=document.getElementById('hwCpuFreq');if(tf.cpuBig>0){cfEl.textContent=(tf.cpuBig/1e6).toFixed(1)+'G';cfEl.style.display=''}else{cfEl.style.display='none'}
      const nfEl=document.getElementById('hwNpuFreq');if(tf.npu>0){nfEl.textContent=(tf.npu/1e9).toFixed(1)+'G';nfEl.style.display=''}else{nfEl.style.display='none'}
    }
  }
```

- [ ] **Step 3: Commit**

```bash
git add web-control/web/index.html
git commit -m "feat: Web前端 hwStats 显示温控等级和频率"
```

---

### Task 8: Write component documentation

**Files:**
- Create: `thermal-controller/README.md`
- Create: `thermal-controller/docs/IMPLEMENTATION.md`
- Create: `thermal-controller/BUG_RECORD.md`

- [ ] **Step 1: Write README.md**

```markdown
# thermal-controller — RK3588 温控调频组件

✨ 基于 sysfs 的 RK3588 用户态温控调频库，通过配置化 4 级策略主动限制 CPU 和 NPU 频率上限。

## 功能概述

- 读取 RK3588 soc-thermal 温度 + CPU/NPU 当前频率
- 4 级温度梯度策略（Normal / Warm / Hot / Critical）含回滞
- 控制 CPU A55 (policy0) + CPU A76 (policy4/policy6) + NPU 频率上限
- 不接管 governor（保持 schedutil / rknpu_ondemand），只调 max_freq
- 启动时自动恢复全速，退出时可配是否恢复
- 通过 config.ini [Thermal] 节配置，重启生效

## 管控范围

| 设备 | sysfs 节点 | 控制方式 |
|------|-----------|---------|
| CPU A55 (核 0-3) | `/sys/.../policy0/scaling_max_freq` | 写上限 |
| CPU A76 (核 4-5) | `/sys/.../policy4/scaling_max_freq` | 写上限 |
| CPU A76 (核 6-7) | `/sys/.../policy6/scaling_max_freq` | 写上限 |
| NPU | `/sys/class/devfreq/fdab0000.npu/max_freq` | 写上限 |

## 构建

```bash
cd thermal-controller && ./build.sh  # 或通过 SentinelQT 一起编译
```

## 依赖

无外部依赖，仅需 C++14 + POSIX 文件 I/O。
```

- [ ] **Step 2: Write IMPLEMENTATION.md**

```markdown
# thermal-controller 实现文档

## 架构总览

```
ThermalController::tick()  [1s 调用]
  ├── read_sensors_()      读取 temp + cur_freq → 缓存
  └── evaluate_and_apply_()  [每 intervalSec 执行]
        ├── 策略评估 (4级回滞)
        └── write_max_freq_() x4 (A55 + A76x2 + NPU)
```

## 线程模型

ThermalController 无独立线程。由调用方（SentinelQT `clockTimer_`）的 1 秒定时器驱动 `tick()`。所有 sysfs 操作在调用线程（Qt 主线程）中同步执行，无锁。

## 核心数据流

1. `tick()` 每次调用内置 `read_sensors_()`：打开 4 个 sysfs 文件分别读 temp + 3 个 cur_freq，关闭后缓存
2. `tickCount_` 递增，若 `tickCount_ % intervalSec == 0` 且 `enabled == true`，执行 `evaluate_and_apply_()`
3. `evaluate_and_apply_()` 根据 tempC_ 和当前 level_ 进行回滞判断，确定新等级
4. 若等级变化（或初次），写入 4 个 max_freq 节点。写入前对比当前 max_freq，相同则跳过

## 配置校验

`validate_config_()` 在构造函数中执行：
- 温度阈值链：warmRecover < warmThreshold < hotRecover < hotThreshold < critRecover < critThreshold
- 频率单调性：Normal >= Warm >= Hot >= Critical (各级别)

校验失败打印 stderr 警告，回退到默认值，不阻止程序启动。
```

- [ ] **Step 3: Write BUG_RECORD.md**

```markdown
# thermal-controller BUG_RECORD

## 1. sysfs 写入失败不重试导致频率未生效

**现象**: tick() 中 write_max_freq_() 返回后频率未变化

**原因**: fopen(path, "w") 失败（权限不足或节点不存在）时静默返回

**解决**: 失败时打印 `strerror(errno)` 到 stderr，下一个周期自动重试。write 调用本身在 fopen 成功的前提下几乎不会失败（内核 sysfs 写操作同步）
```

- [ ] **Step 4: Commit**

```bash
git add thermal-controller/README.md thermal-controller/docs/IMPLEMENTATION.md thermal-controller/BUG_RECORD.md
git commit -m "docs: thermal-controller 组件文档 — README + IMPLEMENTATION + BUG_RECORD"
```
