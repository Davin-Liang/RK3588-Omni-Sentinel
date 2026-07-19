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
        FILE* probe = fopen(kPolicy0MaxFreq, "w");
        if (!probe) {
            fprintf(stderr, "[Thermal] sysfs not writable (%s), disabling\n",
                    kPolicy0MaxFreq);
            cfg_.enabled = false;
        } else {
            fclose(probe);
            startup_restore_();
        }
        if (cfg_.enabled) {
            fprintf(stderr, "[Thermal] started, interval=%ds\n", cfg_.intervalSec);
        }
    }
    if (!cfg_.enabled) {
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

    // 只在等级变化（或首次）时写入频率上限
    if (level_ != prevLevel || tickCount_ == cfg_.intervalSec) {
        if (level_ != prevLevel) {
            fprintf(stderr, "[Thermal] level change: %s -> %s (T=%d°C)\\n",
                    kLevelNames[prevLevel], kLevelNames[level_], t);
        }
        write_max_freq_(kPolicy0MaxFreq, get_level_freq_(cfg_.cpuLittleNormal, cfg_.cpuLittleWarm, cfg_.cpuLittleHot, cfg_.cpuLittleCritical), "CPU little");
        write_max_freq_(kPolicy4MaxFreq, get_level_freq_(cfg_.cpuBigNormal, cfg_.cpuBigWarm, cfg_.cpuBigHot, cfg_.cpuBigCritical), "CPU big(p4)");
        write_max_freq_(kPolicy6MaxFreq, get_level_freq_(cfg_.cpuBigNormal, cfg_.cpuBigWarm, cfg_.cpuBigHot, cfg_.cpuBigCritical), "CPU big(p6)");
        write_max_freq_(kNpuMaxFreq,     get_level_freq_(cfg_.npuNormal, cfg_.npuWarm, cfg_.npuHot, cfg_.npuCritical), "NPU");
    }
}

void ThermalController::tick()
{
    read_sensors_();
    tickCount_++;
    if (cfg_.enabled && (tickCount_ % cfg_.intervalSec == 0)) {
        evaluate_and_apply_();
    }
}

const char* ThermalController::currentLevel() const
{
    if (!cfg_.enabled) return "";  // disabled 时不显示等级
    return kLevelNames[level_];
}

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
