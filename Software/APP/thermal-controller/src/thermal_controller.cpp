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
