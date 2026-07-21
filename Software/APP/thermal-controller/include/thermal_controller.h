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
    int cpuBigCritical = 816000;

    // CPU A55 各等级频率上限 (kHz)
    int cpuLittleNormal   = 1800000;
    int cpuLittleWarm     = 1416000;
    int cpuLittleHot      = 1008000;
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
