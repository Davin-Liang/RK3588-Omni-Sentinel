#include "ai_report_worker.h"
#include <QThread>
#include <cstdio>
#include <cstring>
#include <unistd.h>

// ============================================================================
// 构造函数
// 仅初始化 Qt 父对象，其余成员用默认值（指针空、原子量 false、整数 0）
// ============================================================================
AIReportWorker::AIReportWorker(QObject* parent)
    : QObject(parent) {}

// ============================================================================
// 析构函数
// 调用 stop() 将 running_ 置为 false，让 start() 中的 while 循环退出
// ============================================================================
AIReportWorker::~AIReportWorker() { stop(); }

// ============================================================================
// 设置推理参数
// 主线程在创建 Worker 后、启动线程前调用。
// cfg 包含模型路径、采样温度、最大 token 数等，透传给 DeepSeekInference
// ============================================================================
void AIReportWorker::setConfig(const DeepSeekInference::Config& cfg) { config_ = cfg; }

// ============================================================================
// start() — Worker 线程的主入口（热加载模式）
//
// 启动时即加载模型并热身，在 DDR 空闲时抢占连续大块 workspace，
// 避免全功能运行时动态分配导致 DDR 拥堵 + VmRSS 虚假高水位 → OOM。
// 模型权重 ~1GB + workspace ~500MB 常驻 DDR，后续推理不再分配。
// ============================================================================
void AIReportWorker::start()
{
    // ---- 第 1 步：加载模型 ----
    fprintf(stderr, "[AIReportWorker] eager-loading model (~1GB)...\n");
    if (!inference_.initialize(config_)) {
        fprintf(stderr, "[AIReportWorker] DeepSeekInference init failed\n");
        emit error(QString::fromUtf8("AI 模型初始化失败"));
        return;
    }
    initialized_.store(true);
    fprintf(stderr, "[AIReportWorker] model loaded successfully\n");

    // ---- 第 2 步：热身推理，在 DDR 空闲时预分配 NPU workspace ----
    fprintf(stderr, "[AIReportWorker] warmup inference to pre-allocate workspace...\n");
    {
        std::string warmupResult = inference_.inferSync("Hello", 5000);
        fprintf(stderr, "[AIReportWorker] warmup done, result=%d chars, "
                "waiting 3s for DDR reclaim...\n",
                static_cast<int>(warmupResult.size()));
    }
    // 等待内核 buddy allocator 回收 warmup 触发的临时缓冲
    QThread::sleep(3);
    fprintf(stderr, "[AIReportWorker] DDR reclaim wait done\n");

    running_.store(true);
    fprintf(stderr, "[AIReportWorker] started, waiting for report requests\n");

    // ---- 第 3 步：轮询等待推理请求 ----
    while (running_.load()) {
        if (pending_.load()) {
            fprintf(stderr, "[AIReportWorker] processing report request...\n");

            QString prompt = buildPrompt_();
            std::string promptStr = prompt.toStdString();
            fprintf(stderr, "[AIReportWorker] prompt length: %zu chars\n", promptStr.size());

            std::string result = inference_.inferSync(promptStr, 60000);

            if (result.empty()) {
                fprintf(stderr, "[AIReportWorker] inference returned empty result\n");
                emit error(QString::fromUtf8("AI 推理返回空结果"));
            } else {
                fprintf(stderr, "[AIReportWorker] inference done, result length: %zu\n", result.size());
                emit reportReady(QString::fromStdString(result));
            }

            pending_.store(false);
        }

        QThread::msleep(200);
    }

    // ---- 清理 ----
    fprintf(stderr, "[AIReportWorker] stopping, destroying inference...\n");
    inference_.destroy();
    initialized_.store(false);
    fprintf(stderr, "[AIReportWorker] stopped\n");
}

// ============================================================================
// stop() — 通知 Worker 线程退出
// 析构时会先调 stop() 再 wait() join 线程，确保推理循环安全退出
// ============================================================================
void AIReportWorker::stop() { running_.store(false); }

// ============================================================================
// requestReport() — 触发一次 AI 分析（异步）
//
// 调用来源：
//   - QT 屏幕按钮: on_btn_ai_analysis_() → requestReport()
//   - Web 远程 API: web_ai_report_() → requestReport()
//   - 自动定时器:   on_ai_auto_tick_() 倒计时归零 → requestReport()
//
// 并发保护：
//   使用 compare_exchange_strong 原子操作，只有当前值为 false 时才设置为 true。
//   如果上一次推理还在进行（pending_ 仍为 true），本次请求被静默忽略。
// ============================================================================
void AIReportWorker::requestReport()
{
    bool expected = false;
    if (!pending_.compare_exchange_strong(expected, true)) {
        fprintf(stderr, "[AIReportWorker] request already pending, ignoring duplicate\n");
        return;
    }
    fprintf(stderr, "[AIReportWorker] report requested\n");
}

// ============================================================================
// updateStatus() — 更新系统状态快照（每秒从主线程调用）
//
// 线程安全：
//   QMutexLocker RAII 锁，函数返回时自动解锁。
//   主线程写快照，Worker 线程通过 buildPrompt_() 读快照，互斥保护。
//
// 参数说明：
//   cpuTemp    — CPU 温度（°C），从 /sys/class/thermal/thermal_zone0/temp 读取
//   cpuUsage   — CPU 占用率（%），从 /proc/stat 计算
//   cam0Status — 相机 0 状态字符串，如 "预览中, 推流中, FPS 15.2"
//   cam1Status — 相机 1 状态字符串
//   lidarStatus— 激光雷达状态，如 "运行中, 10Hz"
//   imuStatus  — IMU 状态（当前为占位 "未启用"）
//   fusionStatus — 融合跟踪状态，如 "目标数: 3, 已确认: 2, 告警: 0, 融合引擎: 运行中"
//   cam0Fps    — 相机 0 实时帧率
//   cam1Fps    — 相机 1 实时帧率
// ============================================================================
void AIReportWorker::updateStatus(int cpuTemp, int cpuUsage,
                                  const QString& cam0Status, const QString& cam1Status,
                                  const QString& lidarStatus, const QString& imuStatus,
                                  const QString& fusionStatus,
                                  double cam0Fps, double cam1Fps)
{
    QMutexLocker lock(&statusMutex_);    // 加锁，函数结束时自动解锁
    cpuTemp_      = cpuTemp;
    cpuUsage_     = cpuUsage;
    cam0Status_   = cam0Status;
    cam1Status_   = cam1Status;
    lidarStatus_  = lidarStatus;
    imuStatus_    = imuStatus;
    fusionStatus_ = fusionStatus;
    cam0Fps_      = cam0Fps;
    cam1Fps_      = cam1Fps;
}

// ============================================================================
// buildPrompt_() — 把系统状态快照组装成 DeepSeek 可理解的中文问题
//
// 返回值示例（约 400 字符）：
//   "请对以下RK3588边缘计算平台的运行状态进行综合分析：
//
//    硬件状态：
//    - CPU温度: 65°C
//    - CPU占用率: 42%
//
//    传感器状态：
//    - 相机0 (ISP): 预览中, 推流中, FPS 15.2
//    - 相机1 (USB): 预览关闭, FPS 0.0
//    - 激光雷达: 运行中, 10Hz
//    - IMU: 未启用
//
//    融合跟踪状态：
//    - 目标数: 3, 已确认: 2, 告警: 0, 融合引擎: 运行中
//
//    请分析：
//    1. 系统整体健康度评估（优/良/中/差）
//    2. 是否存在异常或风险点
//    3. 优化建议
//    请用简洁的中文回答，控制在200字以内。"
//
// 注意：返回的只是问题文本，不包含模型模板前缀/后缀。
// 模板（<｜begin▁of▁sentence｜><｜User｜>...<｜Assistant｜>）由 DeepSeekInference::inferSync() 自动添加。
// ============================================================================
QString AIReportWorker::buildPrompt_()
{
    QMutexLocker lock(&statusMutex_);    // 与 updateStatus() 互斥，保证读到一致的快照

    // ---- 读取系统内存信息（/proc/meminfo） ----
    int memTotalMB = 0, memAvailMB = 0;
    {
        FILE* fp = fopen("/proc/meminfo", "r");
        if (fp) {
            char line[256];
            int memTotalKB = 0, memAvailKB = 0;
            while (fgets(line, sizeof(line), fp)) {
                if (strncmp(line, "MemTotal:", 9) == 0)
                    sscanf(line + 9, "%d", &memTotalKB);
                else if (strncmp(line, "MemAvailable:", 13) == 0)
                    sscanf(line + 13, "%d", &memAvailKB);
                if (memTotalKB > 0 && memAvailKB > 0) break;
            }
            fclose(fp);
            memTotalMB = memTotalKB / 1024;
            memAvailMB = memAvailKB / 1024;
        }
    }

    // ---- 读取每核 CPU 占用率（两次采样 /proc/stat，间隔 100ms） ----
    // RK3588: cpu0-3 = Cortex-A76 (大核), cpu4-7 = Cortex-A55 (小核)
    auto readCpuStats_ = [](uint64_t idle[8], uint64_t total[8]) -> bool {
        FILE* fp = fopen("/proc/stat", "r");
        if (!fp) return false;
        char line[256];
        int coreIdx = -1;  // -1 = aggregate line, 0-7 = per-core
        while (fgets(line, sizeof(line), fp) && coreIdx < 7) {
            if (strncmp(line, "cpu ", 4) == 0) {
                coreIdx = -1;  // skip aggregate
                continue;
            }
            if (strncmp(line, "cpu", 3) != 0) break;  // done with cpu lines
            int ci = atoi(line + 3);
            if (ci < 0 || ci > 7) continue;
            uint64_t user, nice, sys, iowait, irq, softirq, steal, guest, guest_nice;
            int n = sscanf(line + 5, "%llu %llu %llu %llu %llu %llu %llu %llu %llu %llu",
                          &user, &nice, &sys, &idle[ci], &iowait, &irq, &softirq,
                          &steal, &guest, &guest_nice);
            if (n >= 4) {
                // idle already stored, compute total
                total[ci] = user + nice + sys + idle[ci] + iowait + irq + softirq + steal;
                if (coreIdx < ci) coreIdx = ci;
            }
        }
        fclose(fp);
        return coreIdx >= 7;  // got all 8 cores
    };

    int bigPct[4]   = {-1, -1, -1, -1};  // cpu0-3 (A76)
    int littlePct[4] = {-1, -1, -1, -1};  // cpu4-7 (A55)
    {
        uint64_t idle1[8] = {}, total1[8] = {};
        uint64_t idle2[8] = {}, total2[8] = {};
        if (readCpuStats_(idle1, total1)) {
            QThread::msleep(100);
            if (readCpuStats_(idle2, total2)) {
                for (int i = 0; i < 8; ++i) {
                    uint64_t dIdle = idle2[i] - idle1[i];
                    uint64_t dTotal = total2[i] - total1[i];
                    if (dTotal > 0) {
                        int pct = (int)(100 - (dIdle * 100 / dTotal));
                        if (i < 4) bigPct[i]    = pct;
                        else       littlePct[i - 4] = pct;
                    }
                }
            }
        }
    }

    // 计算大小核平均占用
    auto avgPct_ = [](const int* arr, int n) -> int {
        int sum = 0, cnt = 0;
        for (int i = 0; i < n; ++i) {
            if (arr[i] >= 0) { sum += arr[i]; ++cnt; }
        }
        return cnt > 0 ? sum / cnt : -1;
    };
    int bigAvg    = avgPct_(bigPct, 4);
    int littleAvg = avgPct_(littlePct, 4);

    QString prompt;
    prompt += QString::fromUtf8("请对以下RK3588边缘计算平台的运行状态进行综合分析：\n\n");
    prompt += QString::fromUtf8("硬件状态：\n");
    prompt += QString::fromUtf8("- CPU温度: %1°C\n").arg(cpuTemp_);
    if (bigAvg >= 0 && littleAvg >= 0) {
        // 有每核数据：显示大小核分别占用 + 汇总
        prompt += QString::fromUtf8("- CPU占用: 8核平均 %1%%, 大核A76 %2%%, 小核A55 %3%%\n")
                      .arg(cpuUsage_).arg(bigAvg).arg(littleAvg);
    } else {
        // 回退：仅显示汇总值
        prompt += QString::fromUtf8("- CPU占用率: %1%\n").arg(cpuUsage_);
    }
    if (memTotalMB > 0) {
        prompt += QString::fromUtf8("- 系统内存: 总计 %1 MB, 可用 %2 MB (已用约 %3 MB)\n")
                      .arg(memTotalMB).arg(memAvailMB).arg(memTotalMB - memAvailMB);
    }
    prompt += "\n";

    prompt += QString::fromUtf8("传感器状态：\n");
    prompt += QString::fromUtf8("- 相机0 (ISP): %1, FPS %2\n")
                  .arg(cam0Status_).arg(cam0Fps_, 0, 'f', 1);
    prompt += QString::fromUtf8("- 相机1 (USB): %1, FPS %2\n")
                  .arg(cam1Status_).arg(cam1Fps_, 0, 'f', 1);
    prompt += QString::fromUtf8("- 激光雷达: %1\n").arg(lidarStatus_);
    prompt += QString::fromUtf8("- IMU: %1\n").arg(imuStatus_);
    prompt += "\n";

    prompt += QString::fromUtf8("融合跟踪状态：\n");
    prompt += QString::fromUtf8("- %1\n").arg(fusionStatus_);
    prompt += "\n";

    // 动态构建分析指令：有跟踪目标时加入安全评估
    bool hasTargets = fusionStatus_.contains(QString::fromUtf8("目标数:")) &&
                      !fusionStatus_.contains(QString::fromUtf8("目标数: 0,"));
    prompt += QString::fromUtf8("请分析：\n");
    prompt += QString::fromUtf8("1. 系统整体健康度评估（优/良/中/差）\n");
    if (hasTargets) {
        prompt += QString::fromUtf8("2. 【安全评估】检查各目标距离是否小于告警阈值，"
                                    "如有目标进入危险区域必须明确警告其编号和距离\n");
        prompt += QString::fromUtf8("3. 是否存在其他异常或风险点\n");
        prompt += QString::fromUtf8("4. 优化建议\n");
        prompt += QString::fromUtf8("请用简洁的中文回答，控制在250字以内。");
    } else {
        prompt += QString::fromUtf8("2. 是否存在异常或风险点\n");
        prompt += QString::fromUtf8("3. 优化建议\n");
        prompt += QString::fromUtf8("请用简洁的中文回答，控制在200字以内。");
    }

    return prompt;
}
