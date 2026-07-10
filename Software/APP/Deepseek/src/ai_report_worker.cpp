#include "ai_report_worker.h"
#include <QThread>
#include <cstdio>

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
// start() — Worker 线程的主入口
//
// 调用栈：
//   主线程:  aiReportThread_->start()
//             → QThread 内部进入事件循环
//             → 发射 QThread::started 信号
//             → 槽函数 AIReportWorker::start() 在此被执行（子线程上下文）
//
// 工作流程：
//   1. 加载 DeepSeek 模型到 NPU（inference_.initialize）
//   2. 进入 while(running_) 轮询，每 200ms 检查一次 pending_ 标志
//   3. 检测到 pending_ == true → buildPrompt_() 构建中文问题
//      → inference_.inferSync() 阻塞等待 NPU 推理（约 2 分钟）
//      → emit reportReady / error 通知主线程
//   4. 循环退出时 inference_.destroy() 释放 NPU 资源
// ============================================================================
void AIReportWorker::start()
{
    // ---- 第 1 步：加载模型 ----
    if (!inference_.initialize(config_)) {
        fprintf(stderr, "[AIReportWorker] DeepSeekInference init failed\n");
        emit error(QString::fromUtf8("AI 模型初始化失败"));
        return;
    }

    running_.store(true);
    fprintf(stderr, "[AIReportWorker] started, waiting for report requests\n");

    // ---- 第 2 步：轮询等待推理请求 ----
    while (running_.load()) {
        if (pending_.load()) {                               // 主线程通过 requestReport() 设置了 pending_
            fprintf(stderr, "[AIReportWorker] processing report request...\n");

            QString prompt = buildPrompt_();                 // 读系统状态快照，生成中文问题
            std::string promptStr = prompt.toStdString();

            fprintf(stderr, "[AIReportWorker] prompt length: %zu chars\n", promptStr.size());

            std::string result = inference_.inferSync(promptStr, 60000);  // 阻塞，NPU 推理

            if (result.empty()) {
                fprintf(stderr, "[AIReportWorker] inference returned empty result\n");
                emit error(QString::fromUtf8("AI 推理返回空结果"));      // 跨线程信号 → 主线程
            } else {
                fprintf(stderr, "[AIReportWorker] inference done, result length: %zu\n", result.size());
                emit reportReady(QString::fromStdString(result));         // 跨线程信号 → 主线程
            }

            pending_.store(false);                           // 重置标志，允许下一次触发
        }

        QThread::msleep(200);                                // 200ms 轮询间隔，避免空转 CPU
    }

    // ---- 第 3 步：清理 ----
    fprintf(stderr, "[AIReportWorker] stopping, destroying inference...\n");
    inference_.destroy();
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

    QString prompt;
    prompt += QString::fromUtf8("请对以下RK3588边缘计算平台的运行状态进行综合分析：\n\n");
    prompt += QString::fromUtf8("硬件状态：\n");
    prompt += QString::fromUtf8("- CPU温度: %1°C\n").arg(cpuTemp_);
    prompt += QString::fromUtf8("- CPU占用率: %1%\n").arg(cpuUsage_);
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

    prompt += QString::fromUtf8("请分析：\n");
    prompt += QString::fromUtf8("1. 系统整体健康度评估（优/良/中/差）\n");
    prompt += QString::fromUtf8("2. 是否存在异常或风险点\n");
    prompt += QString::fromUtf8("3. 优化建议\n");
    prompt += QString::fromUtf8("请用简洁的中文回答，控制在200字以内。");

    return prompt;
}
