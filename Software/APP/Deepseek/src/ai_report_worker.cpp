#include "ai_report_worker.h"
#include <QThread>
#include <cstdio>

AIReportWorker::AIReportWorker(QObject* parent)
    : QObject(parent) {}

AIReportWorker::~AIReportWorker() { stop(); }

void AIReportWorker::setConfig(const DeepSeekInference::Config& cfg) { config_ = cfg; }

void AIReportWorker::start()
{
    if (!inference_.initialize(config_)) {
        fprintf(stderr, "[AIReportWorker] DeepSeekInference init failed\n");
        emit error(QString::fromUtf8("AI 模型初始化失败"));
        return;
    }
    running_.store(true);
    fprintf(stderr, "[AIReportWorker] started, waiting for report requests\n");
    while (running_.load()) {
        if (pending_.load()) {
            fprintf(stderr, "[AIReportWorker] processing report request...\n");
            QString prompt = buildPrompt_();
            std::string promptStr = prompt.toStdString();
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
    fprintf(stderr, "[AIReportWorker] stopping, destroying inference...\n");
    inference_.destroy();
    fprintf(stderr, "[AIReportWorker] stopped\n");
}

void AIReportWorker::stop() { running_.store(false); }

void AIReportWorker::requestReport()
{
    bool expected = false;
    if (!pending_.compare_exchange_strong(expected, true)) {
        fprintf(stderr, "[AIReportWorker] request already pending, ignoring duplicate\n");
        return;
    }
    fprintf(stderr, "[AIReportWorker] report requested\n");
}

void AIReportWorker::updateStatus(int cpuTemp, int cpuUsage,
                                  const QString& cam0Status, const QString& cam1Status,
                                  const QString& lidarStatus, const QString& imuStatus,
                                  const QString& fusionStatus,
                                  double cam0Fps, double cam1Fps)
{
    QMutexLocker lock(&statusMutex_);
    cpuTemp_ = cpuTemp; cpuUsage_ = cpuUsage;
    cam0Status_ = cam0Status; cam1Status_ = cam1Status;
    lidarStatus_ = lidarStatus; imuStatus_ = imuStatus;
    fusionStatus_ = fusionStatus;
    cam0Fps_ = cam0Fps; cam1Fps_ = cam1Fps;
}

QString AIReportWorker::buildPrompt_()
{
    QMutexLocker lock(&statusMutex_);
    QString prompt;
    prompt += QString::fromUtf8("请对以下RK3588边缘计算平台的运行状态进行综合分析：\n\n");
    prompt += QString::fromUtf8("硬件状态：\n");
    prompt += QString::fromUtf8("- CPU温度: %1°C\n").arg(cpuTemp_);
    prompt += QString::fromUtf8("- CPU占用率: %1%\n").arg(cpuUsage_);
    prompt += "\n";
    prompt += QString::fromUtf8("传感器状态：\n");
    prompt += QString::fromUtf8("- 相机0 (ISP): %1, FPS %2\n").arg(cam0Status_).arg(cam0Fps_, 0, 'f', 1);
    prompt += QString::fromUtf8("- 相机1 (USB): %1, FPS %2\n").arg(cam1Status_).arg(cam1Fps_, 0, 'f', 1);
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
