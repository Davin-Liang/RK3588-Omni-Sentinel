#ifndef AI_REPORT_WORKER_H
#define AI_REPORT_WORKER_H

#include <QObject>
#include <QMutex>
#include <QString>
#include <atomic>

#include "deepseek_inference.h"

/**
 * @brief AI 系统状态分析 Worker（运行在独立 QThread）
 *
 * 参照 FusionWorker 模式：QObject + moveToThread + std::atomic 控制生命周期。
 *
 * 主线程每秒调用 updateStatus() 推送系统状态快照；
 * 用户点击按钮后 requestReport() 触发一次推理，完成后 emit reportReady。
 */
class AIReportWorker : public QObject
{
    Q_OBJECT

public:
    explicit AIReportWorker(QObject* parent = nullptr);
    ~AIReportWorker();

    /** @brief 配置推理参数（需在 start() 之前调用） */
    void setConfig(const DeepSeekInference::Config& cfg);

    /** @brief 主线程每秒调用，更新系统状态快照（线程安全） */
    void updateStatus(int cpuTemp, int cpuUsage,
                      const QString& cam0Status, const QString& cam1Status,
                      const QString& lidarStatus, const QString& imuStatus,
                      const QString& fusionStatus,
                      double cam0Fps, double cam1Fps);

public slots:
    /** @brief 在子线程中初始化 DeepSeekInference */
    void start();

    /** @brief 停止工作循环 */
    void stop();

    /** @brief 触发一次 AI 分析（非阻塞，完成后 emit reportReady） */
    void requestReport();

signals:
    /** @brief AI 分析报告就绪 */
    void reportReady(const QString& report);

    /** @brief 推理出错 */
    void error(const QString& msg);

private:
    DeepSeekInference       inference_;
    DeepSeekInference::Config config_;
    std::atomic<bool>       running_{false};
    std::atomic<bool>       pending_{false};

    // 系统状态快照（mutex 保护，主线程写 / worker 线程读）
    QMutex  statusMutex_;
    int     cpuTemp_   = 0;
    int     cpuUsage_  = 0;
    QString cam0Status_;
    QString cam1Status_;
    QString lidarStatus_;
    QString imuStatus_;
    QString fusionStatus_;
    double  cam0Fps_   = 0.0;
    double  cam1Fps_   = 0.0;

    /** @brief 根据缓存的系统状态构建中文分析 prompt */
    QString buildPrompt_();
};

#endif // AI_REPORT_WORKER_H
