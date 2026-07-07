#ifndef AI_REPORT_WORKER_H
#define AI_REPORT_WORKER_H

#include <QObject>
#include <QMutex>
#include <QString>
#include <atomic>

#include "deepseek_inference.h"

class AIReportWorker : public QObject
{
    Q_OBJECT

public:
    explicit AIReportWorker(QObject* parent = nullptr);
    ~AIReportWorker();

    void setConfig(const DeepSeekInference::Config& cfg);

    void updateStatus(int cpuTemp, int cpuUsage,
                      const QString& cam0Status, const QString& cam1Status,
                      const QString& lidarStatus, const QString& imuStatus,
                      const QString& fusionStatus,
                      double cam0Fps, double cam1Fps);

public slots:
    void start();
    void stop();
    void requestReport();

signals:
    void reportReady(const QString& report);
    void error(const QString& msg);

private:
    DeepSeekInference       inference_;
    DeepSeekInference::Config config_;
    std::atomic<bool>       running_{false};
    std::atomic<bool>       pending_{false};

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

    QString buildPrompt_();
};

#endif
