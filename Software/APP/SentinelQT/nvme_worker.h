#ifndef NVME_WORKER_H
#define NVME_WORKER_H

#include <QObject>
#include <atomic>
#include <cstdint>

#include "sentinel_lslidarer.h"  // LidarPoint 完整定义，数组声明需要

class SentinelStreamer;
class SentinelLslidarer;
class NVMeDataManager;

class NvmeWorker : public QObject
{
    Q_OBJECT

public:
    explicit NvmeWorker(SentinelStreamer* streamer,
                        NVMeDataManager* nvme,
                        SentinelLslidarer** lidarPtr,
                        int numCameras,
                        QObject* parent = nullptr);

public slots:
    void start();
    void stop();

signals:
    void error(const QString& msg);

private:
    SentinelStreamer* streamer_;
    NVMeDataManager* nvme_;
    SentinelLslidarer** lidarPtr_;
    int numCameras_;
    std::atomic<bool> running_{false};

    // 视频帧跳帧计数 (降低 NVMe 写压力)
    int frameSkipCount_[2];

    // 雷达轮询缓冲与去重 (1200 = kPointsPerSweep 理论最大值)
    LidarPoint lidarPointsBuf_[1200];
    uint64_t lastLidarTs_;
};

#endif // NVME_WORKER_H
