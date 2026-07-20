#ifndef NVME_WORKER_H
#define NVME_WORKER_H

#include <QObject>
#include <atomic>
#include <cstdint>

class SentinelStreamer;
class SentinelLslidarer;
struct LidarPoint;
class NVMeDataManager;

class NvmeWorker : public QObject
{
    Q_OBJECT

public:
    explicit NvmeWorker(SentinelStreamer* streamer,
                        NVMeDataManager* nvme,
                        SentinelLslidarer* lidar,
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
    SentinelLslidarer* lidar_;
    int numCameras_;
    std::atomic<bool> running_{false};

    // 雷达轮询缓冲与去重 (1200 = kPointsPerSweep 理论最大值)
    LidarPoint lidarPointsBuf_[1200];
    uint64_t lastLidarTs_;
};

#endif // NVME_WORKER_H
