#ifndef NVME_WORKER_H
#define NVME_WORKER_H

#include <QObject>
#include <atomic>
#include <cstdint>

class SentinelStreamer;
class NVMeDataManager;

class NvmeWorker : public QObject
{
    Q_OBJECT

public:
    explicit NvmeWorker(SentinelStreamer* streamer,
                        NVMeDataManager* nvme,
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
    int numCameras_;
    std::atomic<bool> running_{false};
};

#endif // NVME_WORKER_H
