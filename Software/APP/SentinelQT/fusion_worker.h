#ifndef FUSION_WORKER_H
#define FUSION_WORKER_H

#include <QObject>
#include <QVector>
#include <atomic>
#include <cstdint>

struct TrackedTarget;
class LidarCameraFusion;

class FusionWorker : public QObject {
    Q_OBJECT

public:
    FusionWorker(LidarCameraFusion* fusion, QObject* parent = nullptr);
    ~FusionWorker();

public slots:
    void start();
    void stop();

signals:
    void trackingUpdated(const QVector<TrackedTarget>& targets);
    void error(const QString& msg);

private:
    LidarCameraFusion* fusion_;
    std::atomic<bool>  running_{false};
};

#endif // FUSION_WORKER_H
