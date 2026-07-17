#include "fusion_worker.h"
#include "lidar_camera_fusion.h"
#include "lidar_tracking_types.h"

#include <QThread>
#include <cstdio>
#include <cstring>

FusionWorker::FusionWorker(LidarCameraFusion* fusion, QObject* parent)
    : QObject(parent), fusion_(fusion)
{
}

FusionWorker::~FusionWorker()
{
    stop();
}

void FusionWorker::start()
{
    running_.store(true);

    fprintf(stderr, "[FusionWorker] polling loop started\n");

    while (running_.load()) {
        TrackedTarget snapshot[50];
        uint32_t count = 0;
        fusion_->copy_tracked_targets(snapshot, 50, &count);

        QVector<TrackedTarget> targets;
        targets.reserve(static_cast<int>(count));
        for (uint32_t i = 0; i < count; ++i) {
            targets.append(snapshot[i]);
        }
        emit trackingUpdated(targets);

        QThread::msleep(100);
    }

    fprintf(stderr, "[FusionWorker] polling loop exited\n");
}

void FusionWorker::stop()
{
    running_.store(false);
}
