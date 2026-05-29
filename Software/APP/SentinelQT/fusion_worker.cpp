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

    QVector<TrackedTarget> lastSnapshot;

    while (running_.load()) {
        TrackedTarget snapshot[50];
        uint32_t count = 0;
        fusion_->copy_tracked_targets(snapshot, 50, &count);

        bool changed = (static_cast<int>(count) != lastSnapshot.size());
        if (!changed) {
            for (uint32_t i = 0; i < count; ++i) {
                if (snapshot[i].id    != lastSnapshot[static_cast<int>(i)].id ||
                    snapshot[i].posX  != lastSnapshot[static_cast<int>(i)].posX ||
                    snapshot[i].posY  != lastSnapshot[static_cast<int>(i)].posY ||
                    snapshot[i].state != lastSnapshot[static_cast<int>(i)].state) {
                    changed = true;
                    break;
                }
            }
        }

        if (changed && count > 0) {
            QVector<TrackedTarget> targets;
            targets.reserve(static_cast<int>(count));
            for (uint32_t i = 0; i < count; ++i) {
                targets.append(snapshot[i]);
            }
            emit trackingUpdated(targets);
            lastSnapshot = targets;
        } else if (count == 0 && !lastSnapshot.isEmpty()) {
            emit trackingUpdated(QVector<TrackedTarget>());
            lastSnapshot.clear();
        }

        QThread::msleep(100);
    }

    fprintf(stderr, "[FusionWorker] polling loop exited\n");
}

void FusionWorker::stop()
{
    running_.store(false);
}
