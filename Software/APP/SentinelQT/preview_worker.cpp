#include "preview_worker.h"
#include "sentinel-visioner.h"

PreviewWorker::PreviewWorker(SentinelVisioner* visioner, int camNum, QObject* parent)
    : QObject(parent)
    , visioner_(visioner)
    , camNum_(camNum)
{
}

PreviewWorker::~PreviewWorker()
{
    stop();
}

void PreviewWorker::start()
{
    running_.store(true);
    int consecutiveFailures = 0;

    while (running_.load()) {
        NpuPreview task = visioner_->try_get_preview(camNum_, 200);

        if (!running_.load()) {
            if (task.npuImage || task.previewImage)
                visioner_->release_preview(camNum_, &task);
            break;
        }

        if (task.previewImage != nullptr) {
            consecutiveFailures = 0;
            QImage img(static_cast<uchar*>(task.previewImage->virtAddr),
                       task.previewImage->width,
                       task.previewImage->height,
                       QImage::Format_RGB888);
            emit frameReady(img.copy());
        } else if (task.npuImage == nullptr) {
            consecutiveFailures++;
            if (consecutiveFailures > 30) {
                emit error("预览帧超时, 连续" + QString::number(consecutiveFailures) + "次");
                consecutiveFailures = 0;
            }
        }

        visioner_->release_preview(camNum_, &task);
    }
}

void PreviewWorker::stop()
{
    running_.store(false);
}
