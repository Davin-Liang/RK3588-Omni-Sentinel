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

    while (running_.load()) {
        NpuPreview task = visioner_->wait_get_preview(camNum_);

        if (!running_.load()) {
            if (task.npuImage || task.previewImage)
                visioner_->release_preview(camNum_, &task);
            break;
        }

        if (task.previewImage != nullptr) {
            QImage img(static_cast<uchar*>(task.previewImage->virtAddr),
                       task.previewImage->width,
                       task.previewImage->height,
                       QImage::Format_RGB888);
            emit frameReady(img.copy());  // copy before releasing DMA buffer
        }

        visioner_->release_preview(camNum_, &task);
    }
}

void PreviewWorker::stop()
{
    running_.store(false);
}
