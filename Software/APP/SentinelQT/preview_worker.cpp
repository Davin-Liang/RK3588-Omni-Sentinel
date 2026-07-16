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

    fprintf(stderr, "[PreviewWorker] start() running, cam=%d\n", camNum_);

    while (running_.load()) {
        DmaBuffer_t* previewBuf = visioner_->try_get_preview(camNum_, 200);

        if (!running_.load()) {
            fprintf(stderr, "[PreviewWorker] stopped, exiting loop\n");
            if (previewBuf != nullptr)
                visioner_->release_preview(camNum_, previewBuf);
            break;
        }

        if (previewBuf != nullptr) {
            consecutiveFailures = 0;
            QImage img(static_cast<uchar*>(previewBuf->virtAddr),
                       previewBuf->width,
                       previewBuf->height,
                       QImage::Format_RGB888);
            emit frameReady(img.copy());
            visioner_->release_preview(camNum_, previewBuf);
        } else {
            consecutiveFailures++;
            if (consecutiveFailures >= 5) {
                fprintf(stderr, "[PreviewWorker] no frame for %d cycles\n", consecutiveFailures);
            }
            if (consecutiveFailures > 30) {
                emit error("预览帧超时, 连续" + QString::number(consecutiveFailures) + "次");
                consecutiveFailures = 0;
            }
        }
    }

    fprintf(stderr, "[PreviewWorker] start() exited\n");
}

void PreviewWorker::stop()
{
    running_.store(false);
}
