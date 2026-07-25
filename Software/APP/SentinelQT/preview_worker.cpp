#include "preview_worker.h"
#include "sentinel-visioner.h"

#include <QtGlobal>

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

void PreviewWorker::setEisOutputActive(bool active) noexcept
{
    eisOutputActive_.store(active, std::memory_order_release);
}

void PreviewWorker::markFrameConsumed() noexcept
{
    framePending_.store(false, std::memory_order_release);
}

void PreviewWorker::start()
{
    running_.store(true, std::memory_order_release);
    framePending_.store(false, std::memory_order_release);
    int consecutiveFailures = 0;
    uint64_t droppedFrames = 0;

    fprintf(stderr, "[PreviewWorker] start() running, cam=%d\n", camNum_);

    while (running_.load(std::memory_order_acquire)) {
        DmaBuffer_t* previewBuf = visioner_->try_get_preview(camNum_, 200);

        if (!running_.load(std::memory_order_acquire)) {
            fprintf(stderr, "[PreviewWorker] stopped, exiting loop\n");
            if (previewBuf != nullptr) {
                visioner_->release_preview(camNum_, previewBuf);
            }
            break;
        }

        if (previewBuf != nullptr) {
            consecutiveFailures = 0;

            /*
             * 颜色格式说明：
             * - 普通预览路径由 RGA 的 RK_FORMAT_BGR_888 输出，在当前 RK3588
             *   字节序下使用 QImage::Format_RGB888 显示正确；
             * - EIS 预览路径使用 RK_FORMAT_RGB_888，DMA 中实际字节需要按
             *   BGR888 解释，否则 Qt 上会出现红蓝通道交换。
             *
             * 这里只改变 QImage 对字节的解释，不在原 DMA 帧上做任何修改。
             */
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
            const QImage::Format imageFormat =
                eisOutputActive_.load(std::memory_order_acquire)
                    ? QImage::Format_BGR888
                    : QImage::Format_RGB888;
#else
            const QImage::Format imageFormat = QImage::Format_RGB888;
#endif

            /*
             * 跨线程 queued signal 会复制信号参数。若 Qt 主线程正在同步执行
             * NVMe 回溯导出，事件循环无法消费帧，原实现会按 30 FPS 不断
             * img.copy()，造成每秒约 180 MB/路的事件队列积压。
             *
             * framePending_ 把在途帧限制为 1 张：上一帧尚未处理时直接丢弃
             * 当前预览帧，但始终及时归还 DMA buffer。
             */
            bool expected = false;
            if (framePending_.compare_exchange_strong(
                    expected, true,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                QImage view(static_cast<uchar*>(previewBuf->virtAddr),
                            previewBuf->width,
                            previewBuf->height,
                            imageFormat);

                QImage owned = view.copy();
#if QT_VERSION < QT_VERSION_CHECK(5, 14, 0)
                if (eisOutputActive_.load(std::memory_order_acquire)) {
                    owned = owned.rgbSwapped();
                }
#endif
                if (!owned.isNull()) {
                    emit frameReady(owned);
                } else {
                    framePending_.store(false, std::memory_order_release);
                }
            } else {
                ++droppedFrames;
                if (droppedFrames % 300 == 1) {
                    fprintf(stderr,
                            "[PreviewWorker] cam=%d dropped queued preview frames=%llu\n",
                            camNum_,
                            static_cast<unsigned long long>(droppedFrames));
                }
            }

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

    framePending_.store(false, std::memory_order_release);
    fprintf(stderr, "[PreviewWorker] start() exited\n");
}

void PreviewWorker::stop()
{
    running_.store(false, std::memory_order_release);
}
