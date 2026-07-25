#ifndef PREVIEW_WORKER_H
#define PREVIEW_WORKER_H

#include <QObject>
#include <QImage>
#include <atomic>

class SentinelVisioner;

class PreviewWorker : public QObject {
    Q_OBJECT

public:
    PreviewWorker(SentinelVisioner* visioner, int camNum, QObject* parent = nullptr);
    ~PreviewWorker();

public slots:
    void start();
    void stop();

public:
    // EIS 开启后，sentinel-visioner 的 EIS 预览路径输出字节序与普通预览不同。
    // 该标志只影响 QImage 对 DMA 字节的解释，不会修改 DMA 原始数据。
    void setEisOutputActive(bool active) noexcept;

    // 主线程处理完一帧后释放“在途帧”标志。
    // 这样跨线程事件队列中最多只保留一张 1080p QImage，避免回溯阻塞 UI 时内存持续增长。
    void markFrameConsumed() noexcept;

signals:
    void frameReady(const QImage& image);
    void error(const QString& msg);

private:
    SentinelVisioner* visioner_;
    int camNum_;
    std::atomic<bool> running_{false};
    std::atomic<bool> framePending_{false};
    std::atomic<bool> eisOutputActive_{false};
};

#endif // PREVIEW_WORKER_H
