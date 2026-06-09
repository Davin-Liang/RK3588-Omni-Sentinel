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

signals:
    void frameReady(const QImage& image);
    void error(const QString& msg);

private:
    SentinelVisioner* visioner_;
    int camNum_;
    std::atomic<bool> running_{false};
};

#endif // PREVIEW_WORKER_H
