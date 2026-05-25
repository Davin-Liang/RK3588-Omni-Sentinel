#include "widget.h"
#include "./ui_widget.h"

#include "sentinel-visioner.h"
#include "sentinel_streamer.h"
#include "preview_worker.h"

#include <QThread>
#include <QDateTime>
#include <cstring>
#include <cstdio>

Widget* Widget::instance_ = nullptr;

// StreamerCallback runs in streamer's internal thread, cross to main thread via invokeMethod
static void streamer_callback_(int camNum, StreamerEvent event, const char* detail)
{
    Widget* w = Widget::instance();
    if (!w) return;
    QString detailStr = detail ? QString::fromUtf8(detail) : QString();
    QMetaObject::invokeMethod(w, [w, camNum, event, detailStr]() {
        w->on_streamer_event(camNum, event, detailStr.toUtf8().constData());
    }, Qt::QueuedConnection);
}

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , visioner_(new SentinelVisioner())
    , streamer_(new SentinelStreamer())
    , previewWorker_(nullptr)
    , previewThread_(nullptr)
    , config_("config.ini", QSettings::IniFormat)
    , frameCount_(0)
    , lastFpsTsUs_(0)
{
    instance_ = this;
    ui->setupUi(this);

    // Read config
    rtspUrl_ = config_.value("Stream/rtspUrl", "rtsp://192.168.1.100:8554/live/cam0").toString();
    recordPath_ = config_.value("Record/filePath", "/mnt/sdcard/record.mp4").toString();

    // Button signal-slot connections
    connect(ui->btnStartStream, &QPushButton::clicked, this, &Widget::on_btn_start_stream_);
    connect(ui->btnStopStream,  &QPushButton::clicked, this, &Widget::on_btn_stop_stream_);
    connect(ui->btnStartRecord, &QPushButton::clicked, this, &Widget::on_btn_start_record_);
    connect(ui->btnStopRecord,  &QPushButton::clicked, this, &Widget::on_btn_stop_record_);

    // Initialize camera
    if (!init_camera_()) {
        set_status_("相机初始化失败!", "#e74c3c");
        return;
    }

    // Start preview thread
    previewWorker_ = new PreviewWorker(visioner_, 0);
    previewThread_ = new QThread(this);
    previewWorker_->moveToThread(previewThread_);
    connect(previewWorker_, &PreviewWorker::frameReady, this, &Widget::on_frame_ready_);
    connect(previewThread_, &QThread::started, previewWorker_, &PreviewWorker::start);
    previewThread_->start();

    set_status_("系统就绪", "#4ecca3");
}

Widget::~Widget()
{
    // Release in reverse order
    if (previewWorker_) {
        previewWorker_->stop();
    }
    if (previewThread_ && previewThread_->isRunning()) {
        previewThread_->quit();
        previewThread_->wait(3000);
    }

    delete streamer_;
    delete visioner_;
    delete ui;
}

bool Widget::init_camera_()
{
    std::string devName = "/dev/video11";
    if (!visioner_->add_camera(devName, 1920, 1080, 8, 0)) {
        fprintf(stderr, "[SentinelQT] visioner add_camera failed\n");
        return false;
    }

    if (!streamer_->add_camera(0, visioner_)) {
        fprintf(stderr, "[SentinelQT] streamer add_camera failed\n");
        return false;
    }

    streamer_->set_callback(streamer_callback_);

    if (!visioner_->camera_stream_ctrl(0, true)) {
        fprintf(stderr, "[SentinelQT] visioner camera_stream_ctrl failed\n");
        return false;
    }

    return true;
}

void Widget::on_frame_ready_(const QImage& image)
{
    // FPS counter
    frameCount_++;
    if (frameCount_ % 30 == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t nowUs = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
        if (lastFpsTsUs_ > 0 && nowUs > lastFpsTsUs_) {
            double fps = 30.0 * 1000000.0 / (nowUs - lastFpsTsUs_);
            ui->fpsLabel->setText(QString("FPS: %1 | 1920x1080").arg(fps, 0, 'f', 1));
        }
        lastFpsTsUs_ = nowUs;
    }

    // Scale and display
    ui->previewLabel->setPixmap(
        QPixmap::fromImage(image).scaled(
            ui->previewLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
}

void Widget::on_btn_start_stream_()
{
    QByteArray url = rtspUrl_.toUtf8();
    if (streamer_->start_stream(0, url.constData())) {
        set_status_("推流中: " + rtspUrl_, "#00d2ff");
        update_button_states_();
    } else {
        set_status_("推流启动失败!", "#e74c3c");
    }
}

void Widget::on_btn_stop_stream_()
{
    if (streamer_->stop_stream(0)) {
        set_status_("推流已停止", "#888899");
        update_button_states_();
    }
}

void Widget::on_btn_start_record_()
{
    int res = config_.value("Record/resolution", 1080).toInt();
    RecordResolution recordRes = (res == 720)
        ? RecordResolution::RES_720P
        : RecordResolution::RES_1080P;

    QByteArray path = recordPath_.toUtf8();
    if (streamer_->start_record(0, path.constData(), recordRes)) {
        set_status_("录像中: " + recordPath_, "#4ecca3");
        update_button_states_();
    } else {
        set_status_("录像启动失败!", "#e74c3c");
    }
}

void Widget::on_btn_stop_record_()
{
    if (streamer_->stop_record(0)) {
        set_status_("录像已停止", "#888899");
        update_button_states_();
    }
}

void Widget::on_streamer_event(int /*camNum*/, StreamerEvent event, const char* detail)
{
    switch (event) {
    case StreamerEvent::STREAM_STARTED:
        set_status_("推流中: " + QString::fromUtf8(detail ? detail : ""), "#00d2ff");
        break;
    case StreamerEvent::STREAM_STOPPED:
        set_status_("推流已停止", "#888899");
        update_button_states_();
        break;
    case StreamerEvent::RECORD_STARTED:
        set_status_("录像中: " + QString::fromUtf8(detail ? detail : ""), "#4ecca3");
        break;
    case StreamerEvent::RECORD_STOPPED:
        set_status_("录像已停止", "#888899");
        update_button_states_();
        break;
    case StreamerEvent::ERROR:
        set_status_("错误: " + QString::fromUtf8(detail ? detail : "unknown"), "#e74c3c");
        break;
    }
}

void Widget::update_button_states_()
{
    bool streaming = streamer_->is_streaming(0);
    bool recording = streamer_->is_recording(0);

    ui->btnStartStream->setEnabled(!streaming);
    ui->btnStopStream->setEnabled(streaming);
    ui->btnStartRecord->setEnabled(!recording);
    ui->btnStopRecord->setEnabled(recording);

    // Update button styles to match state
    ui->btnStartStream->setStyleSheet(streaming
        ? "font-size: 18px; background-color: #555566; color: #999999; border: none; border-radius: 8px;"
        : "font-size: 18px; background-color: #00d2ff; color: #1a1a2e; border: none; border-radius: 8px;");
    ui->btnStopStream->setStyleSheet(streaming
        ? "font-size: 18px; background-color: #e74c3c; color: #ffffff; border: none; border-radius: 8px;"
        : "font-size: 18px; background-color: #555566; color: #999999; border: none; border-radius: 8px;");
    ui->btnStartRecord->setStyleSheet(recording
        ? "font-size: 18px; background-color: #555566; color: #999999; border: none; border-radius: 8px;"
        : "font-size: 18px; background-color: #4ecca3; color: #1a1a2e; border: none; border-radius: 8px;");
    ui->btnStopRecord->setStyleSheet(recording
        ? "font-size: 18px; background-color: #e74c3c; color: #ffffff; border: none; border-radius: 8px;"
        : "font-size: 18px; background-color: #555566; color: #999999; border: none; border-radius: 8px;");
}

void Widget::set_status_(const QString& msg, const QString& color)
{
    ui->statusLabel->setText(msg);
    ui->statusLabel->setStyleSheet(
        QString("font-size: 14px; color: %1; padding: 6px;").arg(color));
}
