#include "widget.h"
#include "./ui_widget.h"

#include "sentinel-visioner.h"
#include "sentinel_streamer.h"
#include "preview_worker.h"

#include <QCoreApplication>
#include <QDir>
#include <QThread>
#include <QTimer>
#include <QProcess>
#include <cstring>
#include <cstdio>

Widget* Widget::instance_ = nullptr;

static void streamer_callback_(int camNum, StreamerEvent event, const char* detail)
{
    Widget* w = Widget::instance();
    if (!w) return;
    QString detailStr = detail ? QString::fromUtf8(detail) : QString();
    QMetaObject::invokeMethod(w, [w, camNum, event, detailStr]() {
        w->on_streamer_event(camNum, event, detailStr);
    }, Qt::QueuedConnection);
}

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , visioner_(new SentinelVisioner())
    , streamer_(new SentinelStreamer())
    , previewWorker_(nullptr)
    , previewThread_(nullptr)
    , config_(QCoreApplication::applicationDirPath() + "/config.ini", QSettings::IniFormat)
    , recordTimer_(nullptr)
    , playerProcess_(nullptr)
    , frameCount_(0)
    , lastFpsTsUs_(0)
    , previewActive_(true)
{
    instance_ = this;
    ui->setupUi(this);

    rtspUrl_ = config_.value("Stream/rtspUrl", "rtsp://192.168.1.100:8554/live/cam0").toString();
    recordDir_ = config_.value("Record/dir", "/mnt/sdcard").toString();

    // Populate file list
    refresh_file_list_();

    // Resolution combo
    int res = config_.value("Record/resolution", 1080).toInt();
    ui->resCombo->setCurrentIndex(res == 720 ? 1 : 0);
    connect(ui->resCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
        config_.setValue("Record/resolution", idx == 1 ? 720 : 1080);
    });

    // Button connections
    connect(ui->btnTogglePreview, &QPushButton::clicked,
            this, &Widget::on_btn_toggle_preview_);
    connect(ui->btnPlayRecord, &QPushButton::clicked,
            this, &Widget::on_btn_play_record_);
    connect(ui->btnRefreshFiles, &QPushButton::clicked,
            this, &Widget::on_btn_refresh_files_);
    connect(ui->btnStartStream, &QPushButton::clicked, this, &Widget::on_btn_start_stream_);
    connect(ui->btnStopStream,  &QPushButton::clicked, this, &Widget::on_btn_stop_stream_);
    connect(ui->btnStartRecord, &QPushButton::clicked, this, &Widget::on_btn_start_record_);
    connect(ui->btnStopRecord,  &QPushButton::clicked, this, &Widget::on_btn_stop_record_);

    // Recording elapsed-time timer
    recordTimer_ = new QTimer(this);
    connect(recordTimer_, &QTimer::timeout, this, &Widget::update_record_info_);

    if (!init_camera_()) {
        set_status_("相机初始化失败!", "#e74c3c");
        ui->btnStartStream->setEnabled(false);
        ui->btnStopStream->setEnabled(false);
        ui->btnStartRecord->setEnabled(false);
        ui->btnStopRecord->setEnabled(false);
        return;
    }

    start_preview_();

    set_status_("系统就绪", "#4ecca3");
}

Widget::~Widget()
{
    if (playerProcess_ && playerProcess_->state() == QProcess::Running) {
        playerProcess_->terminate();
        playerProcess_->waitForFinished(3000);
    }
    delete playerProcess_;

    if (visioner_) {
        visioner_->camera_stream_ctrl(0, false);
    }

    stop_preview_();

    if (streamer_) {
        streamer_->remove_camera(0);
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

// ---- Preview ----

void Widget::start_preview_()
{
    if (previewWorker_) return;

    previewWorker_ = new PreviewWorker(visioner_, 0);
    previewThread_ = new QThread(this);
    previewWorker_->moveToThread(previewThread_);
    connect(previewWorker_, &PreviewWorker::frameReady, this, &Widget::on_frame_ready_);
    connect(previewThread_, &QThread::started, previewWorker_, &PreviewWorker::start);
    previewThread_->start();
    previewActive_ = true;
}

void Widget::stop_preview_()
{
    if (!previewWorker_) return;

    previewWorker_->stop();
    if (previewThread_ && previewThread_->isRunning()) {
        previewThread_->quit();
        previewThread_->wait(3000);
    }
    delete previewWorker_;
    previewWorker_ = nullptr;
    delete previewThread_;
    previewThread_ = nullptr;
    previewActive_ = false;
}

void Widget::on_btn_toggle_preview_()
{
    if (previewActive_) {
        stop_preview_();
        ui->btnTogglePreview->setText("启动预览");
        ui->btnTogglePreview->setStyleSheet(
            "font-size: 16px; background-color: #555566; color: #999999;"
            " border: none; border-radius: 6px; padding: 0 12px;");
        ui->previewLabel->setText("预览已关闭");
        set_status_("预览已关闭", "#888899");
    } else {
        start_preview_();
        ui->btnTogglePreview->setText("关闭预览");
        ui->btnTogglePreview->setStyleSheet(
            "font-size: 16px; background-color: #e67e22; color: #ffffff;"
            " border: none; border-radius: 6px; padding: 0 12px;");
        ui->previewLabel->setText("等待相机...");
        set_status_("预览已开启", "#4ecca3");
    }
}

// ---- Frame display ----

void Widget::on_frame_ready_(const QImage& image)
{
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

    ui->previewLabel->setPixmap(
        QPixmap::fromImage(image).scaled(
            ui->previewLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
}

// ---- Stream ----

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

// ---- Record ----

void Widget::on_btn_start_record_()
{
    RecordResolution recordRes = (ui->resCombo->currentIndex() == 1)
        ? RecordResolution::RES_720P
        : RecordResolution::RES_1080P;

    QString resText = (recordRes == RecordResolution::RES_720P) ? "720p" : "1080p";
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    currentRecordPath_ = recordDir_ + "/record_" + timestamp + ".mp4";

    QByteArray path = currentRecordPath_.toUtf8();
    if (streamer_->start_record(0, path.constData(), recordRes)) {
        recordStartTime_ = QDateTime::currentDateTime();
        recordTimer_->start(1000);
        update_record_info_();
        set_status_("录像中: " + currentRecordPath_, "#4ecca3");
        update_button_states_();
    } else {
        set_status_("录像启动失败!", "#e74c3c");
    }
}

void Widget::on_btn_stop_record_()
{
    if (streamer_->stop_record(0)) {
        recordTimer_->stop();
        ui->recordInfoLabel->clear();
        set_status_("录像已停止: " + currentRecordPath_, "#888899");
        update_button_states_();
    }
}

void Widget::update_record_info_()
{
    qint64 elapsed = recordStartTime_.secsTo(QDateTime::currentDateTime());
    int h = elapsed / 3600;
    int m = (elapsed % 3600) / 60;
    int s = elapsed % 60;

    QString resText = (ui->resCombo->currentIndex() == 1) ? "720p" : "1080p";
    ui->recordInfoLabel->setText(
        QString("● REC  %1  %2:%3:%4")
            .arg(resText)
            .arg(h, 2, 10, QChar('0'))
            .arg(m, 2, 10, QChar('0'))
            .arg(s, 2, 10, QChar('0')));
}

// ---- Playback ----

void Widget::refresh_file_list_()
{
    QString current = ui->fileCombo->currentText();
    ui->fileCombo->clear();

    QDir dir(recordDir_);
    QStringList filters;
    filters << "*.mp4";
    QStringList files = dir.entryList(filters, QDir::Files, QDir::Time);

    if (files.isEmpty()) {
        ui->fileCombo->addItem("(无录像文件)");
        return;
    }

    ui->fileCombo->addItems(files);

    // Restore previous selection if still exists
    int idx = ui->fileCombo->findText(current);
    if (idx >= 0) ui->fileCombo->setCurrentIndex(idx);
}

void Widget::on_btn_refresh_files_()
{
    refresh_file_list_();
    set_status_("文件列表已刷新", "#888899");
}

void Widget::on_btn_play_record_()
{
    QString selected = ui->fileCombo->currentText();
    if (selected.isEmpty() || selected.startsWith("(")) {
        set_status_("请先选择一个录像文件", "#e74c3c");
        return;
    }

    QString filePath = recordDir_ + "/" + selected;

    if (streamer_->is_recording(0)) {
        set_status_("请先停止录像再播放", "#e74c3c");
        return;
    }

    // Stop preview to free DRM for ffplay
    if (previewActive_) {
        stop_preview_();
        ui->btnTogglePreview->setText("启动预览");
        ui->btnTogglePreview->setStyleSheet(
            "font-size: 16px; background-color: #555566; color: #999999;"
            " border: none; border-radius: 6px; padding: 0 12px;");
        ui->previewLabel->setText("播放中...");
    }

    if (playerProcess_ && playerProcess_->state() == QProcess::Running) {
        playerProcess_->terminate();
        playerProcess_->waitForFinished(1000);
    }

    if (!playerProcess_) {
        playerProcess_ = new QProcess(this);
    }

    // Use ffplay with DRM/KMS output
    QStringList args;
    args << "-fs" << "-autoexit" << "-infbuf" << filePath;

    playerProcess_->start("ffplay", args);
    set_status_("播放中: " + selected, "#9b59b6");

    // When ffplay exits, restart preview
    connect(playerProcess_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int, QProcess::ExitStatus) {
        if (!previewActive_) {
            start_preview_();
            ui->btnTogglePreview->setText("关闭预览");
            ui->btnTogglePreview->setStyleSheet(
                "font-size: 16px; background-color: #e67e22; color: #ffffff;"
                " border: none; border-radius: 6px; padding: 0 12px;");
            ui->previewLabel->setText("等待相机...");
        }
        set_status_("播放结束", "#888899");
    });
}

// ---- Streamer events ----

void Widget::on_streamer_event(int /*camNum*/, StreamerEvent event, const QString& detail)
{
    switch (event) {
    case StreamerEvent::STREAM_STARTED:
        set_status_("推流中: " + detail, "#00d2ff");
        break;
    case StreamerEvent::STREAM_STOPPED:
        set_status_("推流已停止", "#888899");
        update_button_states_();
        break;
    case StreamerEvent::RECORD_STARTED:
        set_status_("录像中: " + detail, "#4ecca3");
        break;
    case StreamerEvent::RECORD_STOPPED:
        set_status_("录像已停止", "#888899");
        update_button_states_();
        break;
    case StreamerEvent::ERROR:
        set_status_("错误: " + (detail.isEmpty() ? "unknown" : detail), "#e74c3c");
        break;
    }
}

// ---- Button states ----

void Widget::update_button_states_()
{
    bool streaming = streamer_->is_streaming(0);
    bool recording = streamer_->is_recording(0);

    ui->btnStartStream->setEnabled(!streaming);
    ui->btnStopStream->setEnabled(streaming);
    ui->btnStartRecord->setEnabled(!recording);
    ui->btnStopRecord->setEnabled(recording);

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
