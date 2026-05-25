#include "widget.h"
#include "./ui_widget.h"

#include "sentinel-visioner.h"
#include "sentinel_streamer.h"
#include "preview_worker.h"

#include <QCoreApplication>
#include <QDir>
#include <QThread>
#include <QTimer>
#include <QMessageBox>
#include <cstring>
#include <cstdio>

extern "C" {
#include <libavformat/avformat.h>
}

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
    , clockTimer_(nullptr)
    , recordTimer_(nullptr)
    , frameCount_(0)
    , lastFpsTsUs_(0)
    , previewActive_(true)
    , prevCpuTotal_(0)
    , prevCpuIdle_(0)
    , systemRunning_(true)
{
    instance_ = this;
    ui->setupUi(this);

    rtspUrl_ = config_.value("Stream/rtspUrl", "rtsp://192.168.1.100:8554/live/cam0").toString();
    recordDir_ = config_.value("Record/dir", "/mnt/sdcard").toString();

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
    connect(ui->btnVideos, &QPushButton::clicked,
            this, &Widget::on_btn_videos_);
    connect(ui->btnBackToMain, &QPushButton::clicked,
            this, &Widget::on_btn_back_);
    connect(ui->btnRefreshVideos, &QPushButton::clicked,
            this, &Widget::on_btn_refresh_videos_);
    connect(ui->btnStream, &QPushButton::clicked, this, &Widget::on_btn_stream_);
    connect(ui->btnRecord, &QPushButton::clicked, this, &Widget::on_btn_record_);
    connect(ui->btnSystem, &QPushButton::clicked, this, &Widget::on_btn_system_);

    // Clock timer
    clockTimer_ = new QTimer(this);
    connect(clockTimer_, &QTimer::timeout, this, &Widget::update_clock_);
    connect(clockTimer_, &QTimer::timeout, this, &Widget::update_hw_usage_);
    clockTimer_->start(1000);
    update_clock_();
    update_hw_usage_();

    // Recording elapsed-time timer
    recordTimer_ = new QTimer(this);
    connect(recordTimer_, &QTimer::timeout, this, &Widget::update_record_info_);

    if (!init_camera_()) {
        set_status_("相机初始化失败!", "#f85149");
        ui->btnStream->setEnabled(false);
        ui->btnRecord->setEnabled(false);
        return;
    }

    start_preview_();

    set_status_("系统就绪", "#3fb950");
}

Widget::~Widget()
{
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
    fprintf(stderr, "[SentinelQT] start_preview_, worker=%p, previewActive=%d\n",
            (void*)previewWorker_, previewActive_);
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
            "font-size: 13px; color: #1a7a2e; background-color: #F5F0D7;"
            " border: 1px solid #3fb950; border-radius: 6px; padding: 0 16px;");
        ui->previewLabel->setText("预览已关闭");
        set_status_("预览已关闭", "#ffffff");
    } else {
        start_preview_();
        ui->btnTogglePreview->setText("关闭预览");
        ui->btnTogglePreview->setStyleSheet(
            "font-size: 13px; color: #b08800; background-color: #F5F0D7;"
            " border: 1px solid #d29922; border-radius: 6px; padding: 0 16px;");
        ui->previewLabel->setText("等待相机...");
        set_status_("预览已开启", "#58a6ff");
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
            ui->fpsLabel->setText(QString("FPS %1  ·  1920×1080").arg(fps, 0, 'f', 1));
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

void Widget::on_btn_stream_()
{
    if (streamer_->is_streaming(0)) {
        if (streamer_->stop_stream(0)) {
            set_status_("推流已停止", "#ffffff");
            update_button_states_();
        }
    } else {
        QByteArray url = rtspUrl_.toUtf8();
        if (streamer_->start_stream(0, url.constData())) {
            set_status_("推流中: " + rtspUrl_, "#58a6ff");
            update_button_states_();
        } else {
            set_status_("推流启动失败!", "#f85149");
        }
    }
}

// ---- Record ----

void Widget::on_btn_record_()
{
    if (streamer_->is_recording(0)) {
        if (streamer_->stop_record(0)) {
            recordTimer_->stop();
            ui->recordInfoLabel->clear();
            set_status_("录像已停止: " + currentRecordPath_, "#ffffff");
            update_button_states_();
        }
    } else {
        RecordResolution recordRes = (ui->resCombo->currentIndex() == 1)
            ? RecordResolution::RES_720P
            : RecordResolution::RES_1080P;

        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        currentRecordPath_ = recordDir_ + "/record_" + timestamp + ".mp4";

        QByteArray path = currentRecordPath_.toUtf8();
        if (streamer_->start_record(0, path.constData(), recordRes)) {
            recordStartTime_ = QDateTime::currentDateTime();
            recordTimer_->start(1000);
            update_record_info_();
            set_status_("录像中: " + currentRecordPath_, "#3fb950");
            update_button_states_();
        } else {
            set_status_("录像启动失败!", "#f85149");
        }
    }
}

// ---- System ----

void Widget::on_btn_system_()
{
    if (systemRunning_) {
        // Stop recording and streaming first
        if (streamer_->is_recording(0)) {
            streamer_->stop_record(0);
            recordTimer_->stop();
            ui->recordInfoLabel->clear();
        }
        if (streamer_->is_streaming(0)) {
            streamer_->stop_stream(0);
        }
        if (previewActive_) {
            stop_preview_();
        }
        // Pause RGA processing (hardware stream stays active)
        visioner_->camera_pause(0, true);

        systemRunning_ = false;
        ui->btnStream->setEnabled(false);
        ui->btnRecord->setEnabled(false);
        ui->btnTogglePreview->setEnabled(false);
        ui->btnSystem->setText("启动系统");
        ui->btnSystem->setStyleSheet(
            "QPushButton { font-size: 16px; font-weight: 600; color: #e6edf3; background-color: #238636;"
            " border: 1px solid #2ea043; border-radius: 8px; }"
            " QPushButton:hover { background-color: #2ea043; }"
            " QPushButton:pressed { background-color: #196c2e; }");
        ui->previewLabel->setText("系统已停止");
        set_status_("系统已停止", "#ffffff");
    } else {
        // Resume RGA processing
        visioner_->camera_pause(0, false);

        // Start preview
        if (!previewActive_) {
            start_preview_();
            ui->btnTogglePreview->setText("关闭预览");
            ui->btnTogglePreview->setStyleSheet(
                "font-size: 13px; color: #b08800; background-color: #F5F0D7;"
                " border: 1px solid #d29922; border-radius: 6px; padding: 0 16px;");
        }

        systemRunning_ = true;
        ui->btnStream->setEnabled(true);
        ui->btnRecord->setEnabled(true);
        ui->btnTogglePreview->setEnabled(true);
        ui->btnSystem->setText("关闭系统");
        ui->btnSystem->setStyleSheet(
            "QPushButton { font-size: 16px; font-weight: 600; color: #e6edf3; background-color: #da3633;"
            " border: 1px solid #f85149; border-radius: 8px; }"
            " QPushButton:hover { background-color: #f85149; }"
            " QPushButton:pressed { background-color: #b62324; }");
        update_button_states_();
        ui->previewLabel->setText("等待相机...");
        set_status_("系统就绪", "#58a6ff");
    }
}

void Widget::update_clock_()
{
    ui->clockLabel->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));
}

void Widget::update_hw_usage_()
{
    int cpuUsage   = -1;
    int rgaUsage   = -1;
    int npuUsage   = -1;
    int tempC      = -1;

    // ---- CPU ----
    FILE* fp = fopen("/proc/stat", "r");
    if (fp) {
        uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
        int n = fscanf(fp, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                       &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);
        fclose(fp);
        if (n >= 4) {
            uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
            uint64_t totalDelta = total - prevCpuTotal_;
            uint64_t idleDelta  = idle  - prevCpuIdle_;
            if (prevCpuTotal_ > 0 && totalDelta > 0)
                cpuUsage = (int)(100 - (idleDelta * 100 / totalDelta));
            prevCpuTotal_ = total;
            prevCpuIdle_  = idle;
        }
    }

    // ---- RGA ----
    int rgaCores[3] = {-1, -1, -1};
    fp = fopen("/sys/kernel/debug/rkrga/load", "r");
    if (fp) {
        char line[128];
        int idx = 0;
        while (fgets(line, sizeof(line), fp) && idx < 3) {
            int load;
            if (sscanf(line, "         load = %d%%", &load) == 1)
                rgaCores[idx++] = load;
        }
        fclose(fp);
    }

    // ---- NPU ----
    int npuCores[3] = {-1, -1, -1};
    fp = fopen("/sys/kernel/debug/rknpu/load", "r");
    if (fp) {
        char line[256];
        if (fgets(line, sizeof(line), fp)) {
            sscanf(line, "NPU load:  Core0: %d%%, Core1: %d%%, Core2: %d%%",
                   &npuCores[0], &npuCores[1], &npuCores[2]);
        }
        fclose(fp);
    }

    // ---- Temperature ----
    fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (fp) {
        int raw;
        if (fscanf(fp, "%d", &raw) == 1) tempC = raw / 1000;
        fclose(fp);
    }

    // ---- Display ----
    QString text;
    text += tempC >= 0 ? QString("%1°C").arg(tempC) : "--°C";
    text += "  ";
    text += cpuUsage >= 0 ? QString("CPU %1%").arg(cpuUsage) : "CPU --%";
    text += "  ";
    text += QString("RGA %1/%2/%3%")
                .arg(rgaCores[0] >= 0 ? QString::number(rgaCores[0]) : "-")
                .arg(rgaCores[1] >= 0 ? QString::number(rgaCores[1]) : "-")
                .arg(rgaCores[2] >= 0 ? QString::number(rgaCores[2]) : "-");
    text += "  ";
    text += QString("NPU %1/%2/%3%")
                .arg(npuCores[0] >= 0 ? QString::number(npuCores[0]) : "-")
                .arg(npuCores[1] >= 0 ? QString::number(npuCores[1]) : "-")
                .arg(npuCores[2] >= 0 ? QString::number(npuCores[2]) : "-");

    ui->hwLabel->setText(text);
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

// ---- Streamer events ----

void Widget::on_streamer_event(int /*camNum*/, StreamerEvent event, const QString& detail)
{
    switch (event) {
    case StreamerEvent::STREAM_STARTED:
        set_status_("推流中: " + detail, "#58a6ff");
        break;
    case StreamerEvent::STREAM_STOPPED:
        set_status_("推流已停止", "#ffffff");
        update_button_states_();
        break;
    case StreamerEvent::RECORD_STARTED:
        set_status_("录像中: " + detail, "#3fb950");
        break;
    case StreamerEvent::RECORD_STOPPED:
        set_status_("录像已停止", "#ffffff");
        update_button_states_();
        break;
    case StreamerEvent::ERROR:
        set_status_("错误: " + (detail.isEmpty() ? "unknown" : detail), "#f85149");
        break;
    }
}

// ---- Button states ----

void Widget::update_button_states_()
{
    bool streaming = streamer_->is_streaming(0);
    bool recording = streamer_->is_recording(0);

    if (streaming) {
        ui->btnStream->setText("停止推流");
        ui->btnStream->setStyleSheet(
            "QPushButton { font-size: 16px; font-weight: 600; color: #e6edf3; background-color: #da3633;"
            " border: 1px solid #f85149; border-radius: 8px; }"
            " QPushButton:hover { background-color: #f85149; }"
            " QPushButton:pressed { background-color: #b62324; }");
    } else {
        ui->btnStream->setText("启动推流");
        ui->btnStream->setStyleSheet(
            "QPushButton { font-size: 16px; font-weight: 600; color: #e6edf3; background-color: #238636;"
            " border: 1px solid #2ea043; border-radius: 8px; }"
            " QPushButton:hover { background-color: #2ea043; }"
            " QPushButton:pressed { background-color: #196c2e; }");
    }

    if (recording) {
        ui->btnRecord->setText("停止录像");
        ui->btnRecord->setStyleSheet(
            "QPushButton { font-size: 16px; font-weight: 600; color: #e6edf3; background-color: #da3633;"
            " border: 1px solid #f85149; border-radius: 8px; }"
            " QPushButton:hover { background-color: #f85149; }"
            " QPushButton:pressed { background-color: #b62324; }");
    } else {
        ui->btnRecord->setText("启动录像");
        ui->btnRecord->setStyleSheet(
            "QPushButton { font-size: 16px; font-weight: 600; color: #e6edf3; background-color: #1f6feb;"
            " border: 1px solid #388bfd; border-radius: 8px; }"
            " QPushButton:hover { background-color: #388bfd; }"
            " QPushButton:pressed { background-color: #1158c7; }");
    }
}

// ---- Video Management ----

void Widget::on_btn_videos_()
{
    scan_videos_();
    ui->stackedWidget->setCurrentIndex(1);
}

void Widget::on_btn_back_()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void Widget::on_btn_refresh_videos_()
{
    scan_videos_();
    ui->videoStatusLabel->setText("列表已刷新");
}

static QString format_duration_(int64_t durationUs)
{
    if (durationUs <= 0) return "--:--";
    int64_t totalSec = durationUs / 1000000;
    int h = totalSec / 3600;
    int m = (totalSec % 3600) / 60;
    int s = totalSec % 60;
    if (h > 0)
        return QString("%1:%2:%3").arg(h).arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
    return QString("%1:%2").arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
}

void Widget::scan_videos_()
{
    QTableWidget* table = ui->videoTable;
    table->setRowCount(0);

    QDir dir(recordDir_);
    QStringList files = dir.entryList({"*.mp4"}, QDir::Files, QDir::Time);

    if (files.isEmpty()) {
        ui->videoStatusLabel->setText("没有录像文件");
        return;
    }

    table->setRowCount(files.size());

    for (int row = 0; row < files.size(); ++row) {
        QString filePath = recordDir_ + "/" + files[row];

        // Filename
        QTableWidgetItem* nameItem = new QTableWidgetItem(files[row]);
        table->setItem(row, 0, nameItem);

        // Read metadata via libavformat
        QString resText = "--";
        QString durText = "--:--";

        AVFormatContext* ctx = avformat_alloc_context();
        if (ctx && avformat_open_input(&ctx, filePath.toUtf8().constData(), nullptr, nullptr) == 0) {
            avformat_find_stream_info(ctx, nullptr);
            int vs = av_find_best_stream(ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
            if (vs >= 0) {
                int w = ctx->streams[vs]->codecpar->width;
                int h = ctx->streams[vs]->codecpar->height;
                resText = QString("%1×%2").arg(w).arg(h);

                // Calculate duration: prefer stream duration, fallback to container
                int64_t durUs = ctx->streams[vs]->duration;
                if (durUs <= 0) durUs = ctx->duration;
                // Fallback: estimate from frame count and frame rate
                if (durUs <= 0 && ctx->streams[vs]->nb_frames > 0) {
                    AVRational fr = ctx->streams[vs]->avg_frame_rate;
                    if (fr.num > 0 && fr.den > 0) {
                        durUs = (int64_t)ctx->streams[vs]->nb_frames * fr.den
                                * 1000000LL / fr.num;
                    }
                }
                durText = format_duration_(durUs);
            }
            avformat_close_input(&ctx);
        }

        QTableWidgetItem* resItem = new QTableWidgetItem(resText);
        resItem->setTextAlignment(Qt::AlignCenter);
        table->setItem(row, 1, resItem);

        QTableWidgetItem* durItem = new QTableWidgetItem(durText);
        durItem->setTextAlignment(Qt::AlignCenter);
        table->setItem(row, 2, durItem);

        // Delete button
        QPushButton* delBtn = new QPushButton("删除");
        delBtn->setStyleSheet(
            "font-size: 13px; color: #2d3535; background-color: #F5F0D7;"
            " border: 1px solid #f85149; border-radius: 4px; padding: 2px 12px;");
        connect(delBtn, &QPushButton::clicked, this, [this, filePath, row]() {
            QMessageBox::StandardButton reply = QMessageBox::question(
                this, "确认删除",
                "确定要删除这个录像文件吗？\n" + filePath,
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                QFile::remove(filePath);
                scan_videos_();
                ui->videoStatusLabel->setText("已删除");
            }
        });
        table->setCellWidget(row, 3, delBtn);
    }

    table->resizeColumnsToContents();
    // Stretch filename column
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    ui->videoStatusLabel->setText(
        QString("共 %1 个录像文件").arg(files.size()));
}

void Widget::set_status_(const QString& msg, const QString& color)
{
    ui->statusLabel->setText(msg);
    ui->statusLabel->setStyleSheet(
        QString("font-size: 14px; color: %1; padding: 6px;").arg(color));
}
