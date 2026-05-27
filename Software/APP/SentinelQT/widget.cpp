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

// ---- Styles ----

static const char* STREAM_ON_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #da3633;"
    " border: 1px solid #f85149; border-radius: 5px; }"
    " QPushButton:hover { background-color: #f85149; }"
    " QPushButton:pressed { background-color: #b62324; }";

static const char* STREAM_OFF_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #238636;"
    " border: 1px solid #2ea043; border-radius: 5px; }"
    " QPushButton:hover { background-color: #2ea043; }"
    " QPushButton:pressed { background-color: #196c2e; }"
    " QPushButton:disabled { color: #484f58; background-color: #21262d; border-color: #30363d; }";

static const char* RECORD_ON_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #da3633;"
    " border: 1px solid #f85149; border-radius: 5px; }"
    " QPushButton:hover { background-color: #f85149; }"
    " QPushButton:pressed { background-color: #b62324; }";

static const char* RECORD_OFF_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #1f6feb;"
    " border: 1px solid #388bfd; border-radius: 5px; }"
    " QPushButton:hover { background-color: #388bfd; }"
    " QPushButton:pressed { background-color: #1158c7; }"
    " QPushButton:disabled { color: #484f58; background-color: #21262d; border-color: #30363d; }";

static const char* PAUSE_ON_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #6e7681;"
    " border: 1px solid #8b949e; border-radius: 5px; }"
    " QPushButton:hover { background-color: #8b949e; }"
    " QPushButton:pressed { background-color: #484f58; }";

static const char* PAUSE_OFF_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #d29922;"
    " border: 1px solid #e2b144; border-radius: 5px; }"
    " QPushButton:hover { background-color: #e2b144; }"
    " QPushButton:pressed { background-color: #b08800; }";

static const char* TOGGLE_ON_STYLE =
    "font-size: 12px; color: #b08800; background-color: #F5F0D7;"
    " border: 1px solid #d29922; border-radius: 5px; padding: 0 8px;";

static const char* TOGGLE_OFF_STYLE =
    "font-size: 12px; color: #1a7a2e; background-color: #F5F0D7;"
    " border: 1px solid #3fb950; border-radius: 5px; padding: 0 8px;";

static const char* SYSTEM_ON_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #da3633;"
    " border: 1px solid #f85149; border-radius: 5px; }"
    " QPushButton:hover { background-color: #f85149; }"
    " QPushButton:pressed { background-color: #b62324; }";

static const char* SYSTEM_OFF_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #238636;"
    " border: 1px solid #2ea043; border-radius: 5px; }"
    " QPushButton:hover { background-color: #2ea043; }"
    " QPushButton:pressed { background-color: #196c2e; }";

// ---- Helper: find button for camera ----

static QPushButton* cam_btn(QPushButton* btn0, QPushButton* btn1, int camNum)
{
    return camNum == 0 ? btn0 : btn1;
}

static QLabel* cam_lbl(QLabel* lbl0, QLabel* lbl1, int camNum)
{
    return camNum == 0 ? lbl0 : lbl1;
}

// ---- Constructor ----

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , visioner_(new SentinelVisioner())
    , streamer_(new SentinelStreamer())
    , previewWorker_{nullptr, nullptr}
    , previewThread_{nullptr, nullptr}
    , config_(QCoreApplication::applicationDirPath() + "/config.ini", QSettings::IniFormat)
    , clockTimer_(nullptr)
    , recordTimer_{nullptr, nullptr}
    , frameCount_{0, 0}
    , lastFpsTsUs_{0, 0}
    , previewActive_{false, false}
    , cameraPaused_{false, false}
    , prevCpuTotal_(0)
    , prevCpuIdle_(0)
{
    instance_ = this;
    ui->setupUi(this);

    load_config_();

    // Resolution combo — controls cam0 record resolution only
    int res0 = recordResolution_[0];
    ui->resCombo->setCurrentIndex(res0 == 720 ? 1 : 0);
    connect(ui->resCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
        recordResolution_[0] = (idx == 1) ? 720 : 1080;
        config_.setValue("Camera0/recordResolution", recordResolution_[0]);
    });

    // Per-camera button wiring
    connect(ui->btnToggle0, &QPushButton::clicked, this, [this]() { on_btn_toggle_preview_(0); });
    connect(ui->btnStream0, &QPushButton::clicked, this, [this]() { on_btn_stream_(0); });
    connect(ui->btnRecord0, &QPushButton::clicked, this, [this]() { on_btn_record_(0); });
    connect(ui->btnPause0,  &QPushButton::clicked, this, [this]() { on_btn_pause_(0); });

    connect(ui->btnToggle1, &QPushButton::clicked, this, [this]() { on_btn_toggle_preview_(1); });
    connect(ui->btnStream1, &QPushButton::clicked, this, [this]() { on_btn_stream_(1); });
    connect(ui->btnRecord1, &QPushButton::clicked, this, [this]() { on_btn_record_(1); });
    connect(ui->btnPause1,  &QPushButton::clicked, this, [this]() { on_btn_pause_(1); });

    // Global buttons
    connect(ui->btnVideos, &QPushButton::clicked, this, &Widget::on_btn_videos_);
    connect(ui->btnBackToMain, &QPushButton::clicked, this, &Widget::on_btn_back_);
    connect(ui->btnRefreshVideos, &QPushButton::clicked, this, &Widget::on_btn_refresh_videos_);
    connect(ui->btnSystem, &QPushButton::clicked, this, &Widget::on_btn_system_);

    // Clock timer
    clockTimer_ = new QTimer(this);
    connect(clockTimer_, &QTimer::timeout, this, &Widget::update_clock_);
    connect(clockTimer_, &QTimer::timeout, this, &Widget::update_hw_usage_);
    clockTimer_->start(1000);
    update_clock_();
    update_hw_usage_();

    // Per-camera record timers
    recordTimer_[0] = new QTimer(this);
    recordTimer_[1] = new QTimer(this);
    connect(recordTimer_[0], &QTimer::timeout, this, [this]() { update_record_info_(0); });
    connect(recordTimer_[1], &QTimer::timeout, this, [this]() { update_record_info_(1); });

    // Init both cameras
    bool ok0 = init_camera_(0);
    bool ok1 = init_camera_(1);

    if (!ok0 && !ok1) {
        set_status_("所有相机初始化失败!", "#f85149");
        return;
    }

    if (ok0) start_preview_(0);
    if (ok1) start_preview_(1);

    set_status_("系统就绪", "#3fb950");
    update_button_states_();
}

// ---- Destructor ----

Widget::~Widget()
{
    for (int i = 0; i < 2; ++i) {
        if (visioner_) {
            visioner_->camera_stream_ctrl(i, false);
        }
        stop_preview_(i);
        if (streamer_) {
            streamer_->remove_camera(i);
        }
    }
    delete streamer_;
    delete visioner_;
    delete ui;
}

// ---- Config ----

void Widget::load_config_()
{
    // Camera 0 (ISP) — 默认 1080p
    deviceName_[0]     = config_.value("Camera0/device", "/dev/video11").toString();
    camWidth_[0]       = config_.value("Camera0/width", 1920).toInt();
    camHeight_[0]      = config_.value("Camera0/height", 1080).toInt();
    rtspUrl_[0]        = config_.value("Camera0/streamUrl", "rtsp://127.0.0.1:8554/live/cam0").toString();
    recordResolution_[0] = config_.value("Camera0/recordResolution", 1080).toInt();

    // Camera 1 (USB) — 默认 720p
    deviceName_[1]     = config_.value("Camera1/device", "/dev/video21").toString();
    camWidth_[1]       = config_.value("Camera1/width", 1280).toInt();
    camHeight_[1]      = config_.value("Camera1/height", 720).toInt();
    rtspUrl_[1]        = config_.value("Camera1/streamUrl", "rtsp://127.0.0.1:8554/live/cam1").toString();
    recordResolution_[1] = config_.value("Camera1/recordResolution", 720).toInt();

    // USB 720p 强制夹紧
    if (camWidth_[1] > 1280 || camHeight_[1] > 720) {
        fprintf(stderr, "[SentinelQT] USB 相机分辨率 %dx%d 超出 720p，已限制为 1280x720\n",
                camWidth_[1], camHeight_[1]);
        camWidth_[1]  = 1280;
        camHeight_[1] = 720;
    }
    if (recordResolution_[1] > 720) {
        fprintf(stderr, "[SentinelQT] USB 录像分辨率 %d 超出 720p，已限制为 720\n",
                recordResolution_[1]);
        recordResolution_[1] = 720;
    }

    recordDir_ = config_.value("Record/dir", "/mnt/sdcard").toString();
}

// ---- Camera init ----

bool Widget::init_camera_(int camNum)
{
    std::string devName = deviceName_[camNum].toStdString();
    CameraType camType = (camNum == 0) ? CameraType::ISP_CAM : CameraType::USB_CAM;

    if (!visioner_->add_camera(devName, camWidth_[camNum], camHeight_[camNum],
                                8, camNum, camType)) {
        fprintf(stderr, "[SentinelQT] visioner add_camera cam%d 失败\n", camNum);
        return false;
    }

    if (!streamer_->add_camera(camNum, visioner_)) {
        fprintf(stderr, "[SentinelQT] streamer add_camera cam%d 失败\n", camNum);
        return false;
    }

    if (camNum == 0) {
        streamer_->set_callback(streamer_callback_);
    }

    if (!visioner_->camera_stream_ctrl(camNum, true)) {
        fprintf(stderr, "[SentinelQT] visioner camera_stream_ctrl cam%d 失败\n", camNum);
        return false;
    }

    return true;
}

// ---- Preview ----

void Widget::start_preview_(int camNum)
{
    if (previewWorker_[camNum]) return;

    previewWorker_[camNum] = new PreviewWorker(visioner_, camNum);
    previewThread_[camNum] = new QThread(this);
    previewWorker_[camNum]->moveToThread(previewThread_[camNum]);
    connect(previewWorker_[camNum], &PreviewWorker::frameReady, this,
            [this, camNum](const QImage& img) { on_frame_ready_(camNum, img); });
    connect(previewThread_[camNum], &QThread::started,
            previewWorker_[camNum], &PreviewWorker::start);
    previewThread_[camNum]->start();
    previewActive_[camNum] = true;
}

void Widget::stop_preview_(int camNum)
{
    if (!previewWorker_[camNum]) return;

    previewWorker_[camNum]->stop();
    if (previewThread_[camNum] && previewThread_[camNum]->isRunning()) {
        previewThread_[camNum]->quit();
        previewThread_[camNum]->wait(3000);
    }
    delete previewWorker_[camNum];
    previewWorker_[camNum] = nullptr;
    delete previewThread_[camNum];
    previewThread_[camNum] = nullptr;
    previewActive_[camNum] = false;
}

// ---- Toggle preview ----

void Widget::on_btn_toggle_preview_(int camNum)
{
    QPushButton* btn = cam_btn(ui->btnToggle0, ui->btnToggle1, camNum);
    QLabel* lbl = cam_lbl(ui->previewLabel0, ui->previewLabel1, camNum);

    if (previewActive_[camNum]) {
        stop_preview_(camNum);
        btn->setText("开启预览");
        btn->setStyleSheet(TOGGLE_OFF_STYLE);
        lbl->setText((camNum == 0) ? "ISP 已关闭" : "USB 已关闭");
        set_status_(QString("CAM%1 预览已关闭").arg(camNum), "#ffffff");
    } else {
        start_preview_(camNum);
        btn->setText("关闭预览");
        btn->setStyleSheet(TOGGLE_ON_STYLE);
        lbl->setText("等待相机...");
        set_status_(QString("CAM%1 预览已开启").arg(camNum), "#58a6ff");
    }
}

// ---- Frame display ----

void Widget::on_frame_ready_(int camNum, const QImage& image)
{
    frameCount_[camNum]++;
    if (frameCount_[camNum] % 30 == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t nowUs = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
        if (lastFpsTsUs_[camNum] > 0 && nowUs > lastFpsTsUs_[camNum]) {
            double fps = 30.0 * 1000000.0 / (nowUs - lastFpsTsUs_[camNum]);
            QLabel* fpsLabel = cam_lbl(ui->fpsLabel0, ui->fpsLabel1, camNum);
            fpsLabel->setText(QString("FPS%1 %2  |  %3x%4")
                .arg(camNum)
                .arg(fps, 0, 'f', 1)
                .arg(camWidth_[camNum])
                .arg(camHeight_[camNum]));
        }
        lastFpsTsUs_[camNum] = nowUs;
    }

    QLabel* previewLabel = cam_lbl(ui->previewLabel0, ui->previewLabel1, camNum);
    previewLabel->setPixmap(
        QPixmap::fromImage(image).scaled(
            previewLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
}

// ---- Stream ----

void Widget::on_btn_stream_(int camNum)
{
    QPushButton* btn = cam_btn(ui->btnStream0, ui->btnStream1, camNum);

    if (streamer_->is_streaming(camNum)) {
        if (streamer_->stop_stream(camNum)) {
            set_status_(QString("CAM%1 推流已停止").arg(camNum), "#ffffff");
            update_camera_button_states_(camNum);
        }
    } else {
        QByteArray url = rtspUrl_[camNum].toUtf8();
        if (streamer_->start_stream(camNum, url.constData())) {
            set_status_(QString("CAM%1 推流中: %2").arg(camNum).arg(rtspUrl_[camNum]), "#58a6ff");
            update_camera_button_states_(camNum);
        } else {
            set_status_(QString("CAM%1 推流启动失败!").arg(camNum), "#f85149");
        }
    }
}

// ---- Record ----

void Widget::on_btn_record_(int camNum)
{
    QPushButton* btn = cam_btn(ui->btnRecord0, ui->btnRecord1, camNum);

    if (streamer_->is_recording(camNum)) {
        if (streamer_->stop_record(camNum)) {
            recordTimer_[camNum]->stop();
            ui->recordInfoLabel->clear();
            set_status_(QString("CAM%1 录像已停止: %2").arg(camNum).arg(currentRecordPath_[camNum]), "#ffffff");
            update_camera_button_states_(camNum);
        }
    } else {
        int resVal = recordResolution_[camNum];
        RecordResolution recordRes = (resVal == 720)
            ? RecordResolution::RES_720P
            : RecordResolution::RES_1080P;

        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        currentRecordPath_[camNum] = recordDir_ + QString("/cam%1_record_%2.mp4")
            .arg(camNum).arg(timestamp);

        QByteArray path = currentRecordPath_[camNum].toUtf8();
        if (streamer_->start_record(camNum, path.constData(), recordRes)) {
            recordStartTime_[camNum] = QDateTime::currentDateTime();
            recordTimer_[camNum]->start(1000);
            update_record_info_(camNum);
            set_status_(QString("CAM%1 录像中: %2").arg(camNum).arg(currentRecordPath_[camNum]), "#3fb950");
            update_camera_button_states_(camNum);
        } else {
            set_status_(QString("CAM%1 录像启动失败!").arg(camNum), "#f85149");
        }
    }
}

// ---- Pause ----

void Widget::on_btn_pause_(int camNum)
{
    QPushButton* btn = cam_btn(ui->btnPause0, ui->btnPause1, camNum);
    QLabel* lbl = cam_lbl(ui->previewLabel0, ui->previewLabel1, camNum);

    if (cameraPaused_[camNum]) {
        visioner_->camera_pause(camNum, false);
        if (!previewActive_[camNum]) {
            start_preview_(camNum);
            QPushButton* toggleBtn = cam_btn(ui->btnToggle0, ui->btnToggle1, camNum);
            toggleBtn->setText("关闭预览");
            toggleBtn->setStyleSheet(TOGGLE_ON_STYLE);
        }
        cameraPaused_[camNum] = false;
        btn->setText("暂停");
        btn->setStyleSheet(PAUSE_ON_STYLE);
        lbl->setText("等待相机...");
        set_status_(QString("CAM%1 已恢复").arg(camNum), "#58a6ff");
    } else {
        if (streamer_->is_recording(camNum)) {
            streamer_->stop_record(camNum);
            recordTimer_[camNum]->stop();
            ui->recordInfoLabel->clear();
        }
        if (streamer_->is_streaming(camNum)) {
            streamer_->stop_stream(camNum);
        }
        if (previewActive_[camNum]) {
            stop_preview_(camNum);
        }
        visioner_->camera_pause(camNum, true);
        cameraPaused_[camNum] = true;
        btn->setText("恢复");
        btn->setStyleSheet(PAUSE_OFF_STYLE);
        lbl->setText(QString("CAM%1 已暂停").arg(camNum));
        set_status_(QString("CAM%1 已暂停").arg(camNum), "#ffffff");
    }
    update_camera_button_states_(camNum);
}

// ---- System ----

void Widget::on_btn_system_()
{
    bool anyPaused = cameraPaused_[0] || cameraPaused_[1];

    if (!anyPaused) {
        for (int i = 0; i < 2; ++i) {
            if (streamer_->is_recording(i)) {
                streamer_->stop_record(i);
                recordTimer_[i]->stop();
            }
            if (streamer_->is_streaming(i)) {
                streamer_->stop_stream(i);
            }
            if (previewActive_[i]) {
                stop_preview_(i);
            }
            visioner_->camera_pause(i, true);
            cameraPaused_[i] = true;
        }
        ui->btnSystem->setText("启动系统");
        ui->btnSystem->setStyleSheet(SYSTEM_OFF_STYLE);
        ui->previewLabel0->setText("系统已停止");
        ui->previewLabel1->setText("系统已停止");
        ui->recordInfoLabel->clear();
        set_status_("系统已停止", "#ffffff");
    } else {
        for (int i = 0; i < 2; ++i) {
            visioner_->camera_pause(i, false);
            if (!previewActive_[i]) {
                start_preview_(i);
                QPushButton* toggleBtn = cam_btn(ui->btnToggle0, ui->btnToggle1, i);
                toggleBtn->setText("关闭预览");
                toggleBtn->setStyleSheet(TOGGLE_ON_STYLE);
            }
            cameraPaused_[i] = false;
            QPushButton* pauseBtn = cam_btn(ui->btnPause0, ui->btnPause1, i);
            pauseBtn->setText("暂停");
            pauseBtn->setStyleSheet(PAUSE_ON_STYLE);
        }
        ui->btnSystem->setText("关闭系统");
        ui->btnSystem->setStyleSheet(SYSTEM_ON_STYLE);
        ui->previewLabel0->setText("等待相机...");
        ui->previewLabel1->setText("等待相机...");
        set_status_("系统就绪", "#58a6ff");
    }
    update_button_states_();
}

// ---- Clock & HW ----

void Widget::update_clock_()
{
    ui->clockLabel->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd  HH:mm:ss"));
}

void Widget::update_hw_usage_()
{
    int cpuUsage   = -1;
    int rgaUsage   = -1;
    int npuUsage   = -1;
    int tempC      = -1;

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

    fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (fp) {
        int raw;
        if (fscanf(fp, "%d", &raw) == 1) tempC = raw / 1000;
        fclose(fp);
    }

    QString text;
    text += tempC >= 0 ? QString("%1°C").arg(tempC) : "--°C";
    text += "  ";
    text += cpuUsage >= 0 ? QString("CPU %1%").arg(cpuUsage) : "CPU --%";
    text += "  ";
    text += QString("RGA %1/%2/%3")
                .arg(rgaCores[0] >= 0 ? QString::number(rgaCores[0]) : "-")
                .arg(rgaCores[1] >= 0 ? QString::number(rgaCores[1]) : "-")
                .arg(rgaCores[2] >= 0 ? QString::number(rgaCores[2]) : "-");
    text += "%";
    text += "  ";
    text += QString("NPU %1/%2/%3")
                .arg(npuCores[0] >= 0 ? QString::number(npuCores[0]) : "-")
                .arg(npuCores[1] >= 0 ? QString::number(npuCores[1]) : "-")
                .arg(npuCores[2] >= 0 ? QString::number(npuCores[2]) : "-");
    text += "%";

    ui->hwLabel->setText(text);
}

// ---- Record info ----

void Widget::update_record_info_(int camNum)
{
    qint64 elapsed = recordStartTime_[camNum].secsTo(QDateTime::currentDateTime());
    int h = elapsed / 3600;
    int m = (elapsed % 3600) / 60;
    int s = elapsed % 60;

    QString resText = (recordResolution_[camNum] == 720) ? "720p" : "1080p";
    ui->recordInfoLabel->setText(
        QString("● REC CAM%1 %2  %3:%4:%5")
            .arg(camNum)
            .arg(resText)
            .arg(h, 2, 10, QChar('0'))
            .arg(m, 2, 10, QChar('0'))
            .arg(s, 2, 10, QChar('0')));
}

// ---- Streamer events ----

void Widget::on_streamer_event(int camNum, StreamerEvent event, const QString& detail)
{
    QString camPrefix = QString("CAM%1 ").arg(camNum);
    switch (event) {
    case StreamerEvent::STREAM_STARTED:
        set_status_(camPrefix + "推流中: " + detail, "#58a6ff");
        break;
    case StreamerEvent::STREAM_STOPPED:
        set_status_(camPrefix + "推流已停止", "#ffffff");
        update_camera_button_states_(camNum);
        break;
    case StreamerEvent::RECORD_STARTED:
        set_status_(camPrefix + "录像中: " + detail, "#3fb950");
        break;
    case StreamerEvent::RECORD_STOPPED:
        set_status_(camPrefix + "录像已停止", "#ffffff");
        update_camera_button_states_(camNum);
        break;
    case StreamerEvent::ERROR:
        set_status_(camPrefix + "错误: " + (detail.isEmpty() ? "unknown" : detail), "#f85149");
        break;
    }
}

// ---- Button states ----

void Widget::update_camera_button_states_(int camNum)
{
    QPushButton* btnStream = cam_btn(ui->btnStream0, ui->btnStream1, camNum);
    QPushButton* btnRecord = cam_btn(ui->btnRecord0, ui->btnRecord1, camNum);
    bool streaming = streamer_->is_streaming(camNum);
    bool recording = streamer_->is_recording(camNum);

    if (streaming) {
        btnStream->setText("停止推流");
        btnStream->setStyleSheet(STREAM_ON_STYLE);
    } else {
        btnStream->setText("启动推流");
        btnStream->setStyleSheet(STREAM_OFF_STYLE);
    }

    if (recording) {
        btnRecord->setText("停止录像");
        btnRecord->setStyleSheet(RECORD_ON_STYLE);
    } else {
        btnRecord->setText("启动录像");
        btnRecord->setStyleSheet(RECORD_OFF_STYLE);
    }

    bool controlsEnabled = !cameraPaused_[camNum];
    btnStream->setEnabled(controlsEnabled);
    btnRecord->setEnabled(controlsEnabled);
}

void Widget::update_button_states_()
{
    update_camera_button_states_(0);
    update_camera_button_states_(1);
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

        QTableWidgetItem* nameItem = new QTableWidgetItem(files[row]);
        table->setItem(row, 0, nameItem);

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

                int64_t durUs = ctx->duration;
                if (durUs <= 0 && ctx->streams[vs]->duration > 0) {
                    durUs = av_rescale_q(ctx->streams[vs]->duration,
                                         ctx->streams[vs]->time_base,
                                         AV_TIME_BASE_Q);
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

        QPushButton* delBtn = new QPushButton("删除");
        delBtn->setMinimumWidth(60);
        delBtn->setStyleSheet(
            "font-size: 13px; color: #2d3535; background-color: #F5F0D7;"
            " border: 1px solid #f85149; border-radius: 4px; padding: 3px 14px;");
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
    if (table->columnWidth(3) < 70)
        table->setColumnWidth(3, 70);
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    ui->videoStatusLabel->setText(
        QString("共 %1 个录像文件").arg(files.size()));
}

void Widget::set_status_(const QString& msg, const QString& color)
{
    // 自动识别按相机消息: "CAM0 xxx" 或 "CAM1 xxx"
    if (msg.startsWith("CAM0 ")) {
        camStatus_[0] = msg.mid(5);
        refresh_status_label_();
    } else if (msg.startsWith("CAM1 ")) {
        camStatus_[1] = msg.mid(5);
        refresh_status_label_();
    } else {
        // 全局消息直接显示
        ui->statusLabel->setText(msg);
        ui->statusLabel->setStyleSheet(
            QString("font-size: 12px; color: %1; padding: 2px;").arg(color));
    }
}

void Widget::refresh_status_label_()
{
    QString text;
    if (!camStatus_[0].isEmpty())
        text += "CAM0: " + camStatus_[0];
    if (!camStatus_[1].isEmpty()) {
        if (!text.isEmpty()) text += "  |  ";
        text += "CAM1: " + camStatus_[1];
    }
    if (text.isEmpty()) text = "系统就绪";
    ui->statusLabel->setText(text);
    ui->statusLabel->setStyleSheet("font-size: 12px; color: #ffffff; padding: 2px;");
}
