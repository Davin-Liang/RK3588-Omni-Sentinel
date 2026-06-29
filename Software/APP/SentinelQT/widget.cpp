#include "widget.h"
#include "./ui_widget.h"

#include "sentinel-visioner.h"
#include "SentinelYoloInfer.h"
#include "sentinel_streamer.h"
#include "web_server.h"
#include "preview_worker.h"
#include "fusion_worker.h"
#include "top_down_view.h"
#include "virtual_keyboard.h"
#include "imu_eis.hpp"
#include "vision_eis.hpp"
#include "nvme_worker.h"
#include "NVMeDataManager.h"

#include <QCoreApplication>
#include <QDir>
#include <chrono>
#include <QThread>
#include <QTimer>
#include <QMessageBox>
#include <QScrollArea>
#include <QDoubleValidator>
#include <QIntValidator>
#include <QComboBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QFrame>
#include <QPainter>
#include <QPushButton>
#include <QStyledItemDelegate>
#include <QVBoxLayout>
#include <cstring>
#include <cstdio>
#include <chrono>
#include "json.hpp"

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

static void fusion_warning_callback_(const TrackedTarget& target, void* /*userData*/)
{
    const char* stateStr = "?";
    switch (target.state) {
    case TrackState::Tentative: stateStr = "Tentative"; break;
    case TrackState::FusionTracking:  stateStr = "Fusion"; break;
    case TrackState::PureRadarTracking: stateStr = "PureRadar"; break;
    case TrackState::Lost:  stateStr = "Lost";  break;
    default: break;
    }
    fprintf(stderr,
        "[FusionWarning] target #%u class=%u dist=%.2fm pos=(%.2f,%.2f) "
        "vel=(%.2f,%.2f) state=%s\n",
        target.id, target.classId, target.distanceMeters,
        target.posX, target.posY, target.velX, target.velY, stateStr);

    Widget* w = Widget::instance();
    if (w) {
        uint64_t alertTsUs = target.lastUpdateNs / 1000;
        QMetaObject::invokeMethod(w, [w, alertTsUs, targetId = static_cast<int>(target.id)]() {
            w->on_fusion_alert_backtrack_(targetId, alertTsUs);
        }, Qt::QueuedConnection);
    }
}

void Widget::on_fusion_alert_backtrack_(int targetId, uint64_t alertTsUs)
{
    double backSecs = config_.value("Backtrack/maxBacktrackSeconds", 5.0).toDouble();

    uint64_t startTs = alertTsUs - static_cast<uint64_t>(backSecs * 1000000.0);
    fprintf(stderr,
        "[SentinelQT] ========================================\n"
        "[SentinelQT] alert backtrack triggered!\n"
        "[SentinelQT]   target id    : %d\n"
        "[SentinelQT]   alert ts     : %llu us\n"
        "[SentinelQT]   back seconds : %.1f s\n"
        "[SentinelQT]   time range   : [%llu, %llu] us\n"
        "[SentinelQT] ========================================\n",
        targetId,
        (unsigned long long)alertTsUs,
        backSecs,
        (unsigned long long)startTs,
        (unsigned long long)alertTsUs);

    do_backtrack_(alertTsUs, -1, QString("alert_t%1").arg(targetId));
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

static const char* FUSION_ON_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #da3633;"
    " border: 1px solid #f85149; border-radius: 6px; }"
    " QPushButton:hover { background-color: #f85149; }"
    " QPushButton:pressed { background-color: #b62324; }";

static const char* FUSION_OFF_STYLE =
    "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #238636;"
    " border: 1px solid #2ea043; border-radius: 6px; }"
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
    , webServer_(nullptr)
    , previewWorker_{nullptr, nullptr}
    , previewThread_{nullptr, nullptr}
    , config_(QCoreApplication::applicationDirPath() + "/config.ini", QSettings::IniFormat)
    , clockTimer_(nullptr)
    , recordTimer_{nullptr, nullptr}
    , frameCount_{0, 0}
    , lastFpsTsUs_{0, 0}
    , lastFps_{0.0, 0.0}
    , previewActive_{false, false}
    , cameraPaused_{false, false}
    , prevCpuTotal_(0)
    , prevCpuIdle_(0)
    , lidar_(nullptr)
    , fusion_(nullptr)
    , fusionWorker_(nullptr)
    , fusionThread_(nullptr)
    , fusionStatusTimer_(nullptr)
    , fusionEnabled_(false)
    , topDownView_(nullptr)
    , virtualKeyboard_(nullptr)
    , fusionCamCount_(1)
    , eisReader_(nullptr)
{
    instance_ = this;
    ui->setupUi(this);

    load_config_();

    // Resolution combo — CAM0
    int res0 = recordResolution_[0];
    ui->resCombo->setCurrentIndex(res0 == 720 ? 1 : 0);

    // Resolution combo — CAM1
    int res1 = recordResolution_[1];
    ui->resCombo1->setCurrentIndex(res1 == 720 ? 1 : 0);

    // 居中 QComboBox 文字
    class CenterDelegate : public QStyledItemDelegate {
    public:
        explicit CenterDelegate(QObject* p = nullptr) : QStyledItemDelegate(p) {}
        void paint(QPainter* painter, const QStyleOptionViewItem& option,
                   const QModelIndex& index) const override {
            QStyleOptionViewItem opt = option;
            opt.displayAlignment = Qt::AlignCenter;
            QStyledItemDelegate::paint(painter, opt, index);
        }
    };
    ui->resCombo->setItemDelegate(new CenterDelegate(ui->resCombo));
    ui->resCombo1->setItemDelegate(new CenterDelegate(ui->resCombo1));
    ui->resCombo->setFixedWidth(80);
    ui->resCombo1->setFixedWidth(80);

    connect(ui->resCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
        recordResolution_[0] = (idx == 1) ? 720 : 1080;
        config_.setValue("Camera0/recordResolution", recordResolution_[0]);
    });

    connect(ui->resCombo1, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
        recordResolution_[1] = (idx == 1) ? 720 : 1080;
        config_.setValue("Camera1/recordResolution", recordResolution_[1]);
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
    connect(ui->btnOsd0,    &QPushButton::clicked, this, [this]() { on_btn_osd_(0); });
    connect(ui->btnOsd1,    &QPushButton::clicked, this, [this]() { on_btn_osd_(1); });
    connect(ui->btnEis0,    &QPushButton::clicked, this, [this]() { on_btn_eis_(0); });
    connect(ui->btnEis1,    &QPushButton::clicked, this, [this]() { on_btn_eis_(1); });
    connect(ui->btnLidarOsd0, &QPushButton::clicked, this, [this]() { on_btn_lidar_osd_(0); });
    connect(ui->btnLidarOsd1, &QPushButton::clicked, this, [this]() { on_btn_lidar_osd_(1); });

    // Global buttons
    connect(ui->btnVideos, &QPushButton::clicked, this, &Widget::on_btn_videos_);
    connect(ui->btnBackToMain, &QPushButton::clicked, this, &Widget::on_btn_back_);
    connect(ui->btnRefreshVideos, &QPushButton::clicked, this, &Widget::on_btn_refresh_videos_);
    connect(ui->btnSystem, &QPushButton::clicked, this, &Widget::on_btn_system_);
    connect(ui->btnBacktrack, &QPushButton::clicked, this, &Widget::on_btn_backtrack_page_);

    // Build backtrack page
    build_backtrack_page_();

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

    // 预览默认关闭，由用户手动开启
    // if (ok0) start_preview_(0);
    // if (ok1) start_preview_(1);

    // --- Fusion 初始化 (不启动 radar/fusion，仅创建对象和加载配置) ---

    lidar_ = new SentinelLslidarer();
    fusion_ = new LidarCameraFusion();

    load_lidar_config_();
    load_fusion_config_();

    fusion_->configure_tracker(fusionTrackerCfg_);
    fusion_->enable_tracking(true);

    // TopDownView 替换占位
    topDownView_ = new TopDownView(ui->fusionViewContainer);
    QVBoxLayout* viewLayout = new QVBoxLayout(ui->fusionViewContainer);
    viewLayout->setContentsMargins(0, 0, 0, 0);
    viewLayout->addWidget(topDownView_);

    // 图例帮助按钮（叠放在 TopDownView 右下角）
    QPushButton* legendHelpBtn = new QPushButton("?", topDownView_);
    legendHelpBtn->setFixedSize(18, 18);
    legendHelpBtn->setStyleSheet(
        "QPushButton { font-size: 11px; font-weight: 700; color: #e6edf3;"
        " background: #58a6ff; border-radius: 9px; border: none; }"
        " QPushButton:pressed { background: #388bfd; }");
    legendHelpBtn->setToolTip("点击查看图例说明");
    connect(legendHelpBtn, &QPushButton::clicked, this, [this]() {
        QMessageBox::information(this, "图例说明",
            QString::fromUtf8(
                "● 已确认 — 目标连续命中达到确认帧数，跟踪稳定可靠\n\n"
                "● 待确认 — 目标刚出现或命中帧数不足，处于试探性跟踪阶段\n\n"
                "● 外推中 — 目标短暂丢失后靠速度预测维持，未观测到新数据\n\n"
                "● 告  警 — 目标进入设定的告警距离范围，触发碰撞预警"));
    });
    topDownView_->installEventFilter(this);
    topDownView_->setProperty("legendHelpBtn", QVariant::fromValue<QWidget*>(legendHelpBtn));

    // VirtualKeyboard 替换占位
    virtualKeyboard_ = new VirtualKeyboard(ui->keyboardContainer);
    QVBoxLayout* kbLayout = new QVBoxLayout(ui->keyboardContainer);
    kbLayout->setContentsMargins(0, 0, 0, 0);
    kbLayout->addWidget(virtualKeyboard_);

    // 构建参数 UI
    build_fusion_param_ui_();
    sync_fusion_config_to_ui_();

    // 融合页面按钮连接
    connect(ui->btnFusionToggle, &QPushButton::clicked,
            this, &Widget::on_btn_fusion_toggle_);
    connect(ui->btnBackFromFusion, &QPushButton::clicked,
            this, &Widget::on_btn_back_from_fusion_);

    // 主页面 "融合管理" 按钮
    connect(ui->btnFusion, &QPushButton::clicked, this, [this]() {
        ui->stackedWidget->setCurrentIndex(2);
    });

    // 虚拟键盘可见性
    connect(virtualKeyboard_, &VirtualKeyboard::visibilityChanged,
            this, [this](bool visible) {
        ui->keyboardContainer->setVisible(visible);
    });

    // 融合状态定时器
    fusionStatusTimer_ = new QTimer(this);
    connect(fusionStatusTimer_, &QTimer::timeout,
            this, &Widget::on_fusion_status_update_);

    // eventFilter 安装到所有参数编辑框
    for (auto* le : fusionParamEdits_) {
        le->installEventFilter(this);
    }

    // ---- Web 远程控制服务器 ----
    int webPort = config_.value("WebServer/port", 8080).toInt();
    bool webEnabled = config_.value("WebServer/enabled", true).toBool();
    if (webEnabled) {
        webServer_ = new WebServer(static_cast<uint16_t>(webPort));
        // 命令回调在 WebServer 线程执行，必须通过 BlockingQueuedConnection
        // 转发到 Qt 主线程执行 handle_web_command，并同步等待结果
        webServer_->set_command_handler([this](const std::string& method,
                                                const std::string& path,
                                                const std::string& body) -> std::string {
            fprintf(stderr, "[WebServer] cmdHandler called: %s %s\n", method.c_str(), path.c_str());
            std::string result;
            QMetaObject::invokeMethod(this, [this, &result, &method, &path, &body]() {
                fprintf(stderr, "[WebServer] invokeMethod lambda executing on main thread\n");
                result = handle_web_command(method, path, body);
            }, Qt::BlockingQueuedConnection);
            fprintf(stderr, "[WebServer] cmdHandler returning: %s\n", result.c_str());
            return result;
        });
        if (webServer_->start()) {
            fprintf(stderr, "[SentinelQT] Web server started on port %d\n", webPort);
        } else {
            fprintf(stderr, "[SentinelQT] Web server failed to start on port %d\n", webPort);
        }
    }

    init_nvme_();

    set_status_("系统就绪", "#3fb950");
    update_button_states_();
}

// ---- Destructor ----

Widget::~Widget()
{
    // WebServer 必须在其他组件之前停止，避免 BlockingQueuedConnection 死锁
    if (webServer_) {
        webServer_->stop();
        delete webServer_;
        webServer_ = nullptr;
    }

    // 停止 fusion 子系统
    if (fusionWorker_) {
        fusionWorker_->stop();
        if (fusionThread_ && fusionThread_->isRunning()) {
            fusionThread_->quit();
            fusionThread_->wait(3000);
        }
        delete fusionWorker_;
        fusionWorker_ = nullptr;
        delete fusionThread_;
        fusionThread_ = nullptr;
    }
    if (fusion_) {
        fusion_->stop();
    }
    if (lidar_) {
        lidar_->stop();
    }
    delete fusion_;
    delete lidar_;

    deinit_nvme_();

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
    deinit_eis_();
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

    // USB 分辨率允许 1080p（MJPG 硬件支持 30fps）

    recordDir_ = config_.value("Record/dir", "/mnt/sdcard").toString();

    backtrackDir_ = config_.value("Backtrack/backtrackDir", "/mnt/sdcard/backtrack").toString();
    nvmeDevicePath_ = config_.value("Backtrack/nvmeDevice", "/dev/nvme0n1").toString();

    // EIS 防抖配置：视觉为主 + IMU 辅助。
    // 视觉 EIS 由 sentinel-visioner 在采集线程内对实时相机帧做 LK 光流估计；
    // ICM45686 只提供 gyroRms / vibrationLevel 作为辅助，不再直接输出 offset。
    {
        bool eisCfgEnabled = config_.value("EIS/enabled", false).toBool();
        imuAssistWindowMs_ = config_.value("EIS/imuAssistWindowMs", 200).toUInt();

        for (int c = 0; c < 2; ++c) {
            QString prefix = QString("EIS/Cam%1").arg(c);
            visualEisCfg_[c].camId = c;
            visualEisCfg_[c].inputWidth  = camWidth_[c];
            visualEisCfg_[c].inputHeight = camHeight_[c];
            visualEisCfg_[c].processWidth  = config_.value(prefix + "ProcessWidth", 640).toInt();
            visualEisCfg_[c].processHeight = config_.value(prefix + "ProcessHeight", 360).toInt();
            visualEisCfg_[c].maxCorners = config_.value(prefix + "MaxCorners", 500).toInt();
            visualEisCfg_[c].qualityLevel = config_.value(prefix + "QualityLevel", 0.01).toDouble();
            visualEisCfg_[c].minDistance = config_.value(prefix + "MinDistance", 10.0).toDouble();
            visualEisCfg_[c].minTrackedPoints = config_.value(prefix + "MinTrackedPoints", 30).toInt();
            visualEisCfg_[c].minInliers = config_.value(prefix + "MinInliers", 20).toInt();
            visualEisCfg_[c].ransacThreshold = config_.value(prefix + "RansacThreshold", 3.0).toDouble();
            visualEisCfg_[c].maxOpticalFlow = config_.value(prefix + "MaxOpticalFlow", 80.0).toDouble();
            visualEisCfg_[c].maxOffsetPixel = config_.value(prefix + "MaxOffsetPixel", 80).toInt();
            visualEisCfg_[c].enableImuAdaptiveAlpha = config_.value(prefix + "EnableImuAdaptiveAlpha", true).toBool();
            visualEisCfg_[c].alphaLowVibration = config_.value(prefix + "AlphaLow", 0.30f).toFloat();
            visualEisCfg_[c].alphaMidVibration = config_.value(prefix + "AlphaMid", 0.20f).toFloat();
            visualEisCfg_[c].alphaHighVibration = config_.value(prefix + "AlphaHigh", 0.12f).toFloat();
            visualEisCfg_[c].enableRotationEstimate = config_.value(prefix + "EnableRotationEstimate", true).toBool();
        }

        showEisControl_ = config_.value("EIS/showEisControl", true).toBool();

        bool eisDebug = config_.value("EIS/eisRecordDebug", false).toBool();
        for (int c = 0; c < 2; ++c) {
            streamer_->set_eis_record_debug(c, eisDebug);
        }

        if (eisCfgEnabled) {
            // 只初始化 IMU 辅助线程和回调；真正启用视觉 EIS 在每路相机按钮/配置处完成。
            init_eis_();
        }
    }
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

    // 给 sentinel-visioner 配置该路相机的实时视觉 EIS 参数。
    // 注意：视觉 EIS 的上一帧、轨迹、offset 状态在 SentinelVisioner 内部每路独立维护。
    visualEisCfg_[camNum].inputWidth = camWidth_[camNum];
    visualEisCfg_[camNum].inputHeight = camHeight_[camNum];
    visioner_->set_visual_eis_config(camNum, visualEisCfg_[camNum]);

    if (!streamer_->add_camera(camNum, visioner_)) {
        fprintf(stderr, "[SentinelQT] streamer add_camera cam%d 失败\n", camNum);
        return false;
    }

    if (config_.value("Backtrack/enabled", true).toBool()) {
        int numSlots = config_.value("Backtrack/ringBufferSlots", 150).toInt();
        if (!streamer_->init_record_buffer(camNum, numSlots, camWidth_[camNum], camHeight_[camNum])) {
            fprintf(stderr, "[SentinelQT] init record buffer cam%d failed\n", camNum);
        }
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
        set_status_(QString("相机%1 预览已关闭").arg(camNum), "#ffffff");
    } else {
        start_preview_(camNum);
        btn->setText("关闭预览");
        btn->setStyleSheet(TOGGLE_ON_STYLE);
        lbl->setText("等待相机...");
        set_status_(QString("相机%1 预览已开启").arg(camNum), "#58a6ff");
    }
}

// ---- Frame display ----

void Widget::on_frame_ready_(int camNum, const QImage& image)
{
    // 缓存最新预览帧供 Web MJPEG 端点使用
    if (webServer_) {
        webServer_->set_cached_preview(camNum, image);
    }

    frameCount_[camNum]++;
    if (frameCount_[camNum] % 30 == 0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t nowUs = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
        if (lastFpsTsUs_[camNum] > 0 && nowUs > lastFpsTsUs_[camNum]) {
            double fps = 30.0 * 1000000.0 / (nowUs - lastFpsTsUs_[camNum]);
            lastFps_[camNum] = fps;
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
            set_status_(QString("相机%1 推流已停止").arg(camNum), "#ffffff");
            update_camera_button_states_(camNum);
        }
    } else {
        QByteArray url = rtspUrl_[camNum].toUtf8();
        if (streamer_->start_stream(camNum, url.constData())) {
            set_status_(QString("相机%1 推流中: %2").arg(camNum).arg(rtspUrl_[camNum]), "#58a6ff");
            update_camera_button_states_(camNum);
        } else {
            set_status_(QString("相机%1 推流启动失败!").arg(camNum), "#f85149");
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
            set_status_(QString("相机%1 录像已停止: %2").arg(camNum).arg(currentRecordPath_[camNum]), "#ffffff");
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
            set_status_(QString("相机%1 录像中: %2").arg(camNum).arg(currentRecordPath_[camNum]), "#3fb950");
            update_camera_button_states_(camNum);
        } else {
            set_status_(QString("相机%1 录像启动失败!").arg(camNum), "#f85149");
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
        set_status_(QString("相机%1 已恢复").arg(camNum), "#58a6ff");
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
        lbl->setText(QString("相机%1 已暂停").arg(camNum));
        set_status_(QString("相机%1 已暂停").arg(camNum), "#ffffff");
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
    text += cpuUsage >= 0 ? QString("CPU%1").arg(cpuUsage, 3) : "CPU --";
    text += "  ";
    text += QString("RGA%1/%2/%3")
                .arg(rgaCores[0] >= 0 ? QString::number(rgaCores[0]) : "-")
                .arg(rgaCores[1] >= 0 ? QString::number(rgaCores[1]) : "-")
                .arg(rgaCores[2] >= 0 ? QString::number(rgaCores[2]) : "-");
    text += "  ";
    text += QString("NPU%1/%2/%3")
                .arg(npuCores[0] >= 0 ? QString::number(npuCores[0]) : "-")
                .arg(npuCores[1] >= 0 ? QString::number(npuCores[1]) : "-")
                .arg(npuCores[2] >= 0 ? QString::number(npuCores[2]) : "-");

    ui->hwLabel->setText(text);

    // Web 状态推送 (1Hz)
    if (webServer_ && webServer_->is_running()) {
        webServer_->push_status(get_status_json_());
    }
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
    QString camPrefix = QString("相机%1 ").arg(camNum);
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

    // Web 事件推送
    if (webServer_ && webServer_->is_running()) {
        const char* evName = "unknown";
        switch (event) {
        case StreamerEvent::STREAM_STARTED: evName = "stream_started"; break;
        case StreamerEvent::STREAM_STOPPED: evName = "stream_stopped"; break;
        case StreamerEvent::RECORD_STARTED: evName = "record_started"; break;
        case StreamerEvent::RECORD_STOPPED: evName = "record_stopped"; break;
        case StreamerEvent::ERROR:          evName = "error"; break;
        }
        nlohmann::json j;
        j["cam"] = camNum;
        j["detail"] = detail.toStdString();
        webServer_->push_event(evName, j.dump());
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

    QPushButton* btnPause = cam_btn(ui->btnPause0, ui->btnPause1, camNum);
    if (cameraPaused_[camNum]) {
        btnPause->setText("恢复");
        btnPause->setStyleSheet(PAUSE_OFF_STYLE);
    } else {
        btnPause->setText("暂停");
        btnPause->setStyleSheet(PAUSE_ON_STYLE);
    }

    QPushButton* btnEis = cam_btn(ui->btnEis0, ui->btnEis1, camNum);
    if (eisEnabled_[camNum]) {
        btnEis->setText(QString::fromUtf8("防抖开"));
        btnEis->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; color: #000; "
            "background-color: #4CAF50; border: 1px solid #388E3C; border-radius: 8px; }");
    } else {
        btnEis->setText(QString::fromUtf8("防抖关"));
        btnEis->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; "
            "background-color: #6e7681; border: 1px solid #8b949e; border-radius: 8px; }");
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

// ============================================================================
// Fusion: 配置加载/保存
// ============================================================================

void Widget::load_lidar_config_()
{
    lidarCfg_.serialPort = config_.value("Lidar/device", "/dev/sentinel_lidar")
                               .toString().toStdString();
    lidarCfg_.baudRate      = config_.value("Lidar/baudRate", 460800).toInt();
    lidarCfg_.n10PlusHz     = config_.value("Lidar/hz", 10).toInt();
    lidarCfg_.minRange      = config_.value("Lidar/minRange", 0.15f).toFloat();
    lidarCfg_.maxRange      = config_.value("Lidar/maxRange", 50.0f).toFloat();
    lidarCfg_.angleDisableMin = config_.value("Lidar/angleDisableMin", 0).toInt();
    lidarCfg_.angleDisableMax = config_.value("Lidar/angleDisableMax", 0).toInt();
}

void Widget::load_fusion_config_()
{
    fusionEnabled_  = config_.value("Fusion/enabled", false).toBool();
    fusionCamCount_ = config_.value("Fusion/camCount", 1).toUInt();

    // DBSCAN 聚类
    fusionTrackerCfg_.dbscanEpsMeters =
        config_.value("Fusion/dbscanEpsMeters", 0.5f).toFloat();
    fusionTrackerCfg_.dbscanMinPoints =
        config_.value("Fusion/dbscanMinPoints", 5u).toUInt();
    fusionTrackerCfg_.maxPointDistanceMeters =
        config_.value("Fusion/maxPointDistanceMeters", 30.0f).toFloat();
    fusionTrackerCfg_.maxClusterDistanceMeters =
        config_.value("Fusion/maxClusterDistanceMeters", 10.0f).toFloat();
    fusionTrackerCfg_.clusterPersistenceFrames =
        config_.value("Fusion/clusterPersistenceFrames", 2u).toUInt();
    fusionTrackerCfg_.bboxClaimMaxPixelDist =
        config_.value("Fusion/bboxClaimMaxPixelDist", 100.0f).toFloat();
    fusionTrackerCfg_.minBboxClaimPoints =
        config_.value("Fusion/minBboxClaimPoints", 10u).toUInt();

    // Alpha-Beta 滤波
    fusionTrackerCfg_.alpha =
        config_.value("Fusion/alpha", 0.45f).toFloat();
    fusionTrackerCfg_.beta =
        config_.value("Fusion/beta", 0.2f).toFloat();
    fusionTrackerCfg_.minHitsForVelocity =
        config_.value("Fusion/minHitsForVelocity", 2u).toUInt();

    // 关联
    fusionTrackerCfg_.bboxAssocMaxDistMeters =
        config_.value("Fusion/bboxAssocMaxDistMeters", 0.75f).toFloat();
    fusionTrackerCfg_.orphanAssocMaxDistMeters =
        config_.value("Fusion/orphanAssocMaxDistMeters", 0.5f).toFloat();

    // 生命周期
    fusionTrackerCfg_.minHitsToConfirm =
        config_.value("Fusion/minHitsToConfirm", 3u).toUInt();
    fusionTrackerCfg_.maxTentativeMisses =
        config_.value("Fusion/maxTentativeMisses", 1u).toUInt();
    fusionTrackerCfg_.maxFusionMisses =
        config_.value("Fusion/maxFusionMisses", 2u).toUInt();
    fusionTrackerCfg_.maxLostFrames =
        config_.value("Fusion/maxLostFrames", 20u).toUInt();
    fusionTrackerCfg_.maxTracks =
        config_.value("Fusion/maxTracks", 50u).toUInt();

    // 告警
    fusionTrackerCfg_.warningEnterDistMeters =
        config_.value("Fusion/warningEnterDistMeters", 0.5f).toFloat();
    fusionTrackerCfg_.warningExitDistMeters =
        config_.value("Fusion/warningExitDistMeters", 0.6f).toFloat();
    fusionTrackerCfg_.minConfirmedAgeForWarning =
        config_.value("Fusion/minConfirmedAgeForWarning", 2u).toUInt();
    fusionTrackerCfg_.warningCooldownNs =
        config_.value("Fusion/warningCooldownNs", 2000000000ULL).toULongLong();

    // 可视化
    fusionTrackerCfg_.clusterVisOpacity =
        config_.value("Fusion/clusterVisOpacity", 0.3f).toFloat();
    fusionTrackerCfg_.radarRangeMeters =
        config_.value("Fusion/radarRangeMeters", 10.0f).toFloat();

    // Camera 0
    fusionCamCfg_[0].fx = config_.value("Fusion/Cam0Fx", 400.0f).toFloat();
    fusionCamCfg_[0].fy = config_.value("Fusion/Cam0Fy", 400.0f).toFloat();
    fusionCamCfg_[0].cx = config_.value("Fusion/Cam0Cx", 320.0f).toFloat();
    fusionCamCfg_[0].cy = config_.value("Fusion/Cam0Cy", 240.0f).toFloat();
    fusionCamCfg_[0].imgWidth  = config_.value("Fusion/Cam0ImgWidth", 640u).toUInt();
    fusionCamCfg_[0].imgHeight = config_.value("Fusion/Cam0ImgHeight", 480u).toUInt();
    for (int i = 0; i < 16; ++i) {
        fusionCamCfg_[0].tLidarToCam[i] =
            config_.value(QString("Fusion/Cam0T%1").arg(i), 0.0f).toFloat();
    }
    // Camera 1
    fusionCamCfg_[1].fx = config_.value("Fusion/Cam1Fx", 400.0f).toFloat();
    fusionCamCfg_[1].fy = config_.value("Fusion/Cam1Fy", 400.0f).toFloat();
    fusionCamCfg_[1].cx = config_.value("Fusion/Cam1Cx", 320.0f).toFloat();
    fusionCamCfg_[1].cy = config_.value("Fusion/Cam1Cy", 240.0f).toFloat();
    fusionCamCfg_[1].imgWidth  = config_.value("Fusion/Cam1ImgWidth", 640u).toUInt();
    fusionCamCfg_[1].imgHeight = config_.value("Fusion/Cam1ImgHeight", 480u).toUInt();
    for (int i = 0; i < 16; ++i) {
        fusionCamCfg_[1].tLidarToCam[i] =
            config_.value(QString("Fusion/Cam1T%1").arg(i), 0.0f).toFloat();
    }
}

void Widget::save_fusion_config_()
{
    config_.setValue("Fusion/enabled", fusionEnabled_);
    config_.setValue("Fusion/camCount", fusionCamCount_);

    config_.setValue("Fusion/dbscanEpsMeters", fusionTrackerCfg_.dbscanEpsMeters);
    config_.setValue("Fusion/dbscanMinPoints", fusionTrackerCfg_.dbscanMinPoints);
    config_.setValue("Fusion/maxPointDistanceMeters", fusionTrackerCfg_.maxPointDistanceMeters);
    config_.setValue("Fusion/maxClusterDistanceMeters", fusionTrackerCfg_.maxClusterDistanceMeters);
    config_.setValue("Fusion/clusterPersistenceFrames", fusionTrackerCfg_.clusterPersistenceFrames);
    config_.setValue("Fusion/bboxClaimMaxPixelDist", fusionTrackerCfg_.bboxClaimMaxPixelDist);
    config_.setValue("Fusion/minBboxClaimPoints", fusionTrackerCfg_.minBboxClaimPoints);
    config_.setValue("Fusion/alpha", fusionTrackerCfg_.alpha);
    config_.setValue("Fusion/beta", fusionTrackerCfg_.beta);
    config_.setValue("Fusion/minHitsForVelocity", fusionTrackerCfg_.minHitsForVelocity);
    config_.setValue("Fusion/bboxAssocMaxDistMeters", fusionTrackerCfg_.bboxAssocMaxDistMeters);
    config_.setValue("Fusion/orphanAssocMaxDistMeters", fusionTrackerCfg_.orphanAssocMaxDistMeters);
    config_.setValue("Fusion/minHitsToConfirm", fusionTrackerCfg_.minHitsToConfirm);
    config_.setValue("Fusion/maxTentativeMisses", fusionTrackerCfg_.maxTentativeMisses);
    config_.setValue("Fusion/maxFusionMisses", fusionTrackerCfg_.maxFusionMisses);
    config_.setValue("Fusion/maxLostFrames", fusionTrackerCfg_.maxLostFrames);
    config_.setValue("Fusion/maxTracks", fusionTrackerCfg_.maxTracks);
    config_.setValue("Fusion/warningEnterDistMeters", fusionTrackerCfg_.warningEnterDistMeters);
    config_.setValue("Fusion/warningExitDistMeters", fusionTrackerCfg_.warningExitDistMeters);
    config_.setValue("Fusion/minConfirmedAgeForWarning", fusionTrackerCfg_.minConfirmedAgeForWarning);
    config_.setValue("Fusion/warningCooldownNs", (qulonglong)fusionTrackerCfg_.warningCooldownNs);
    config_.setValue("Fusion/clusterVisOpacity", fusionTrackerCfg_.clusterVisOpacity);
    config_.setValue("Fusion/radarRangeMeters", fusionTrackerCfg_.radarRangeMeters);

    config_.setValue("Fusion/Cam0Fx", fusionCamCfg_[0].fx);
    config_.setValue("Fusion/Cam0Fy", fusionCamCfg_[0].fy);
    config_.setValue("Fusion/Cam0Cx", fusionCamCfg_[0].cx);
    config_.setValue("Fusion/Cam0Cy", fusionCamCfg_[0].cy);

    config_.setValue("Fusion/Cam1Fx", fusionCamCfg_[1].fx);
    config_.setValue("Fusion/Cam1Fy", fusionCamCfg_[1].fy);
    config_.setValue("Fusion/Cam1Cx", fusionCamCfg_[1].cx);
    config_.setValue("Fusion/Cam1Cy", fusionCamCfg_[1].cy);

    ++fusionConfigVersion_;
    config_.sync();
}

// ============================================================================
// Fusion: 参数 UI 构建
// ============================================================================

void Widget::build_fusion_param_ui_()
{
    QWidget* content = ui->paramScrollContent;
    QVBoxLayout* layout = new QVBoxLayout(content);
    layout->setSpacing(2);
    layout->setContentsMargins(6, 6, 6, 6);

    // 参数说明映射
    QMap<QString, QString> descriptions;
    descriptions["dbscanEpsMeters"] =
        QString::fromUtf8("DBSCAN邻域半径，两点距离小于此值归为同一簇");
    descriptions["dbscanMinPoints"] =
        QString::fromUtf8("DBSCAN核心点所需最小邻居数，不足则视为噪声");
    descriptions["maxClusterDistanceMeters"] =
        QString::fromUtf8("簇质心到LiDAR原点的最大距离，更远的簇被丢弃");
    descriptions["clusterPersistenceFrames"] =
        QString::fromUtf8("簇需连续出现多少帧后才输出为正式检测（过滤闪烁）");
    descriptions["bboxClaimMaxPixelDist"] =
        QString::fromUtf8("簇投影点离bbox边缘的最大像素距离，超出则不被认领");
    descriptions["minBboxClaimPoints"] =
        QString::fromUtf8("bbox只认领点数>=此值的簇，过滤远处噪声");
    descriptions["alpha"] =
        QString::fromUtf8("位置滤波平滑系数，越大越信任当前观测（响应快但抖动大）");
    descriptions["beta"] =
        QString::fromUtf8("速度估计平滑系数，越大速度响应越快但噪声也越大");
    descriptions["bboxAssocMaxDistMeters"] =
        QString::fromUtf8("检测与航迹匹配的硬门限距离，超过此值直接不关联");
    descriptions["orphanAssocMaxDistMeters"] =
        QString::fromUtf8("孤儿簇与航迹匹配的硬门限距离");
    descriptions["minHitsToConfirm"] =
        QString::fromUtf8("连续命中帧数阈值，Tentative→FusionTracking");
    descriptions["maxLostFrames"] =
        QString::fromUtf8("Lost状态最大存活帧数，超时则删除航迹");
    descriptions["maxTracks"] =
        QString::fromUtf8("同时跟踪的目标数量上限，满时淘汰最老Lost");
    descriptions["warningEnterDistMeters"] =
        QString::fromUtf8("目标进入此距离内触发告警（迟滞进入阈值）");
    descriptions["warningExitDistMeters"] =
        QString::fromUtf8("目标离开此距离外解除告警（迟滞退出阈值）");
    descriptions["cam0_fx"] = QString::fromUtf8("X轴焦距（像素），影响水平投影缩放");
    descriptions["cam0_fy"] = QString::fromUtf8("Y轴焦距（像素），影响垂直投影缩放");
    descriptions["cam0_cx"] = QString::fromUtf8("主点X坐标（像素），图像水平中心");
    descriptions["cam0_cy"] = QString::fromUtf8("主点Y坐标（像素），图像垂直中心");
    descriptions["cam1_fx"] = QString::fromUtf8("X轴焦距（像素）");
    descriptions["cam1_fy"] = QString::fromUtf8("Y轴焦距（像素）");
    descriptions["cam1_cx"] = QString::fromUtf8("主点X坐标（像素）");
    descriptions["cam1_cy"] = QString::fromUtf8("主点Y坐标（像素）");

    // 为每个 section 创建独立的说明标签
    auto makeDescLabel = [&]() -> QLabel* {
        QLabel* lbl = new QLabel(content);
        lbl->setWordWrap(true);
        lbl->setStyleSheet(
            "font-size: 12px; color: #2d3535; background-color: #e8f0fe;"
            " border: 1px solid #58a6ff; border-radius: 4px; padding: 6px; margin: 2px 0;");
        lbl->hide();
        layout->addWidget(lbl);
        return lbl;
    };

    auto makeDescTimer = [&](QLabel* lbl) -> QTimer* {
        QTimer* t = new QTimer(content);
        t->setSingleShot(true);
        QObject::connect(t, &QTimer::timeout, lbl, &QLabel::hide);
        return t;
    };

    QLabel* trackerDescLabel = nullptr;
    QTimer* trackerDescTimer = nullptr;
    QLabel* cam0DescLabel = nullptr;
    QTimer* cam0DescTimer = nullptr;
    QLabel* cam1DescLabel = nullptr;
    QTimer* cam1DescTimer = nullptr;

    auto addSection = [&](const QString& title, QLabel*& descLabel, QTimer*& descTimer) {
        descLabel = makeDescLabel();
        descTimer = makeDescTimer(descLabel);
        QLabel* lbl = new QLabel(title, content);
        lbl->setStyleSheet(
            "font-size: 13px; font-weight: 600; color: #58a6ff;"
            " background: transparent; border: none; padding: 6px 0 2px 0;");
        layout->addWidget(lbl);
    };

    auto addParam = [&](const QString& key, const QString& label,
                        const QString& suffix, bool isFloat, bool isInt,
                        QLabel* descLabel, QTimer* descTimer) {
        QHBoxLayout* row = new QHBoxLayout();
        row->setSpacing(2);

        QPushButton* helpBtn = new QPushButton("?", content);
        helpBtn->setFixedSize(18, 18);
        helpBtn->setStyleSheet(
            "QPushButton { font-size: 10px; font-weight: 700; color: #58a6ff;"
            " background-color: #F5F0D7; border: 1px solid #58a6ff;"
            " border-radius: 9px; }"
            "QPushButton:pressed { background-color: #d4e6f1; }");
        QString desc = descriptions.value(key);
        if (!desc.isEmpty() && descLabel) {
            QObject::connect(helpBtn, &QPushButton::clicked, content, [descLabel, descTimer, desc]() {
                descLabel->setText(desc);
                descLabel->show();
                descTimer->start(4000);
            });
        } else {
            helpBtn->setEnabled(false);
            helpBtn->setStyleSheet(
                "QPushButton { font-size: 10px; color: #8b949e;"
                " background-color: #F5F0D7; border: 1px solid #d0d7de;"
                " border-radius: 9px; }");
        }
        row->addWidget(helpBtn);

        QLabel* nameLbl = new QLabel(label, content);
        nameLbl->setMinimumWidth(95);
        nameLbl->setStyleSheet(
            "font-size: 11px; color: #4a5555; background: transparent; border: none;");
        QLineEdit* edit = new QLineEdit(content);
        edit->setMinimumWidth(60);
        edit->setMaximumWidth(100);
        edit->setMaximumHeight(28);
        edit->setStyleSheet(
            "font-size: 12px; color: #2d3535; background-color: #F5F0D7;"
            " border: 1px solid #8b949e; border-radius: 4px; padding: 2px 4px;");
        if (isFloat) {
            edit->setValidator(new QDoubleValidator(-999.0, 999.0, 3, edit));
        } else if (isInt) {
            edit->setValidator(new QIntValidator(0, 9999, edit));
        }
        QLabel* suffixLbl = new QLabel(suffix, content);
        suffixLbl->setStyleSheet(
            "font-size: 11px; color: #4a5555; background: transparent; border: none;");
        suffixLbl->setMaximumWidth(24);
        row->addWidget(nameLbl);
        row->addWidget(edit);
        row->addWidget(suffixLbl);
        layout->addLayout(row);
        fusionParamEdits_[key] = edit;
        connect(edit, &QLineEdit::editingFinished,
                this, &Widget::on_fusion_param_changed_);
    };

    addSection(QString::fromUtf8("跟踪器参数"), trackerDescLabel, trackerDescTimer);
    addParam("dbscanEpsMeters",           QString::fromUtf8("DBSCAN半径"),    "m",  true,  false, trackerDescLabel, trackerDescTimer);
    addParam("dbscanMinPoints",           QString::fromUtf8("最小点数"),      "",   false, true,  trackerDescLabel, trackerDescTimer);
    addParam("maxClusterDistanceMeters",  QString::fromUtf8("最大聚类距离"),   "m",  true,  false, trackerDescLabel, trackerDescTimer);
    addParam("clusterPersistenceFrames",  QString::fromUtf8("持久帧数"),      "",   false, true,  trackerDescLabel, trackerDescTimer);
    addParam("bboxClaimMaxPixelDist",     QString::fromUtf8("bbox认领像素"),   "px", true,  false, trackerDescLabel, trackerDescTimer);
    addParam("minBboxClaimPoints",        QString::fromUtf8("认领最小点数"),   "",   false, true,  trackerDescLabel, trackerDescTimer);
    addParam("alpha",                     QString::fromUtf8("Alpha增益"),     "",   true,  false, trackerDescLabel, trackerDescTimer);
    addParam("beta",                      QString::fromUtf8("Beta增益"),      "",   true,  false, trackerDescLabel, trackerDescTimer);
    addParam("bboxAssocMaxDistMeters",    QString::fromUtf8("关联门限"),      "m",  true,  false, trackerDescLabel, trackerDescTimer);
    addParam("orphanAssocMaxDistMeters",  QString::fromUtf8("孤儿门限"),      "m",  true,  false, trackerDescLabel, trackerDescTimer);
    addParam("minHitsToConfirm",          QString::fromUtf8("确认帧数"),      "",   false, true,  trackerDescLabel, trackerDescTimer);
    addParam("maxLostFrames",             QString::fromUtf8("丢失帧数"),      "",   false, true,  trackerDescLabel, trackerDescTimer);
    addParam("maxTracks",                 QString::fromUtf8("最大航迹"),      "",   false, true,  trackerDescLabel, trackerDescTimer);
    addParam("warningEnterDistMeters",    QString::fromUtf8("告警距离"),      "m",  true,  false, trackerDescLabel, trackerDescTimer);
    addParam("warningExitDistMeters",     QString::fromUtf8("解除距离"),      "m",  true,  false, trackerDescLabel, trackerDescTimer);

    addSection(QString::fromUtf8("相机参数 CAM0"), cam0DescLabel, cam0DescTimer);
    addParam("cam0_fx", "fx", "", true, false, cam0DescLabel, cam0DescTimer);
    addParam("cam0_fy", "fy", "", true, false, cam0DescLabel, cam0DescTimer);
    addParam("cam0_cx", "cx", "", true, false, cam0DescLabel, cam0DescTimer);
    addParam("cam0_cy", "cy", "", true, false, cam0DescLabel, cam0DescTimer);

    addSection(QString::fromUtf8("相机参数 CAM1"), cam1DescLabel, cam1DescTimer);
    addParam("cam1_fx", "fx", "", true, false, cam1DescLabel, cam1DescTimer);
    addParam("cam1_fy", "fy", "", true, false, cam1DescLabel, cam1DescTimer);
    addParam("cam1_cx", "cx", "", true, false, cam1DescLabel, cam1DescTimer);
    addParam("cam1_cy", "cy", "", true, false, cam1DescLabel, cam1DescTimer);

    layout->addStretch();
}

// ============================================================================
// Fusion: UI ↔ Config 同步
// ============================================================================

void Widget::sync_fusion_config_to_ui_()
{
    auto setVal = [this](const QString& key, float v) {
        if (fusionParamEdits_.contains(key))
            fusionParamEdits_[key]->setText(QString::number(v, 'f', 3));
    };
    auto setInt = [this](const QString& key, uint32_t v) {
        if (fusionParamEdits_.contains(key))
            fusionParamEdits_[key]->setText(QString::number(v));
    };

    setVal("dbscanEpsMeters",           fusionTrackerCfg_.dbscanEpsMeters);
    setInt("dbscanMinPoints",           fusionTrackerCfg_.dbscanMinPoints);
    setVal("maxClusterDistanceMeters",  fusionTrackerCfg_.maxClusterDistanceMeters);
    setInt("clusterPersistenceFrames",  fusionTrackerCfg_.clusterPersistenceFrames);
    setVal("bboxClaimMaxPixelDist",     fusionTrackerCfg_.bboxClaimMaxPixelDist);
    setInt("minBboxClaimPoints",        fusionTrackerCfg_.minBboxClaimPoints);
    setVal("alpha",                     fusionTrackerCfg_.alpha);
    setVal("beta",                      fusionTrackerCfg_.beta);
    setVal("bboxAssocMaxDistMeters",    fusionTrackerCfg_.bboxAssocMaxDistMeters);
    setVal("orphanAssocMaxDistMeters",  fusionTrackerCfg_.orphanAssocMaxDistMeters);
    setInt("minHitsToConfirm",          fusionTrackerCfg_.minHitsToConfirm);
    setInt("maxLostFrames",             fusionTrackerCfg_.maxLostFrames);
    setInt("maxTracks",                 fusionTrackerCfg_.maxTracks);
    setVal("warningEnterDistMeters",    fusionTrackerCfg_.warningEnterDistMeters);
    setVal("warningExitDistMeters",     fusionTrackerCfg_.warningExitDistMeters);

    setVal("cam0_fx", fusionCamCfg_[0].fx);
    setVal("cam0_fy", fusionCamCfg_[0].fy);
    setVal("cam0_cx", fusionCamCfg_[0].cx);
    setVal("cam0_cy", fusionCamCfg_[0].cy);

    setVal("cam1_fx", fusionCamCfg_[1].fx);
    setVal("cam1_fy", fusionCamCfg_[1].fy);
    setVal("cam1_cx", fusionCamCfg_[1].cx);
    setVal("cam1_cy", fusionCamCfg_[1].cy);
}

void Widget::sync_ui_to_fusion_config_()
{
    auto getVal = [this](const QString& key) -> float {
        if (fusionParamEdits_.contains(key))
            return fusionParamEdits_[key]->text().toFloat();
        return 0.0f;
    };
    auto getInt = [this](const QString& key) -> uint32_t {
        if (fusionParamEdits_.contains(key))
            return fusionParamEdits_[key]->text().toUInt();
        return 0;
    };

    fusionTrackerCfg_.dbscanEpsMeters           = getVal("dbscanEpsMeters");
    fusionTrackerCfg_.dbscanMinPoints           = getInt("dbscanMinPoints");
    fusionTrackerCfg_.maxClusterDistanceMeters  = getVal("maxClusterDistanceMeters");
    fusionTrackerCfg_.clusterPersistenceFrames  = getInt("clusterPersistenceFrames");
    fusionTrackerCfg_.bboxClaimMaxPixelDist     = getVal("bboxClaimMaxPixelDist");
    fusionTrackerCfg_.minBboxClaimPoints        = getInt("minBboxClaimPoints");
    fusionTrackerCfg_.alpha                     = getVal("alpha");
    fusionTrackerCfg_.beta                      = getVal("beta");
    fusionTrackerCfg_.bboxAssocMaxDistMeters    = getVal("bboxAssocMaxDistMeters");
    fusionTrackerCfg_.orphanAssocMaxDistMeters  = getVal("orphanAssocMaxDistMeters");
    fusionTrackerCfg_.minHitsToConfirm          = getInt("minHitsToConfirm");
    fusionTrackerCfg_.maxLostFrames             = getInt("maxLostFrames");
    fusionTrackerCfg_.maxTracks                 = getInt("maxTracks");
    fusionTrackerCfg_.warningEnterDistMeters    = getVal("warningEnterDistMeters");
    fusionTrackerCfg_.warningExitDistMeters     = getVal("warningExitDistMeters");

    fusionCamCfg_[0].fx = getVal("cam0_fx");
    fusionCamCfg_[0].fy = getVal("cam0_fy");
    fusionCamCfg_[0].cx = getVal("cam0_cx");
    fusionCamCfg_[0].cy = getVal("cam0_cy");

    fusionCamCfg_[1].fx = getVal("cam1_fx");
    fusionCamCfg_[1].fy = getVal("cam1_fy");
    fusionCamCfg_[1].cx = getVal("cam1_cx");
    fusionCamCfg_[1].cy = getVal("cam1_cy");
}

// ============================================================================
// Fusion: 事件处理
// ============================================================================

void Widget::on_btn_osd_(int camNum)
{
    osdEnabled_[camNum] = !osdEnabled_[camNum];
    QPushButton* btn = (camNum == 0) ? ui->btnOsd0 : ui->btnOsd1;

    if (osdEnabled_[camNum]) {
        // 懒加载：首次开启 OSD 时创建 yoloInfer_
        if (!yoloInfer_) {
            SentinelYoloInferConfig inferCfg;
            inferCfg.modelPath = config_.value("Fusion/modelPath",
                "./models/yolov8n.rknn").toString().toStdString();
            inferCfg.boxThreshold = 0.25f;
            inferCfg.waitTimeoutMs = 200;
            yoloInfer_ = new SentinelYoloInfer(visioner_, inferCfg);
            for (int c = 0; c < 2; ++c) {
                if (!yoloInfer_->create_infer_thread(c))
                    fprintf(stderr, "[SentinelQT] OSD infer thread cam %d failed\n", c);
            }
            streamer_->set_osd_provider(
                [this](int cam, std::vector<StreamOsdBBox>& out, int) {
                    // 清空队列只保留最新一帧，避免 FIFO 积压导致 OSD 延迟
                    YoloBBoxList boxes;
                    bool gotAny = false;
                    while (yoloInfer_->try_get_osd_result(cam, boxes, 0))
                        gotAny = true;
                    if (!gotAny) return false;
                    for (const auto& b : boxes) {
                        out.push_back({b.x1, b.y1, b.x2, b.y2,
                                       b.classId, b.confidence});
                    }
                    return true;
                });
            if (fusionEnabled_) {
                fusion_->set_detection_provider(
                    [this](int cam, std::vector<YoloBBox>& out, int timeoutMs) {
                        return yoloInfer_->try_get_fusion_result(cam, out, timeoutMs);
                    });
            }
        }
        streamer_->set_stream_osd_mode(camNum, StreamOsdMode::WITH_OSD);
        btn->setText("框去除");
        btn->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; color: #000; "
            "background-color: #4CAF50; border: 1px solid #388E3C; border-radius: 8px; }");
        fprintf(stderr, "[SentinelQT] cam %d OSD enabled\n", camNum);
    } else {
        streamer_->set_stream_osd_mode(camNum, StreamOsdMode::WITHOUT_OSD);
        btn->setText("框叠加");
        btn->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; "
            "background-color: #6e7681; border: 1px solid #8b949e; border-radius: 8px; }");
        fprintf(stderr, "[SentinelQT] cam %d OSD disabled\n", camNum);

        if (!osdEnabled_[0] && !osdEnabled_[1] && !fusionEnabled_) {
            yoloInfer_->stop_all();
            delete yoloInfer_;
            yoloInfer_ = nullptr;
        }
    }
}

void Widget::setup_lidar_osd_provider_()
{
    if (!fusion_) return;
    streamer_->set_lidar_osd_provider(
        [this](int camNum, std::vector<StreamLidarOsdBBox>& out, int timeoutMs) {
            LidarOsdSnapshot snap;
            if (!fusion_->try_get_lidar_osd_snapshot(snap, timeoutMs))
                return false;
            for (uint32_t c = 0; c < snap.camCount; ++c) {
                if (snap.cameras[c].camNum != camNum) continue;
                auto& cam = snap.cameras[c];
                uint32_t offset = 0;
                for (uint32_t b = 0; b < cam.bboxCount; ++b) {
                    StreamLidarOsdBBox box;
                    box.x1 = cam.bboxX1[b];
                    box.y1 = cam.bboxY1[b];
                    box.x2 = cam.bboxX2[b];
                    box.y2 = cam.bboxY2[b];
                    box.pointCount = cam.bboxPointCounts[b];
                    box.pointsU.assign(cam.bboxPointU.begin() + offset,
                                       cam.bboxPointU.begin() + offset + box.pointCount);
                    box.pointsV.assign(cam.bboxPointV.begin() + offset,
                                       cam.bboxPointV.begin() + offset + box.pointCount);
                    box.distanceMeters = (b < cam.bboxClusterDistMeters.size())
                        ? cam.bboxClusterDistMeters[b] : 0.0f;
                    // [OSD_Widget] 已注释
                    offset += box.pointCount;
                    out.push_back(std::move(box));
                }
            }
            return !out.empty();
        });
}

void Widget::on_btn_lidar_osd_(int camNum)
{
    lidarOsdEnabled_[camNum] = !lidarOsdEnabled_[camNum];
    QPushButton* btn = (camNum == 0) ? ui->btnLidarOsd0 : ui->btnLidarOsd1;

    if (lidarOsdEnabled_[camNum]) {
        if (!fusionEnabled_ || !fusion_) {
            fprintf(stderr, "[SentinelQT] LiDAR OSD requires fusion to be enabled\n");
            lidarOsdEnabled_[camNum] = false;
            return;
        }
        setup_lidar_osd_provider_();
        streamer_->set_stream_lidar_osd_mode(camNum, StreamLidarOsdMode::WITH_LIDAR_OSD);
        btn->setText("点云投影开");
        btn->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; color: #000; "
            "background-color: #FF9800; border: 1px solid #F57C00; border-radius: 8px; }");
        fprintf(stderr, "[SentinelQT] cam %d LiDAR OSD enabled\n", camNum);
    } else {
        streamer_->set_stream_lidar_osd_mode(camNum, StreamLidarOsdMode::WITHOUT_LIDAR_OSD);
        btn->setText("点云投影关");
        btn->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; "
            "background-color: #6e7681; border: 1px solid #8b949e; border-radius: 8px; }");
        fprintf(stderr, "[SentinelQT] cam %d LiDAR OSD disabled\n", camNum);
    }
}

void Widget::on_btn_fusion_toggle_()
{
    if (!fusionEnabled_) {
        // ---- 启用融合 ----
        sync_ui_to_fusion_config_();
        save_fusion_config_();

        lidar_->load_config(lidarCfg_);
        if (!lidar_->start()) {
            ui->fusionStatusLabel->setText("雷达启动失败");
            return;
        }

        // 启动 NPU 推理（如果 OSD 已经创建则复用）
        {
            if (!yoloInfer_) {
                SentinelYoloInferConfig inferCfg;
                inferCfg.modelPath = config_.value("Fusion/modelPath",
                    "./models/yolov8n.rknn").toString().toStdString();
                inferCfg.boxThreshold = 0.25f;
                inferCfg.waitTimeoutMs = 200;
                yoloInfer_ = new SentinelYoloInfer(visioner_, inferCfg);
                for (int c = 0; c < 2; ++c) {
                    if (!yoloInfer_->create_infer_thread(c))
                        fprintf(stderr, "[SentinelQT] infer thread cam %d failed\n", c);
                }
                streamer_->set_osd_provider(
                    [this](int cam, std::vector<StreamOsdBBox>& out, int) {
                        YoloBBoxList boxes;
                        bool gotAny = false;
                        while (yoloInfer_->try_get_osd_result(cam, boxes, 0))
                            gotAny = true;
                        if (!gotAny) return false;
                        for (const auto& b : boxes) {
                            out.push_back({b.x1, b.y1, b.x2, b.y2,
                                           b.classId, b.confidence});
                        }
                        return true;
                    });
            }
            fusion_->set_detection_provider(
                [this](int camNum, std::vector<YoloBBox>& out, int timeoutMs) {
                    return yoloInfer_->try_get_fusion_result(camNum, out, timeoutMs);
                });
        }

        fusion_->configure_tracker(fusionTrackerCfg_);
        fusion_->enable_tracking(true);
        // 自动回溯暂关闭
        // fusion_->register_warning_callback(fusion_warning_callback_, nullptr);

        if (!fusion_->start(lidar_, fusionCamCfg_, fusionCamCount_)) {
            if (!osdEnabled_[0] && !osdEnabled_[1]) {
                yoloInfer_->stop_all();
                delete yoloInfer_;
                yoloInfer_ = nullptr;
            }
            lidar_->stop();
            ui->fusionStatusLabel->setText("融合启动失败");
            return;
        }

        fusionWorker_ = new FusionWorker(fusion_);
        fusionThread_ = new QThread(this);
        fusionWorker_->moveToThread(fusionThread_);
        connect(fusionWorker_, &FusionWorker::trackingUpdated,
                this, &Widget::on_tracking_updated_);
        connect(fusionThread_, &QThread::started,
                fusionWorker_, &FusionWorker::start);
        fusionThread_->start();

        fusionStatusTimer_->start(1000);
        fusionEnabled_ = true;

        setup_lidar_osd_provider_();

        ui->btnFusionToggle->setText("停止融合");
        ui->btnFusionToggle->setStyleSheet(FUSION_ON_STYLE);
        ui->fusionStatusLabel->setText("目标: 0 | 已确认: 0 | 告警: 0 | 融合: 运行中");

        fprintf(stderr, "[SentinelQT] Fusion enabled\n");
    } else {
        // ---- 禁用融合 ----
        if (fusionWorker_) {
            fusionWorker_->stop();
            if (fusionThread_ && fusionThread_->isRunning()) {
                fusionThread_->quit();
                fusionThread_->wait(3000);
            }
            delete fusionWorker_;
            fusionWorker_ = nullptr;
            delete fusionThread_;
            fusionThread_ = nullptr;
        }

        fusion_->stop();

        if (!osdEnabled_[0] && !osdEnabled_[1] && yoloInfer_) {
            yoloInfer_->stop_all();
            delete yoloInfer_;
            yoloInfer_ = nullptr;
        }

        lidar_->stop();
        fusionStatusTimer_->stop();
        fusion_->reset_tracking();

        topDownView_->set_targets({});
        topDownView_->update();
        topDownView_->update();

        fusionEnabled_ = false;
        lastTrackedTargets_.clear();

        ui->btnFusionToggle->setText("启用融合");
        ui->btnFusionToggle->setStyleSheet(FUSION_OFF_STYLE);
        ui->fusionStatusLabel->setText("目标: 0 | 已确认: 0 | 告警: 0 | 融合: 关闭");

        fprintf(stderr, "[SentinelQT] Fusion disabled\n");
    }
}

void Widget::on_btn_back_from_fusion_()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void Widget::on_tracking_updated_(const QVector<TrackedTarget>& targets)
{
    lastTrackedTargets_ = targets;
    topDownView_->set_targets(targets);
    topDownView_->update();

    // Web 跟踪数据推送 (5Hz，由 FusionWorker 频率决定)
    if (webServer_ && webServer_->is_running()) {
        nlohmann::json j;
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& t : targets) {
            nlohmann::json tj;
            tj["id"] = t.id;
            tj["state"] = (t.state == TrackState::FusionTracking) ? "FusionTracking" :
                          (t.state == TrackState::PureRadarTracking) ? "PureRadarTracking" :
                          (t.state == TrackState::Tentative) ? "Tentative" : "Lost";
            tj["posX"] = t.posX;
            tj["posY"] = t.posY;
            tj["velX"] = t.velX;
            tj["velY"] = t.velY;
            tj["distanceMeters"] = t.distanceMeters;
            tj["confidence"] = t.confidence;
            tj["pointCount"] = t.pointCount;
            tj["classId"] = t.classId;
            tj["warningActive"] = t.warningActive;
            tj["age"] = t.age;
            arr.push_back(tj);
        }
        j["targets"] = arr;

        // 聚类可视化数据
        if (fusion_ && fusion_->is_running()) {
            ClusterVisData cvData[32];
            uint32_t cvCount = 0;
            if (fusion_->copy_cluster_vis(cvData, 32, &cvCount) && cvCount > 0) {
                nlohmann::json cArr = nlohmann::json::array();
                for (uint32_t i = 0; i < cvCount; ++i) {
                    nlohmann::json cj;
                    cj["cx"] = cvData[i].cx;
                    cj["cy"] = cvData[i].cy;
                    cj["radius"] = cvData[i].radius;
                    cj["pointCount"] = cvData[i].pointCount;
                    cj["isOrphan"] = cvData[i].isOrphan;
                    cj["bboxIdx"] = cvData[i].bboxIdx;
                    cArr.push_back(cj);
                }
                j["clusters"] = cArr;
            }
        }

        j["ts"] = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        webServer_->push_tracking(j.dump());
    }
}

void Widget::on_fusion_param_changed_()
{
    sync_ui_to_fusion_config_();
    save_fusion_config_();

    if (fusionEnabled_) {
        fusion_->configure_tracker(fusionTrackerCfg_);
        fusion_->enable_tracking(true);
        fusion_->update_camera_intrinsics(
            0, fusionCamCfg_[0].fx, fusionCamCfg_[0].fy,
            fusionCamCfg_[0].cx, fusionCamCfg_[0].cy,
            fusionCamCfg_[0].imgWidth, fusionCamCfg_[0].imgHeight);
        fusion_->update_camera_intrinsics(
            1, fusionCamCfg_[1].fx, fusionCamCfg_[1].fy,
            fusionCamCfg_[1].cx, fusionCamCfg_[1].cy,
            fusionCamCfg_[1].imgWidth, fusionCamCfg_[1].imgHeight);
    }
}

void Widget::on_fusion_status_update_()
{
    uint32_t total = static_cast<uint32_t>(lastTrackedTargets_.size());
    uint32_t fusionCnt = 0, pureRadarCnt = 0, warnings = 0;
    for (const auto& t : lastTrackedTargets_) {
        if (t.state == TrackState::FusionTracking) ++fusionCnt;
        else if (t.state == TrackState::PureRadarTracking) ++pureRadarCnt;
        if (t.warningActive) ++warnings;
    }

    ui->fusionStatusLabel->setText(
        QString("目标:%1 | 融合:%2 | 纯雷达:%3 | 告警:%4 | %5")
            .arg(total)
            .arg(fusionCnt)
            .arg(pureRadarCnt)
            .arg(warnings)
            .arg(fusionEnabled_ ? "运行中" : "关闭"));

    QStringList distItems;
    int shown = 0;
    for (const auto& t : lastTrackedTargets_) {
        if ((t.state == TrackState::FusionTracking
             || t.state == TrackState::PureRadarTracking) && shown < 3) {
            distItems.append(QString("T%1:%2m").arg(t.id).arg(t.distanceMeters, 0, 'f', 1));
            ++shown;
        }
    }
    ui->fusionDistLabel->setText(distItems.join("  "));
}

// ======================================================================
//  Web 远程控制 API
// ======================================================================

std::string Widget::handle_web_command(const std::string& method,
                                        const std::string& path,
                                        const std::string& body)
{
    fprintf(stderr, "[WebCmd] %s %s\n", method.c_str(), path.c_str());

    // ---- 状态查询 (GET) ----
    if (method == "GET") {
        if (path == "/api/v1/status")        return get_status_json_();
        if (path == "/api/v1/status/hw")     return get_hw_json_();
        if (path == "/api/v1/videos")        return get_videos_json_();
        if (path == "/api/v1/fusion/config") return get_fusion_config_json_();
        if (path == "/api/v1/eis/config")    return get_eis_config_json_();
        if (path == "/api/v1/eis/visible")   return showEisControl_ ? R"({"visible":true})" : R"({"visible":false})";
        if (path == "/api/v1/backtrack/files") return get_backtrack_files_json_();
        return R"({"ok":false,"error":"unknown GET path"})";
    }

    // ---- 相机控制 (POST) ----
    if (method == "POST") {
        if (path == "/api/v1/cam/0/preview/start")  return web_start_preview_(0);
        if (path == "/api/v1/cam/0/preview/stop")   return web_stop_preview_(0);
        if (path == "/api/v1/cam/0/stream/start")   return web_start_stream_(0);
        if (path == "/api/v1/cam/0/stream/stop")    return web_stop_stream_(0);
        if (path == "/api/v1/cam/0/record/start")   return web_start_record_(0);
        if (path == "/api/v1/cam/0/record/stop")    return web_stop_record_(0);
        if (path == "/api/v1/cam/0/pause")          return web_pause_(0);
        if (path == "/api/v1/cam/0/resume")         return web_resume_(0);
        if (path == "/api/v1/cam/0/osd/start")      return web_osd_start_(0);
        if (path == "/api/v1/cam/0/osd/stop")       return web_osd_stop_(0);
        if (path == "/api/v1/cam/1/preview/start")  return web_start_preview_(1);
        if (path == "/api/v1/cam/1/preview/stop")   return web_stop_preview_(1);
        if (path == "/api/v1/cam/1/stream/start")   return web_start_stream_(1);
        if (path == "/api/v1/cam/1/stream/stop")    return web_stop_stream_(1);
        if (path == "/api/v1/cam/1/record/start")   return web_start_record_(1);
        if (path == "/api/v1/cam/1/record/stop")    return web_stop_record_(1);
        if (path == "/api/v1/cam/1/pause")          return web_pause_(1);
        if (path == "/api/v1/cam/1/resume")         return web_resume_(1);
        if (path == "/api/v1/cam/1/osd/start")      return web_osd_start_(1);
        if (path == "/api/v1/cam/1/osd/stop")       return web_osd_stop_(1);
        if (path == "/api/v1/cam/0/eis/start")      return web_eis_start_(0);
        if (path == "/api/v1/cam/0/eis/stop")       return web_eis_stop_(0);
        if (path == "/api/v1/cam/1/eis/start")      return web_eis_start_(1);
        if (path == "/api/v1/cam/1/eis/stop")       return web_eis_stop_(1);
        if (path == "/api/v1/cam/0/lidar-osd/start") return web_lidar_osd_start_(0);
        if (path == "/api/v1/cam/0/lidar-osd/stop")  return web_lidar_osd_stop_(0);
        if (path == "/api/v1/cam/1/lidar-osd/start") return web_lidar_osd_start_(1);
        if (path == "/api/v1/cam/1/lidar-osd/stop")  return web_lidar_osd_stop_(1);
        if (path == "/api/v1/system/start")         return web_system_start_();
        if (path == "/api/v1/system/stop")          return web_system_stop_();
        if (path == "/api/v1/lidar/start")          return web_lidar_start_();
        if (path == "/api/v1/lidar/stop")           return web_lidar_stop_();
        if (path == "/api/v1/fusion/start")         return web_fusion_start_();
        if (path == "/api/v1/fusion/stop")          return web_fusion_stop_();
        if (path == "/api/v1/fusion/config")        return web_fusion_config_(body);
        if (path == "/api/v1/eis/config")           return web_eis_config_(body);
        if (path == "/api/v1/fusion/camera/0/intrinsics") return web_fusion_intrinsics_(0, body);
        if (path == "/api/v1/fusion/camera/1/intrinsics") return web_fusion_intrinsics_(1, body);
        if (path == "/api/v1/backtrack/query")  return web_backtrack_query_(body);
        return R"({"ok":false,"error":"unknown POST path"})";
    }

    // ---- PUT ----
    if (method == "PUT") {
        if (path == "/api/v1/cam/0/record-resolution") return web_set_record_resolution_(0, body);
        if (path == "/api/v1/cam/1/record-resolution") return web_set_record_resolution_(1, body);
        return R"({"ok":false,"error":"unknown PUT path"})";
    }

    // ---- DELETE ----
    if (method == "DELETE") {
        if (path == "/api/v1/videos") return web_delete_video_(body);
        if (path == "/api/v1/backtrack/files") return web_delete_backtrack_(body);
        return R"({"ok":false,"error":"unknown DELETE path"})";
    }

    return R"({"ok":false,"error":"unknown method"})";
}

// ---- Preview ----

std::string Widget::web_start_preview_(int camNum)
{
    if (previewActive_[camNum]) return R"({"ok":true})";
    start_preview_(camNum);
    QPushButton* btn = cam_btn(ui->btnToggle0, ui->btnToggle1, camNum);
    btn->setText("关闭预览");
    btn->setStyleSheet(TOGGLE_ON_STYLE);
    update_camera_button_states_(camNum);
    return R"({"ok":true})";
}

std::string Widget::web_stop_preview_(int camNum)
{
    if (!previewActive_[camNum]) return R"({"ok":true})";
    stop_preview_(camNum);
    QPushButton* btn = cam_btn(ui->btnToggle0, ui->btnToggle1, camNum);
    btn->setText("开启预览");
    btn->setStyleSheet(TOGGLE_OFF_STYLE);
    update_camera_button_states_(camNum);
    return R"({"ok":true})";
}

// ---- Stream ----

std::string Widget::web_start_stream_(int camNum)
{
    if (streamer_->is_streaming(camNum)) return R"({"ok":true})";

    // 如果相机已暂停，先恢复
    if (cameraPaused_[camNum]) {
        visioner_->camera_pause(camNum, false);
        cameraPaused_[camNum] = false;
        if (!previewActive_[camNum]) {
            start_preview_(camNum);
        }
    }

    QByteArray url = rtspUrl_[camNum].toUtf8();
    if (streamer_->start_stream(camNum, url.constData())) {
        update_camera_button_states_(camNum);
        return R"({"ok":true})";
    }
    return R"({"ok":false,"error":"stream start failed"})";
}

std::string Widget::web_stop_stream_(int camNum)
{
    if (!streamer_->is_streaming(camNum)) return R"({"ok":true})";
    streamer_->stop_stream(camNum);
    update_camera_button_states_(camNum);
    return R"({"ok":true})";
}

// ---- Record ----

std::string Widget::web_start_record_(int camNum)
{
    if (streamer_->is_recording(camNum)) return R"({"ok":true})";

    // 如果相机已暂停，先恢复
    if (cameraPaused_[camNum]) {
        visioner_->camera_pause(camNum, false);
        cameraPaused_[camNum] = false;
        if (!previewActive_[camNum]) {
            start_preview_(camNum);
        }
    }

    int resVal = recordResolution_[camNum];
    RecordResolution recordRes = (resVal == 720)
        ? RecordResolution::RES_720P : RecordResolution::RES_1080P;

    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    currentRecordPath_[camNum] = recordDir_ + QString("/cam%1_record_%2.mp4")
        .arg(camNum).arg(timestamp);

    QByteArray path = currentRecordPath_[camNum].toUtf8();
    if (streamer_->start_record(camNum, path.constData(), recordRes)) {
        recordStartTime_[camNum] = QDateTime::currentDateTime();
        recordTimer_[camNum]->start(1000);
        update_record_info_(camNum);
        update_camera_button_states_(camNum);
        return R"({"ok":true})";
    }
    return R"({"ok":false,"error":"record start failed"})";
}

std::string Widget::web_stop_record_(int camNum)
{
    if (!streamer_->is_recording(camNum)) return R"({"ok":true})";
    streamer_->stop_record(camNum);
    recordTimer_[camNum]->stop();
    ui->recordInfoLabel->clear();
    update_camera_button_states_(camNum);
    return R"({"ok":true})";
}

// ---- Pause/Resume ----

std::string Widget::web_pause_(int camNum)
{
    if (cameraPaused_[camNum]) return R"({"ok":true})";

    // 先停止推流、录像和预览，避免暂停时系统状态混乱导致卡死
    if (streamer_->is_streaming(camNum)) {
        streamer_->stop_stream(camNum);
    }
    if (streamer_->is_recording(camNum)) {
        streamer_->stop_record(camNum);
        recordTimer_[camNum]->stop();
        ui->recordInfoLabel->clear();
    }
    if (previewActive_[camNum]) {
        stop_preview_(camNum);
    }

    cameraPaused_[camNum] = true;
    visioner_->camera_pause(camNum, true);
    update_camera_button_states_(camNum);
    refresh_status_label_();
    return R"({"ok":true})";
}

std::string Widget::web_resume_(int camNum)
{
    if (!cameraPaused_[camNum]) return R"({"ok":true})";

    visioner_->camera_pause(camNum, false);

    // 恢复预览（暂停时已被 web_pause_ 停止）
    if (!previewActive_[camNum]) {
        start_preview_(camNum);
        QPushButton* toggleBtn = cam_btn(ui->btnToggle0, ui->btnToggle1, camNum);
        toggleBtn->setText("关闭预览");
        toggleBtn->setStyleSheet(TOGGLE_ON_STYLE);
    }

    cameraPaused_[camNum] = false;
    update_camera_button_states_(camNum);
    refresh_status_label_();
    return R"({"ok":true})";
}

// ---- OSD ----

std::string Widget::web_osd_start_(int camNum)
{
    if (osdEnabled_[camNum]) return R"({"ok":true})";

    if (!yoloInfer_) {
        SentinelYoloInferConfig inferCfg;
        inferCfg.modelPath = config_.value("Fusion/modelPath",
            "./models/yolov8n.rknn").toString().toStdString();
        inferCfg.boxThreshold = 0.25f;
        inferCfg.waitTimeoutMs = 200;
        yoloInfer_ = new SentinelYoloInfer(visioner_, inferCfg);
        for (int c = 0; c < 2; ++c) {
            if (!yoloInfer_->create_infer_thread(c))
                fprintf(stderr, "[SentinelQT] OSD infer thread cam %d failed\n", c);
        }
        streamer_->set_osd_provider(
            [this](int cam, std::vector<StreamOsdBBox>& out, int) {
                YoloBBoxList boxes;
                bool gotAny = false;
                while (yoloInfer_->try_get_osd_result(cam, boxes, 0))
                    gotAny = true;
                if (!gotAny) return false;
                for (const auto& b : boxes) {
                    out.push_back({b.x1, b.y1, b.x2, b.y2,
                                   b.classId, b.confidence});
                }
                return true;
            });
        if (fusionEnabled_) {
            fusion_->set_detection_provider(
                [this](int cam, std::vector<YoloBBox>& out, int to) {
                    return yoloInfer_->try_get_fusion_result(cam, out, to);
                });
        }
    }
    streamer_->set_stream_osd_mode(camNum, StreamOsdMode::WITH_OSD);
    osdEnabled_[camNum] = true;

    QPushButton* btn = (camNum == 0) ? ui->btnOsd0 : ui->btnOsd1;
    btn->setText("框去除");
    btn->setStyleSheet(
        "QPushButton { font-size: 12px; font-weight: 600; color: #000; "
        "background-color: #4CAF50; border: 1px solid #388E3C; border-radius: 8px; }");
    fprintf(stderr, "[SentinelQT] cam %d OSD enabled via web\n", camNum);
    return R"({"ok":true})";
}

std::string Widget::web_osd_stop_(int camNum)
{
    if (!osdEnabled_[camNum]) return R"({"ok":true})";

    streamer_->set_stream_osd_mode(camNum, StreamOsdMode::WITHOUT_OSD);
    osdEnabled_[camNum] = false;

    QPushButton* btn = (camNum == 0) ? ui->btnOsd0 : ui->btnOsd1;
    btn->setText("框叠加");
    btn->setStyleSheet(
        "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; "
        "background-color: #6e7681; border: 1px solid #8b949e; border-radius: 8px; }");
    fprintf(stderr, "[SentinelQT] cam %d OSD disabled via web\n", camNum);

    if (!osdEnabled_[0] && !osdEnabled_[1] && !fusionEnabled_) {
        yoloInfer_->stop_all();
        delete yoloInfer_;
        yoloInfer_ = nullptr;
    }
    return R"({"ok":true})";
}

// ============================================================================
// EIS 防抖：视觉为主 + IMU 辅助
// ============================================================================

void Widget::init_eis_()
{
    if (eisReader_) return;

    std::string devPath = config_.value("EIS/device",
        "/dev/icm45686").toString().toStdString();

    float sampleHz = config_.value("EIS/sampleHz", 100.0f).toFloat();

    eisReader_ = new Icm45686Reader(512);
    if (!eisReader_->openDevice(devPath)) {
        // 视觉 EIS 可以在没有 IMU 的情况下独立运行。
        // 这里不再让 IMU 打开失败阻断整个防抖功能，而是降级为纯视觉 EIS。
        fprintf(stderr, "[SentinelQT] Visual EIS: IMU device %s open failed, run visual-only EIS.\n",
                devPath.c_str());
        delete eisReader_;
        eisReader_ = nullptr;
        visioner_->set_imu_assist_callback(nullptr);
        set_status_("EIS IMU不可用，已降级为纯视觉防抖", "#d29922");
        return;
    }

    uint8_t gyroRange = static_cast<uint8_t>(config_.value("EIS/gyroRange", 0).toInt());
    uint8_t accelRange = static_cast<uint8_t>(config_.value("EIS/accelRange", 0).toInt());
    eisReader_->setGyroRange(gyroRange);
    eisReader_->setAccelRange(accelRange);

    if (!eisReader_->start(sampleHz)) {
        fprintf(stderr, "[SentinelQT] Visual EIS: IMU reader thread start failed\n");
        eisReader_->closeDevice();
        delete eisReader_;
        eisReader_ = nullptr;
        visioner_->set_imu_assist_callback(nullptr);
        set_status_("EIS IMU读取线程启动失败，已降级为纯视觉防抖", "#d29922");
        return;
    }

    // 给 SentinelVisioner 注册 IMU 辅助状态回调。
    // SentinelVisioner 内部的视觉 EIS 会在每帧处理时取 gyroRms / vibrationLevel，
    // 用于动态选择轨迹平滑 alpha；IMU 不再直接给 offsetX/offsetY。
    visioner_->set_imu_assist_callback(
        [this](uint64_t timestampUs, int camNum, VisionImuAssistState& state) -> bool {
            return imu_assist_callback_(timestampUs, camNum, state);
        });

    int streamerMargin = config_.value("EIS/streamerMargin", 32).toInt();
    bool eisDebug = config_.value("EIS/eisRecordDebug", false).toBool();
    for (int c = 0; c < 2; ++c) {
        streamer_->set_eis_params(c, streamerMargin);
        streamer_->set_eis_record_debug(c, eisDebug);
    }

    fprintf(stderr, "[SentinelQT] Visual EIS IMU assist initialized: %s @ %.0f Hz\n",
            devPath.c_str(), static_cast<double>(sampleHz));
}

void Widget::deinit_eis_()
{
    // 关闭 IMU 辅助回调，同时关闭每路视觉 EIS。
    visioner_->set_imu_assist_callback(nullptr);
    for (int i = 0; i < 2; ++i) {
        if (visioner_) {
            visioner_->enable_visual_eis(i, false);
        }
    }

    if (eisReader_) {
        eisReader_->stop();
        eisReader_->closeDevice();
        delete eisReader_;
        eisReader_ = nullptr;
    }

    for (int i = 0; i < 2; ++i) {
        eisEnabled_[i] = false;
        QPushButton* btn = (i == 0) ? ui->btnEis0 : ui->btnEis1;
        if (btn) {
            btn->setText(QString::fromUtf8("防抖关"));
            btn->setStyleSheet(
                "QPushButton { font-size: 12px; font-weight: 600; "
                "color: #e6edf3; background-color: #6e7681; "
                "border: 1px solid #8b949e; border-radius: 8px; }");
        }
    }

    fprintf(stderr, "[SentinelQT] Visual EIS deinitialized\n");
}

bool Widget::imu_assist_callback_(uint64_t /*timestampUs*/, int /*camNum*/,
                                  VisionImuAssistState& state)
{
    if (!eisReader_) {
        return false;
    }

    ImuAssistState imu;
    if (!eisReader_->getAssistState(imu, imuAssistWindowMs_)) {
        return false;
    }

    state.timestampNs = imu.timestampNs;
    state.accelX = imu.accelX;
    state.accelY = imu.accelY;
    state.accelZ = imu.accelZ;
    state.gyroX = imu.gyroX;
    state.gyroY = imu.gyroY;
    state.gyroZ = imu.gyroZ;
    state.accelNorm = imu.accelNorm;
    state.gyroNorm = imu.gyroNorm;
    state.gyroRms = imu.gyroRms;
    state.vibrationLevel = imu.vibrationLevel;

    return true;
}

void Widget::on_btn_eis_(int camNum)
{
    QPushButton* btn = (camNum == 0) ? ui->btnEis0 : ui->btnEis1;

    if (!eisEnabled_[camNum]) {
        // 先尝试启动 IMU 辅助；即便 IMU 打不开，SentinelVisioner 仍可执行纯视觉 EIS。
        if (!eisReader_) {
            init_eis_();
        }

        visioner_->set_visual_eis_config(camNum, visualEisCfg_[camNum]);
        if (!visioner_->enable_visual_eis(camNum, true)) {
            set_status_(QString("相机%1 视觉EIS启动失败").arg(camNum + 1), "#f85149");
            return;
        }

        eisEnabled_[camNum] = true;
        btn->setText(QString::fromUtf8("防抖开"));
        btn->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; color: #000; "
            "background-color: #4CAF50; border: 1px solid #388E3C; border-radius: 8px; }");
        set_status_(QString("相机%1 视觉EIS已启用").arg(camNum + 1), "#3fb950");
    } else {
        visioner_->enable_visual_eis(camNum, false);
        eisEnabled_[camNum] = false;
        btn->setText(QString::fromUtf8("防抖关"));
        btn->setStyleSheet(
            "QPushButton { font-size: 12px; font-weight: 600; "
            "color: #e6edf3; background-color: #6e7681; "
            "border: 1px solid #8b949e; border-radius: 8px; }");
        set_status_(QString("相机%1 视觉EIS已禁用").arg(camNum + 1), "#ffffff");

        if (!eisEnabled_[0] && !eisEnabled_[1]) {
            deinit_eis_();
        }
    }

    fprintf(stderr, "[SentinelQT] cam %d Visual EIS %s\n", camNum,
            eisEnabled_[camNum] ? "enabled" : "disabled");
}

std::string Widget::web_eis_start_(int camNum)
{
    if (eisEnabled_[camNum]) return R"({"ok":true})";
    if (!eisReader_) {
        init_eis_();
    }
    visioner_->set_visual_eis_config(camNum, visualEisCfg_[camNum]);
    if (!visioner_->enable_visual_eis(camNum, true)) {
        return R"({"ok":false,"error":"Visual EIS enable failed"})";
    }
    eisEnabled_[camNum] = true;
    update_camera_button_states_(camNum);
    return R"({"ok":true})";
}

std::string Widget::web_eis_stop_(int camNum)
{
    if (!eisEnabled_[camNum]) return R"({"ok":true})";
    visioner_->enable_visual_eis(camNum, false);
    eisEnabled_[camNum] = false;
    update_camera_button_states_(camNum);
    if (!eisEnabled_[0] && !eisEnabled_[1]) {
        deinit_eis_();
    }
    return R"({"ok":true})";
}

// ---- LiDAR OSD ----

std::string Widget::web_lidar_osd_start_(int camNum)
{
    if (lidarOsdEnabled_[camNum]) return R"({"ok":true})";

    if (!fusionEnabled_ || !fusion_) {
        return R"({"ok":false,"error":"fusion not enabled"})";
    }
    setup_lidar_osd_provider_();
    streamer_->set_stream_lidar_osd_mode(camNum, StreamLidarOsdMode::WITH_LIDAR_OSD);
    lidarOsdEnabled_[camNum] = true;

    QPushButton* btn = (camNum == 0) ? ui->btnLidarOsd0 : ui->btnLidarOsd1;
    btn->setText("点云投影开");
    btn->setStyleSheet(
        "QPushButton { font-size: 12px; font-weight: 600; color: #000; "
        "background-color: #FF9800; border: 1px solid #F57C00; border-radius: 8px; }");
    fprintf(stderr, "[SentinelQT] cam %d LiDAR OSD enabled via web\n", camNum);
    return R"({"ok":true})";
}

std::string Widget::web_lidar_osd_stop_(int camNum)
{
    if (!lidarOsdEnabled_[camNum]) return R"({"ok":true})";

    streamer_->set_stream_lidar_osd_mode(camNum, StreamLidarOsdMode::WITHOUT_LIDAR_OSD);
    lidarOsdEnabled_[camNum] = false;

    QPushButton* btn = (camNum == 0) ? ui->btnLidarOsd0 : ui->btnLidarOsd1;
    btn->setText("点云投影关");
    btn->setStyleSheet(
        "QPushButton { font-size: 12px; font-weight: 600; color: #e6edf3; "
        "background-color: #6e7681; border: 1px solid #8b949e; border-radius: 8px; }");
    fprintf(stderr, "[SentinelQT] cam %d LiDAR OSD disabled via web\n", camNum);
    return R"({"ok":true})";
}

// ---- System ----

std::string Widget::web_system_start_()
{
    bool anyActive = previewActive_[0] || previewActive_[1];
    if (anyActive) return R"({"ok":true})";
    on_btn_system_();
    return R"({"ok":true})";
}

std::string Widget::web_system_stop_()
{
    bool allStopped = !previewActive_[0] && !previewActive_[1];
    if (allStopped) return R"({"ok":true})";
    on_btn_system_();
    return R"({"ok":true})";
}

// ---- Record resolution ----

std::string Widget::web_set_record_resolution_(int camNum, const std::string& body)
{
    try {
        auto j = nlohmann::json::parse(body);
        std::string res = j.value("resolution", "1080p");
        int val = (res == "720p") ? 720 : 1080;
        recordResolution_[camNum] = val;
        (camNum == 0 ? ui->resCombo : ui->resCombo1)->setCurrentIndex(val == 720 ? 1 : 0);
        config_.setValue(QString("Camera%1/recordResolution").arg(camNum), val);
        return R"({"ok":true})";
    } catch (...) {
        return R"({"ok":false,"error":"invalid JSON"})";
    }
}

// ---- Videos ----

std::string Widget::web_delete_video_(const std::string& body)
{
    try {
        auto j = nlohmann::json::parse(body);
        std::string filePath = j.value("path", "");
        if (filePath.empty())
            return R"({"ok":false,"error":"missing path"})";
        if (QFile::remove(QString::fromStdString(filePath))) {
            return R"({"ok":true})";
        }
        return R"({"ok":false,"error":"delete failed"})";
    } catch (...) {
        return R"({"ok":false,"error":"invalid JSON"})";
    }
}

std::string Widget::web_delete_backtrack_(const std::string& body)
{
    try {
        auto j = nlohmann::json::parse(body);
        std::string fileName = j.value("name", "");
        if (fileName.empty())
            return R"({"ok":false,"error":"missing name"})";
        QDir dir(backtrackDir_);
        QString filePath = dir.absoluteFilePath(QString::fromStdString(fileName));
        if (QFile::remove(filePath))
            return R"({"ok":true})";
        return R"({"ok":false,"error":"delete failed"})";
    } catch (...) {
        return R"({"ok":false,"error":"invalid JSON"})";
    }
}

// ---- LiDAR ----

std::string Widget::web_lidar_start_()
{
    if (!lidar_) return R"({"ok":false,"error":"no lidar"})";
    if (!lidar_->load_config(lidarCfg_)) return R"({"ok":false,"error":"lidar config failed"})";
    if (!lidar_->start()) return R"({"ok":false,"error":"lidar start failed"})";
    return R"({"ok":true})";
}

std::string Widget::web_lidar_stop_()
{
    if (lidar_) lidar_->stop();
    return R"({"ok":true})";
}

// ---- Fusion ----

std::string Widget::web_fusion_start_()
{
    if (fusionEnabled_) return R"({"ok":true})";

    if (!lidar_) return R"({"ok":false,"error":"LiDAR 未初始化"})";
    if (!fusion_) return R"({"ok":false,"error":"Fusion 未初始化"})";

    sync_ui_to_fusion_config_();
    save_fusion_config_();

    // 如果 LiDAR 还没启动则启动（可能已由 /lidar/start 提前启动）
    if (!lidar_->is_running()) {
        if (!lidar_->load_config(lidarCfg_))
            return R"({"ok":false,"error":"LiDAR 配置加载失败"})";
        if (!lidar_->start())
            return R"({"ok":false,"error":"LiDAR 启动失败（检查串口设备）"})";
    }

    // 启动 NPU 推理
    if (!yoloInfer_) {
        SentinelYoloInferConfig inferCfg;
        inferCfg.modelPath = config_.value("Fusion/modelPath",
            "./models/yolov8n.rknn").toString().toStdString();
        inferCfg.boxThreshold = 0.25f;
        inferCfg.waitTimeoutMs = 200;
        yoloInfer_ = new SentinelYoloInfer(visioner_, inferCfg);
        for (int c = 0; c < 2; ++c) {
            if (!yoloInfer_->create_infer_thread(c))
                fprintf(stderr, "[SentinelQT] infer thread cam %d failed\n", c);
        }
        streamer_->set_osd_provider(
            [this](int cam, std::vector<StreamOsdBBox>& out, int) {
                YoloBBoxList boxes;
                bool gotAny = false;
                while (yoloInfer_->try_get_osd_result(cam, boxes, 0))
                    gotAny = true;
                if (!gotAny) return false;
                for (const auto& b : boxes) {
                    out.push_back({b.x1, b.y1, b.x2, b.y2,
                                   b.classId, b.confidence});
                }
                return true;
            });
    }
    fusion_->set_detection_provider(
        [this](int camNum, std::vector<YoloBBox>& out, int timeoutMs) {
            return yoloInfer_->try_get_fusion_result(camNum, out, timeoutMs);
        });

    fusion_->configure_tracker(fusionTrackerCfg_);
    fusion_->enable_tracking(true);
    // 自动回溯暂关闭
    // fusion_->register_warning_callback(fusion_warning_callback_, nullptr);

    if (!fusion_->start(lidar_, fusionCamCfg_, fusionCamCount_)) {
        if (!osdEnabled_[0] && !osdEnabled_[1] && yoloInfer_) {
            yoloInfer_->stop_all();
            delete yoloInfer_;
            yoloInfer_ = nullptr;
        }
        lidar_->stop();
        return R"({"ok":false,"error":"融合启动失败（检查相机内参配置）"})";
    }

    fusionWorker_ = new FusionWorker(fusion_);
    fusionThread_ = new QThread(this);
    fusionWorker_->moveToThread(fusionThread_);
    connect(fusionWorker_, &FusionWorker::trackingUpdated,
            this, &Widget::on_tracking_updated_);
    connect(fusionThread_, &QThread::started,
            fusionWorker_, &FusionWorker::start);
    fusionThread_->start();

    fusionStatusTimer_->start(1000);
    fusionEnabled_ = true;

    setup_lidar_osd_provider_();

    ui->btnFusionToggle->setText("停止融合");
    ui->btnFusionToggle->setStyleSheet(FUSION_ON_STYLE);
    ui->fusionStatusLabel->setText("目标: 0 | 已确认: 0 | 告警: 0 | 融合: 运行中");

    fprintf(stderr, "[SentinelQT] Fusion enabled via web\n");
    return R"({"ok":true})";
}

std::string Widget::web_fusion_stop_()
{
    if (!fusionEnabled_) return R"({"ok":true})";
    on_btn_fusion_toggle_();
    return fusionEnabled_ ? R"({"ok":false,"error":"fusion stop failed"})" : R"({"ok":true})";
}

std::string Widget::web_fusion_config_(const std::string& body)
{
    try {
        auto j = nlohmann::json::parse(body);
        if (j.contains("dbscanEpsMeters"))
            fusionTrackerCfg_.dbscanEpsMeters = j["dbscanEpsMeters"];
        if (j.contains("dbscanMinPoints"))
            fusionTrackerCfg_.dbscanMinPoints = j["dbscanMinPoints"];
        if (j.contains("maxClusterDistanceMeters"))
            fusionTrackerCfg_.maxClusterDistanceMeters = j["maxClusterDistanceMeters"];
        if (j.contains("clusterPersistenceFrames"))
            fusionTrackerCfg_.clusterPersistenceFrames = j["clusterPersistenceFrames"];
        if (j.contains("bboxClaimMaxPixelDist"))
            fusionTrackerCfg_.bboxClaimMaxPixelDist = j["bboxClaimMaxPixelDist"];
        if (j.contains("minBboxClaimPoints"))
            fusionTrackerCfg_.minBboxClaimPoints = j["minBboxClaimPoints"];
        if (j.contains("alpha"))
            fusionTrackerCfg_.alpha = j["alpha"];
        if (j.contains("beta"))
            fusionTrackerCfg_.beta = j["beta"];
        if (j.contains("bboxAssocMaxDistMeters"))
            fusionTrackerCfg_.bboxAssocMaxDistMeters = j["bboxAssocMaxDistMeters"];
        if (j.contains("orphanAssocMaxDistMeters"))
            fusionTrackerCfg_.orphanAssocMaxDistMeters = j["orphanAssocMaxDistMeters"];
        if (j.contains("minHitsToConfirm"))
            fusionTrackerCfg_.minHitsToConfirm = j["minHitsToConfirm"];
        if (j.contains("maxLostFrames"))
            fusionTrackerCfg_.maxLostFrames = j["maxLostFrames"];
        if (j.contains("maxTracks"))
            fusionTrackerCfg_.maxTracks = j["maxTracks"];
        if (j.contains("warningEnterDistMeters"))
            fusionTrackerCfg_.warningEnterDistMeters = j["warningEnterDistMeters"];
        if (j.contains("warningExitDistMeters"))
            fusionTrackerCfg_.warningExitDistMeters = j["warningExitDistMeters"];
        if (j.contains("clusterVisOpacity"))
            fusionTrackerCfg_.clusterVisOpacity = j["clusterVisOpacity"];
        if (j.contains("radarRangeMeters"))
            fusionTrackerCfg_.radarRangeMeters = j["radarRangeMeters"];

        if (fusion_) fusion_->configure_tracker(fusionTrackerCfg_);
        save_fusion_config_();
        sync_fusion_config_to_ui_();
        return R"({"ok":true})";
    } catch (...) {
        return R"({"ok":false,"error":"invalid JSON"})";
    }
}

std::string Widget::web_fusion_intrinsics_(int camNum, const std::string& body)
{
    try {
        auto j = nlohmann::json::parse(body);
        fusionCamCfg_[camNum].fx = j.value("fx", fusionCamCfg_[camNum].fx);
        fusionCamCfg_[camNum].fy = j.value("fy", fusionCamCfg_[camNum].fy);
        fusionCamCfg_[camNum].cx = j.value("cx", fusionCamCfg_[camNum].cx);
        fusionCamCfg_[camNum].cy = j.value("cy", fusionCamCfg_[camNum].cy);

        if (fusion_) {
            fusion_->update_camera_intrinsics(camNum,
                fusionCamCfg_[camNum].fx, fusionCamCfg_[camNum].fy,
                fusionCamCfg_[camNum].cx, fusionCamCfg_[camNum].cy,
                fusionCamCfg_[camNum].imgWidth, fusionCamCfg_[camNum].imgHeight);
        }
        save_fusion_config_();
        return R"({"ok":true})";
    } catch (...) {
        return R"({"ok":false,"error":"invalid JSON"})";
    }
}

// ---- Status JSON builders ----

std::string Widget::get_status_json_() const
{
    nlohmann::json j;

    for (int i = 0; i < 2; ++i) {
        nlohmann::json cam;
        cam["previewActive"] = previewActive_[i];
        cam["paused"]        = cameraPaused_[i];
        cam["osdEnabled"]    = osdEnabled_[i];
        cam["eisEnabled"]    = eisEnabled_[i];
        cam["lidarOsdEnabled"] = lidarOsdEnabled_[i];
        cam["streaming"]     = streamer_ ? streamer_->is_streaming(i) : false;
        cam["recording"]     = streamer_ ? streamer_->is_recording(i) : false;
        cam["width"]         = camWidth_[i];
        cam["height"]        = camHeight_[i];
        cam["device"]        = deviceName_[i].toStdString();
        cam["streamUrl"]     = rtspUrl_[i].toStdString();
        cam["recordResolution"] = recordResolution_[i];

        if (streamer_ && streamer_->is_recording(i)) {
            qint64 elapsed = recordStartTime_[i].secsTo(QDateTime::currentDateTime());
            cam["recordingElapsedSec"] = elapsed;
            cam["recordingPath"] = currentRecordPath_[i].toStdString();
        } else {
            cam["recordingElapsedSec"] = 0;
            cam["recordingPath"] = "";
        }

        // FPS
        cam["fps"] = lastFps_[i];

        j[QString("cam%1").arg(i).toStdString()] = cam;
    }

    // HW stats
    j["hw"] = nlohmann::json::parse(get_hw_json_());

    // Fusion
    j["fusionEnabled"] = fusionEnabled_;
    if (fusionEnabled_) {
        nlohmann::json targets = nlohmann::json::array();
        for (const auto& t : lastTrackedTargets_) {
            nlohmann::json tj;
            tj["id"] = t.id;
            tj["state"] = (t.state == TrackState::FusionTracking) ? "FusionTracking" :
                          (t.state == TrackState::PureRadarTracking) ? "PureRadarTracking" :
                          (t.state == TrackState::Tentative) ? "Tentative" : "Lost";
            tj["posX"] = t.posX;
            tj["posY"] = t.posY;
            tj["velX"] = t.velX;
            tj["velY"] = t.velY;
            tj["distanceMeters"] = t.distanceMeters;
            tj["confidence"] = t.confidence;
            tj["pointCount"] = t.pointCount;
            tj["classId"] = t.classId;
            tj["warningActive"] = t.warningActive;
            targets.push_back(tj);
        }
        j["trackingTargets"] = targets;
    }

    nlohmann::json btr;
    btr["seconds"] = backtrackSecsEdit_ ? backtrackSecsEdit_->text().toDouble() : 5.0;
    btr["cam"]      = backtrackCamCombo_ ? backtrackCamCombo_->currentData().toInt() : -1;
    j["backtrack"] = btr;

    // Record resolution (system control → Web sync)
    nlohmann::json sys;
    sys["recordResolution"] = recordResolution_[0];
    j["system"] = sys;
    // 聚类可视化数据（1Hz，不依赖 tracking 信号）
    if (fusion_ && fusion_->is_running()) {
        ClusterVisData cvData[32];
        uint32_t cvCount = 0;
        fusion_->copy_cluster_vis(cvData, 32, &cvCount);
        static int cvLogCnt = 0;
        if (++cvLogCnt % 10 == 0)
            fprintf(stderr, "[StatusJSON] clusterVis count=%u\n", cvCount);
        if (cvCount > 0) {
            nlohmann::json cArr = nlohmann::json::array();
            for (uint32_t i = 0; i < cvCount; ++i) {
                nlohmann::json cj;
                cj["cx"] = cvData[i].cx;
                cj["cy"] = cvData[i].cy;
                cj["radius"] = cvData[i].radius;
                cj["pointCount"] = cvData[i].pointCount;
                cj["isOrphan"] = cvData[i].isOrphan;
                cj["bboxIdx"] = cvData[i].bboxIdx;
                cArr.push_back(cj);
            }
            j["clusters"] = cArr;
        }
    }

    j["radarRangeMeters"] = fusionTrackerCfg_.radarRangeMeters;
    j["fusionConfigVersion"] = fusionConfigVersion_;

    j["ok"] = true;
    return j.dump();
}

std::string Widget::get_hw_json_() const
{
    nlohmann::json j;

    // CPU — 读取但不修改缓存的 prevCpuTotal_/prevCpuIdle_，
    // 避免干扰 update_hw_usage_() 的独立计算
    FILE* fp = fopen("/proc/stat", "r");
    if (fp) {
        uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
        int n = fscanf(fp, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                       &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);
        fclose(fp);
        if (n >= 4) {
            uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
            // 用静态变量保存上一次的 web 查询值，独立于 update_hw_usage_()
            static uint64_t webPrevTotal = 0;
            static uint64_t webPrevIdle  = 0;
            uint64_t totalDelta = total - webPrevTotal;
            uint64_t idleDelta  = idle  - webPrevIdle;
            if (webPrevTotal > 0 && totalDelta > 0)
                j["cpu"] = (int)(100 - (idleDelta * 100 / totalDelta));
            else
                j["cpu"] = 0;
            webPrevTotal = total;
            webPrevIdle  = idle;
        }
    } else {
        j["cpu"] = -1;
    }

    // Temperature
    fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (fp) {
        int raw;
        if (fscanf(fp, "%d", &raw) == 1) j["temp"] = raw / 1000;
        else j["temp"] = -1;
        fclose(fp);
    } else {
        j["temp"] = -1;
    }

    // RGA
    nlohmann::json rga = nlohmann::json::array();
    fp = fopen("/sys/kernel/debug/rkrga/load", "r");
    if (fp) {
        char line[128];
        while (fgets(line, sizeof(line), fp)) {
            int load;
            if (sscanf(line, "         load = %d%%", &load) == 1)
                rga.push_back(load);
        }
        fclose(fp);
    }
    j["rga"] = rga;

    // NPU
    nlohmann::json npu = nlohmann::json::array();
    fp = fopen("/sys/kernel/debug/rknpu/load", "r");
    if (fp) {
        char line[256];
        if (fgets(line, sizeof(line), fp)) {
            int c0, c1, c2;
            if (sscanf(line, "NPU load:  Core0: %d%%, Core1: %d%%, Core2: %d%%", &c0, &c1, &c2) == 3) {
                npu.push_back(c0);
                npu.push_back(c1);
                npu.push_back(c2);
            }
        }
        fclose(fp);
    }
    j["npu"] = npu;

    j["ok"] = true;
    return j.dump();
}

std::string Widget::get_videos_json_() const
{
    nlohmann::json result = nlohmann::json::array();
    QDir dir(recordDir_);
    QStringList filters;
    filters << "*.mp4" << "*.MP4";
    QFileInfoList files = dir.entryInfoList(filters, QDir::Files, QDir::Time);

    for (const QFileInfo& fi : files) {
        nlohmann::json v;
        v["name"] = fi.fileName().toStdString();
        v["path"] = fi.absoluteFilePath().toStdString();
        v["size"]  = fi.size();

        // 读取分辨率和时长 (libavformat)
        AVFormatContext* ctx = avformat_alloc_context();
        if (ctx && avformat_open_input(&ctx, fi.absoluteFilePath().toUtf8().constData(), nullptr, nullptr) == 0) {
            if (avformat_find_stream_info(ctx, nullptr) >= 0) {
                int vid = av_find_best_stream(ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
                if (vid >= 0) {
                    v["width"]  = ctx->streams[vid]->codecpar->width;
                    v["height"] = ctx->streams[vid]->codecpar->height;
                }
                if (ctx->duration > 0)
                    v["durationSec"] = (double)ctx->duration / AV_TIME_BASE;
                else
                    v["durationSec"] = 0;
            }
            avformat_close_input(&ctx);
        }
        if (ctx) avformat_free_context(ctx);

        result.push_back(v);
    }

    nlohmann::json resp;
    resp["videos"] = result;
    resp["ok"] = true;
    return resp.dump();
}

std::string Widget::get_fusion_config_json_() const
{
    nlohmann::json j;

    // Tracker config
    j["dbscanEpsMeters"]         = fusionTrackerCfg_.dbscanEpsMeters;
    j["dbscanMinPoints"]         = fusionTrackerCfg_.dbscanMinPoints;
    j["maxClusterDistanceMeters"] = fusionTrackerCfg_.maxClusterDistanceMeters;
    j["clusterPersistenceFrames"] = fusionTrackerCfg_.clusterPersistenceFrames;
    j["bboxClaimMaxPixelDist"]   = fusionTrackerCfg_.bboxClaimMaxPixelDist;
    j["minBboxClaimPoints"]      = fusionTrackerCfg_.minBboxClaimPoints;
    j["alpha"]                   = fusionTrackerCfg_.alpha;
    j["beta"]                    = fusionTrackerCfg_.beta;
    j["bboxAssocMaxDistMeters"]  = fusionTrackerCfg_.bboxAssocMaxDistMeters;
    j["orphanAssocMaxDistMeters"] = fusionTrackerCfg_.orphanAssocMaxDistMeters;
    j["minHitsToConfirm"]        = fusionTrackerCfg_.minHitsToConfirm;
    j["maxLostFrames"]           = fusionTrackerCfg_.maxLostFrames;
    j["maxTracks"]               = fusionTrackerCfg_.maxTracks;
    j["warningEnterDistMeters"]  = fusionTrackerCfg_.warningEnterDistMeters;
    j["warningExitDistMeters"]   = fusionTrackerCfg_.warningExitDistMeters;
    j["clusterVisOpacity"]       = fusionTrackerCfg_.clusterVisOpacity;

    // Camera intrinsics
    for (int i = 0; i < 2; ++i) {
        nlohmann::json cam;
        cam["fx"] = fusionCamCfg_[i].fx;
        cam["fy"] = fusionCamCfg_[i].fy;
        cam["cx"] = fusionCamCfg_[i].cx;
        cam["cy"] = fusionCamCfg_[i].cy;
        cam["imgWidth"]  = fusionCamCfg_[i].imgWidth;
        cam["imgHeight"] = fusionCamCfg_[i].imgHeight;
        j[QString("cam%1").arg(i).toStdString()] = cam;
    }

    j["fusionEnabled"] = fusionEnabled_;
    j["camCount"] = fusionCamCount_;
    j["showEisControl"] = showEisControl_;

    j["ok"] = true;
    return j.dump();
}

std::string Widget::get_eis_config_json_() const
{
    nlohmann::json j;
    j["mode"] = "visual-primary-imu-assist";
    j["imuAssistWindowMs"] = imuAssistWindowMs_;

    for (int i = 0; i < 2; ++i) {
        nlohmann::json cam;
        cam["inputWidth"] = visualEisCfg_[i].inputWidth;
        cam["inputHeight"] = visualEisCfg_[i].inputHeight;
        cam["processWidth"] = visualEisCfg_[i].processWidth;
        cam["processHeight"] = visualEisCfg_[i].processHeight;
        cam["maxCorners"] = visualEisCfg_[i].maxCorners;
        cam["qualityLevel"] = visualEisCfg_[i].qualityLevel;
        cam["minDistance"] = visualEisCfg_[i].minDistance;
        cam["minTrackedPoints"] = visualEisCfg_[i].minTrackedPoints;
        cam["minInliers"] = visualEisCfg_[i].minInliers;
        cam["ransacThreshold"] = visualEisCfg_[i].ransacThreshold;
        cam["maxOpticalFlow"] = visualEisCfg_[i].maxOpticalFlow;
        cam["maxOffsetPixel"] = visualEisCfg_[i].maxOffsetPixel;
        cam["enableImuAdaptiveAlpha"] = visualEisCfg_[i].enableImuAdaptiveAlpha;
        cam["alphaLow"] = visualEisCfg_[i].alphaLowVibration;
        cam["alphaMid"] = visualEisCfg_[i].alphaMidVibration;
        cam["alphaHigh"] = visualEisCfg_[i].alphaHighVibration;
        cam["enableRotationEstimate"] = visualEisCfg_[i].enableRotationEstimate;
        j[QString("cam%1").arg(i).toStdString()] = cam;
    }

    j["ok"] = true;
    return j.dump();
}

std::string Widget::web_eis_config_(const std::string& body)
{
    try {
        auto j = nlohmann::json::parse(body);

        if (j.contains("imuAssistWindowMs")) {
            imuAssistWindowMs_ = j["imuAssistWindowMs"];
        }

        for (int i = 0; i < 2; ++i) {
            std::string key = QString("cam%1").arg(i).toStdString();
            if (!j.contains(key)) continue;

            auto& cam = j[key];
            if (cam.contains("processWidth")) visualEisCfg_[i].processWidth = cam["processWidth"];
            if (cam.contains("processHeight")) visualEisCfg_[i].processHeight = cam["processHeight"];
            if (cam.contains("maxCorners")) visualEisCfg_[i].maxCorners = cam["maxCorners"];
            if (cam.contains("qualityLevel")) visualEisCfg_[i].qualityLevel = cam["qualityLevel"];
            if (cam.contains("minDistance")) visualEisCfg_[i].minDistance = cam["minDistance"];
            if (cam.contains("minTrackedPoints")) visualEisCfg_[i].minTrackedPoints = cam["minTrackedPoints"];
            if (cam.contains("minInliers")) visualEisCfg_[i].minInliers = cam["minInliers"];
            if (cam.contains("ransacThreshold")) visualEisCfg_[i].ransacThreshold = cam["ransacThreshold"];
            if (cam.contains("maxOpticalFlow")) visualEisCfg_[i].maxOpticalFlow = cam["maxOpticalFlow"];
            if (cam.contains("maxOffsetPixel")) visualEisCfg_[i].maxOffsetPixel = cam["maxOffsetPixel"];
            if (cam.contains("enableImuAdaptiveAlpha")) visualEisCfg_[i].enableImuAdaptiveAlpha = cam["enableImuAdaptiveAlpha"];
            if (cam.contains("alphaLow")) visualEisCfg_[i].alphaLowVibration = cam["alphaLow"];
            if (cam.contains("alphaMid")) visualEisCfg_[i].alphaMidVibration = cam["alphaMid"];
            if (cam.contains("alphaHigh")) visualEisCfg_[i].alphaHighVibration = cam["alphaHigh"];
            if (cam.contains("enableRotationEstimate")) visualEisCfg_[i].enableRotationEstimate = cam["enableRotationEstimate"];

            visualEisCfg_[i].inputWidth = camWidth_[i];
            visualEisCfg_[i].inputHeight = camHeight_[i];
            if (visioner_) {
                visioner_->set_visual_eis_config(i, visualEisCfg_[i]);
            }
        }

        return R"({"ok":true})";
    } catch (...) {
        return R"({"ok":false,"error":"invalid JSON"})";
    }
}

// ============================================================================
// 数据回溯
// ============================================================================

// TODO: 硬盘数据管理类开发后启用
// class RecordWriter : public QThread {
//     Q_OBJECT
// public:
//     RecordWriter(int camNum, SentinelStreamer* streamer, QObject* parent = nullptr)
//         : QThread(parent), camNum_(camNum), streamer_(streamer), running_(false) {}
//     void stop() { running_ = false; wait(); }
// protected:
//     void run() override {
//         running_ = true;
//         while (running_) {
//             uint8_t* data = nullptr; size_t size = 0; uint64_t ts = 0;
//             if (streamer_->try_get_record_frame(camNum_, &data, &size, &ts)) {
//                 // diskManager_->save_frame(camNum_, data, size, ts);
//                 streamer_->release_record_frame(camNum_, data);
//             } else {
//                 usleep(5000);
//             }
//         }
//     }
// private:
//     int camNum_;
//     SentinelStreamer* streamer_;
//     std::atomic<bool> running_;
// };

void Widget::build_backtrack_page_()
{
    QWidget* page = ui->pageBacktrack;
    QVBoxLayout* rootLayout = new QVBoxLayout(page);
    rootLayout->setContentsMargins(8, 8, 8, 4);
    rootLayout->setSpacing(6);

    // 标题栏
    QFrame* titleBar = new QFrame(page);
    titleBar->setFixedHeight(38);
    titleBar->setStyleSheet("QFrame { background-color: #F4EAC5; border-radius: 10px; }");
    QHBoxLayout* titleLayout = new QHBoxLayout(titleBar);
    titleLayout->setContentsMargins(12, 0, 12, 0);
    QLabel* titleLabel = new QLabel("数据回溯管理", titleBar);
    titleLabel->setStyleSheet("font-size: 16px; font-weight: 700; color: #58a6ff;");
    titleLayout->addWidget(titleLabel);
    titleLayout->addStretch();
    QPushButton* btnBack = new QPushButton("返回", titleBar);
    btnBack->setFixedSize(80, 28);
    btnBack->setStyleSheet("font-size: 12px; color: #2d3535; background-color: #F5F0D7; border: 1px solid #8b949e; border-radius: 8px;");
    connect(btnBack, &QPushButton::clicked, this, &Widget::on_btn_back_from_backtrack_);
    titleLayout->addWidget(btnBack);
    rootLayout->addWidget(titleBar);

    // 参数区
    QFrame* paramFrame = new QFrame(page);
    QHBoxLayout* paramLayout = new QHBoxLayout(paramFrame);
    paramLayout->setContentsMargins(0, 0, 0, 0);
    paramLayout->setSpacing(10);

    QLabel* lblSecs = new QLabel("回溯秒数:", paramFrame);
    lblSecs->setStyleSheet("font-size: 13px; color: #2d3535;");
    paramLayout->addWidget(lblSecs);

    backtrackSecsEdit_ = new QLineEdit(paramFrame);
    backtrackSecsEdit_->setText("5.0");
    backtrackSecsEdit_->setFixedWidth(70);
    backtrackSecsEdit_->setValidator(new QDoubleValidator(0.1, 30.0, 1, backtrackSecsEdit_));
    backtrackSecsEdit_->installEventFilter(this);
    backtrackSecsEdit_->setStyleSheet("font-size: 13px; color: #2d3535; background: #F5F0D7; border: 1px solid #30363d; border-radius: 6px; padding: 2px 4px;");
    paramLayout->addWidget(backtrackSecsEdit_);

    QLabel* lblCam = new QLabel("摄像头:", paramFrame);
    lblCam->setStyleSheet("font-size: 13px; color: #2d3535;");
    paramLayout->addWidget(lblCam);

    backtrackCamCombo_ = new QComboBox(paramFrame);
    backtrackCamCombo_->addItem("全部", -1);
    backtrackCamCombo_->addItem("相机1", 0);
    backtrackCamCombo_->addItem("相机2", 1);
    backtrackCamCombo_->setFixedWidth(80);
    backtrackCamCombo_->setStyleSheet("QComboBox { font-size: 13px; color: #2d3535; background: #F5F0D7; border: 1px solid #30363d; border-radius: 6px; padding: 2px 4px; }");
    paramLayout->addWidget(backtrackCamCombo_);

    paramLayout->addStretch();

    QPushButton* btnBacktrack = new QPushButton("手动回溯", paramFrame);
    btnBacktrack->setFixedSize(80, 28);
    btnBacktrack->setStyleSheet("font-size: 12px; font-weight: 600; color: #e6edf3; background-color: #1f6feb; border: 1px solid #388bfd; border-radius: 8px;");
    connect(btnBacktrack, &QPushButton::clicked, this, &Widget::on_btn_backtrack_);
    paramLayout->addWidget(btnBacktrack);

    QPushButton* btnRefresh = new QPushButton("刷新列表", paramFrame);
    btnRefresh->setFixedSize(80, 28);
    btnRefresh->setStyleSheet("font-size: 12px; color: #2d3535; background-color: #F5F0D7; border: 1px solid #8b949e; border-radius: 8px;");
    connect(btnRefresh, &QPushButton::clicked, this, &Widget::on_btn_refresh_backtrack_);
    paramLayout->addWidget(btnRefresh);

    rootLayout->addWidget(paramFrame);

    // 文件列表
    backtrackTable_ = new QTableWidget(page);
    backtrackTable_->setColumnCount(4);
    backtrackTable_->setHorizontalHeaderLabels({"文件名", "时长", "大小", "操作"});
    backtrackTable_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    backtrackTable_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    backtrackTable_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    backtrackTable_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    backtrackTable_->verticalHeader()->setVisible(false);
    backtrackTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    backtrackTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    backtrackTable_->setStyleSheet(
        "QTableWidget { background-color: #F4EAC5; border: 1px solid #30363d; border-radius: 8px; font-size: 14px; color: #2d3535; }"
        "QHeaderView::section { background-color: #F5F0D7; color: #2d3535; font-size: 13px; font-weight: 600; padding: 6px; border: none; border-bottom: 2px solid #30363d; }"
        "QTableWidget::item { padding: 6px; }");
    rootLayout->addWidget(backtrackTable_, 1);

    // 状态标签
    QLabel* statusLabel = new QLabel("就绪", page);
    statusLabel->setFixedHeight(20);
    statusLabel->setAlignment(Qt::AlignCenter);
    statusLabel->setStyleSheet("font-size: 12px; color: #4a5555;");
    rootLayout->addWidget(statusLabel);
}

// ---- NVMe ----

void Widget::init_nvme_()
{
    if (!config_.value("Backtrack/enabled", true).toBool())
        return;

    nvme_manager_ = new NVMeDataManager();
    if (!nvme_manager_->initialize(nvmeDevicePath_.toUtf8().constData())) {
        fprintf(stderr, "[SentinelQT] NVMe init failed\n");
        delete nvme_manager_;
        nvme_manager_ = nullptr;
        set_status_("NVMe 不可用", "#f85149");
        return;
    }

    nvme_worker_ = new NvmeWorker(streamer_, nvme_manager_, 2);
    nvme_thread_ = new QThread(this);
    nvme_worker_->moveToThread(nvme_thread_);

    connect(nvme_thread_, &QThread::started,
            nvme_worker_, &NvmeWorker::start);
    connect(nvme_worker_, &NvmeWorker::error,
            this, [this](const QString& msg) {
                set_status_("NVMe: " + msg, "#f85149");
            });

    nvme_thread_->start();
    set_status_("NVMe 回溯已启动", "#2ea043");
}

void Widget::deinit_nvme_()
{
    if (nvme_worker_) {
        nvme_worker_->stop();
        if (nvme_thread_ && nvme_thread_->isRunning()) {
            nvme_thread_->quit();
            nvme_thread_->wait(3000);
        }
        delete nvme_worker_;
        nvme_worker_ = nullptr;
        delete nvme_thread_;
        nvme_thread_ = nullptr;
    }
    if (nvme_manager_) {
        nvme_manager_->shutdown();
        delete nvme_manager_;
        nvme_manager_ = nullptr;
    }
}

void Widget::do_backtrack_(uint64_t triggerTsUs, int cameraId,
                           const QString& label)
{
    if (!nvme_manager_) {
        set_status_("NVMe 未初始化", "#f85149");
        return;
    }

    double backSecs = backtrackSecsEdit_
        ? backtrackSecsEdit_->text().toDouble()
        : config_.value("Backtrack/maxBacktrackSeconds", 5.0).toDouble();

    uint64_t triggerNs = triggerTsUs * 1000;

    QDir dir(backtrackDir_);
    if (!dir.exists()) dir.mkpath(".");

    QString tsStr = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");

    int startCam = (cameraId == -1) ? 0 : cameraId;
    int endCam   = (cameraId == -1) ? 1 : cameraId;
    int okCount  = 0;

    for (int cam = startCam; cam <= endCam; ++cam) {
        QString fileName = QString("backtrack_%1_%2_cam%3.mp4")
            .arg(label, tsStr)
            .arg(cam);
        QString filePath = dir.absoluteFilePath(fileName);

        bool ok = nvme_manager_->export_trigger_video_clip(
            triggerNs, filePath.toStdString(), backSecs, 15,
            camWidth_[cam], camHeight_[cam], cam,
            true);

        if (ok) {
            fprintf(stderr, "[SentinelQT] backtrack clip saved: %s\n",
                    filePath.toUtf8().constData());
            ++okCount;
        } else {
            fprintf(stderr, "[SentinelQT] backtrack export failed for cam%d\n", cam);
        }
    }

    if (okCount > 0) {
        set_status_(QString("回溯完成: %1 个视频").arg(okCount), "#2ea043");
    } else {
        set_status_("回溯导出失败", "#f85149");
    }

    on_btn_refresh_backtrack_();
}

void Widget::on_btn_backtrack_page_()
{
    ui->stackedWidget->setCurrentIndex(3);
    on_btn_refresh_backtrack_();
}

void Widget::on_btn_back_from_backtrack_()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void Widget::on_btn_backtrack_()
{
    // 检查是否在推流/录像，否则 RecordBufferPool 无数据
    bool anyActive = false;
    for (int i = 0; i < 2; ++i) {
        if (streamer_->is_streaming(i) || streamer_->is_recording(i)) {
            anyActive = true;
            break;
        }
    }
    if (!anyActive) {
        QMessageBox::warning(this, "无法回溯",
            "当前未在推流/录像，没有帧数据写入磁盘。\n请先开启推流或录像。");
        return;
    }

    double backSecs = backtrackSecsEdit_->text().toDouble();
    int cam = backtrackCamCombo_->currentData().toInt();

    fprintf(stderr,
        "[SentinelQT] manual backtrack: cam=%d seconds=%.1f\n",
        cam, backSecs);

    auto now = std::chrono::steady_clock::now();
    uint64_t nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();

    do_backtrack_(nowUs, cam, QString("manual_cam%1").arg(cam));
}

void Widget::on_btn_refresh_backtrack_()
{
    backtrackTable_->setRowCount(0);
    if (backtrackDir_.isEmpty()) return;

    QDir dir(backtrackDir_);
    if (!dir.exists()) {
        fprintf(stderr, "[SentinelQT] backtrack dir not found: %s\n",
                backtrackDir_.toUtf8().constData());
        return;
    }

    QFileInfoList files = dir.entryInfoList(QDir::Files, QDir::Time);
    for (const QFileInfo& fi : files) {
        int row = backtrackTable_->rowCount();
        backtrackTable_->insertRow(row);

        QTableWidgetItem* nameItem = new QTableWidgetItem(fi.fileName());
        nameItem->setToolTip(fi.fileName());
        backtrackTable_->setItem(row, 0, nameItem);

        // 解析时长
        QString durText = "--:--";
        QString filePath = fi.absoluteFilePath();
        AVFormatContext* ctx = avformat_alloc_context();
        if (ctx && avformat_open_input(&ctx, filePath.toUtf8().constData(), nullptr, nullptr) == 0) {
            if (avformat_find_stream_info(ctx, nullptr) >= 0) {
                int64_t durUs = ctx->duration;
                if (durUs <= 0) {
                    int vs = av_find_best_stream(ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
                    if (vs >= 0 && ctx->streams[vs]->duration > 0)
                        durUs = av_rescale_q(ctx->streams[vs]->duration,
                                             ctx->streams[vs]->time_base, AV_TIME_BASE_Q);
                }
                durText = format_duration_(durUs);
            }
            avformat_close_input(&ctx);
        }
        if (ctx) avformat_free_context(ctx);

        QTableWidgetItem* durItem = new QTableWidgetItem(durText);
        durItem->setTextAlignment(Qt::AlignCenter);
        backtrackTable_->setItem(row, 1, durItem);

        double sizeKB = fi.size() / 1024.0;
        QString sizeStr = sizeKB >= 1024.0
            ? QString("%1 MB").arg(sizeKB / 1024.0, 0, 'f', 1)
            : QString("%1 KB").arg(sizeKB, 0, 'f', 1);
        backtrackTable_->setItem(row, 2, new QTableWidgetItem(sizeStr));

        QPushButton* delBtn = new QPushButton("删除");
        delBtn->setMinimumWidth(60);
        delBtn->setStyleSheet(
            "font-size: 13px; color: #2d3535; background-color: #F5F0D7;"
            " border: 1px solid #f85149; border-radius: 4px; padding: 3px 14px;");
        connect(delBtn, &QPushButton::clicked, this, [this, filePath]() {
            QMessageBox::StandardButton reply = QMessageBox::question(
                this, "确认删除",
                "确定要删除这个回溯文件吗？\n" + filePath,
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                QFile::remove(filePath);
                on_btn_refresh_backtrack_();
            }
        });
        backtrackTable_->setCellWidget(row, 3, delBtn);
    }
}

// ---- Web handlers ----

std::string Widget::web_backtrack_query_(const std::string& body)
{
    try {
        auto j = nlohmann::json::parse(body);
        int cam = j.value("cam", -1);
        double seconds = j.value("seconds", 5.0);

        // 检查是否在推流/录像
        bool anyActive = false;
        for (int i = 0; i < 2; ++i) {
            if (streamer_->is_streaming(i) || streamer_->is_recording(i)) {
                anyActive = true;
                break;
            }
        }
        if (!anyActive)
            return R"({"ok":false,"error":"not streaming or recording, no frame data"})";

        // 同步 Qt 界面控件
        backtrackSecsEdit_->setText(QString::number(seconds, 'f', 1));
        int comboIdx = backtrackCamCombo_->findData(cam);
        if (comboIdx >= 0) backtrackCamCombo_->setCurrentIndex(comboIdx);

        fprintf(stderr,
            "[SentinelQT] web backtrack query: cam=%d seconds=%.1f\n",
            cam, seconds);

        auto now = std::chrono::steady_clock::now();
        uint64_t nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();

        do_backtrack_(nowUs, cam, QString("web_cam%1").arg(cam));

        nlohmann::json resp;
        resp["ok"] = true;
        resp["message"] = "backtrack completed, check file list";
        return resp.dump();
    } catch (...) {
        return R"({"ok":false,"error":"invalid JSON"})";
    }
}

std::string Widget::get_backtrack_files_json_() const
{
    nlohmann::json files = nlohmann::json::array();
    if (!backtrackDir_.isEmpty()) {
        QDir dir(backtrackDir_);
        QFileInfoList list = dir.entryInfoList(QDir::Files, QDir::Time);
        for (const QFileInfo& fi : list) {
            nlohmann::json v;
            v["name"] = fi.fileName().toStdString();
            v["path"] = fi.absoluteFilePath().toStdString();
            v["size"] = fi.size();

            // 解析时长
            QString durText = "--:--";
            AVFormatContext* ctx = avformat_alloc_context();
            if (ctx && avformat_open_input(&ctx, fi.absoluteFilePath().toUtf8().constData(), nullptr, nullptr) == 0) {
                if (avformat_find_stream_info(ctx, nullptr) >= 0) {
                    int64_t durUs = ctx->duration;
                    if (durUs <= 0) {
                        int vs = av_find_best_stream(ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
                        if (vs >= 0 && ctx->streams[vs]->duration > 0)
                            durUs = av_rescale_q(ctx->streams[vs]->duration,
                                                 ctx->streams[vs]->time_base, AV_TIME_BASE_Q);
                    }
                    if (durUs > 0) {
                        int64_t totalSec = durUs / 1000000;
                        int h = totalSec / 3600;
                        int m = (totalSec % 3600) / 60;
                        int s = totalSec % 60;
                        if (h > 0)
                            durText = QString("%1:%2:%3").arg(h).arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
                        else
                            durText = QString("%1:%2").arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
                    }
                }
                avformat_close_input(&ctx);
            }
            if (ctx) avformat_free_context(ctx);
            v["duration"] = durText.toStdString();

            files.push_back(v);
        }
    }
    nlohmann::json resp;
    resp["files"] = files;
    resp["ok"] = true;
    return resp.dump();
}

bool Widget::eventFilter(QObject* obj, QEvent* event)
{
    if (event->type() == QEvent::FocusIn) {
        QLineEdit* le = qobject_cast<QLineEdit*>(obj);
        if (le && (fusionParamEdits_.values().contains(le) || le == backtrackSecsEdit_)) {
            virtualKeyboard_->show_for(le);
        }
    } else if (event->type() == QEvent::FocusOut) {
        QLineEdit* le = qobject_cast<QLineEdit*>(obj);
        if (le && (fusionParamEdits_.values().contains(le) || le == backtrackSecsEdit_)) {
            // 延迟判断：如果焦点移到了键盘按钮上则不隐藏
            QTimer::singleShot(50, this, [this]() {
                QWidget* fw = QApplication::focusWidget();
                if (!fw || !virtualKeyboard_->isAncestorOf(fw)) {
                    virtualKeyboard_->hide_keyboard();
                }
            });
        }
    } else if (event->type() == QEvent::Resize && obj == topDownView_) {
        QWidget* btn = topDownView_->property("legendHelpBtn").value<QWidget*>();
        if (btn) {
            int w = topDownView_->width();
            int h = topDownView_->height();
            btn->move(w - 26, h - 26);
        }
    }
    return QWidget::eventFilter(obj, event);
}
