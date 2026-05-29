#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QImage>
#include <QSettings>
#include <QDateTime>
#include <QMap>
#include <QLineEdit>
#include <QEvent>
#include <memory>

#include "lidar_camera_fusion.h"

class SentinelVisioner;
class SentinelStreamer;
class SentinelLslidarer;
class PreviewWorker;
class FusionWorker;
class TopDownView;
class VirtualKeyboard;
class QThread;
class QTimer;
enum class StreamerEvent;

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    static Widget* instance() { return instance_; }

    void on_streamer_event(int camNum, StreamerEvent event, const QString& detail);

private slots:
    void on_frame_ready_(int camNum, const QImage& image);
    void on_btn_stream_(int camNum);
    void on_btn_record_(int camNum);
    void on_btn_toggle_preview_(int camNum);
    void on_btn_pause_(int camNum);
    void on_btn_system_();
    void on_btn_videos_();
    void on_btn_back_();
    void on_btn_refresh_videos_();
    void update_clock_();
    void update_hw_usage_();
    void update_record_info_(int camNum);

    // ---- Fusion slots ----
    void on_btn_fusion_toggle_();
    void on_btn_back_from_fusion_();
    void on_tracking_updated_(const QVector<TrackedTarget>& targets);
    void on_fusion_param_changed_();
    void on_fusion_status_update_();

private:
    Ui::Widget *ui;

    SentinelVisioner* visioner_;
    SentinelStreamer* streamer_;

    PreviewWorker* previewWorker_[2];
    QThread* previewThread_[2];

    QSettings config_;

    QString deviceName_[2];
    int camWidth_[2];
    int camHeight_[2];
    QString rtspUrl_[2];
    int recordResolution_[2];
    QString recordDir_;

    QTimer* clockTimer_;
    QTimer* recordTimer_[2];
    QDateTime recordStartTime_[2];
    QString currentRecordPath_[2];

    int frameCount_[2];
    uint64_t lastFpsTsUs_[2];
    bool previewActive_[2];
    bool cameraPaused_[2];
    QString camStatus_[2];
    uint64_t prevCpuTotal_;
    uint64_t prevCpuIdle_;

    static Widget* instance_;

    // ---- Fusion ----
    SentinelLslidarer*  lidar_;
    LidarCameraFusion*  fusion_;
    FusionWorker*       fusionWorker_;
    QThread*            fusionThread_;
    QTimer*             fusionStatusTimer_;
    bool                fusionEnabled_;
    TopDownView*        topDownView_;
    VirtualKeyboard*    virtualKeyboard_;
    TrackerConfig       fusionTrackerCfg_;
    CameraConfig        fusionCamCfg_[2];
    LidarConfig         lidarCfg_;
    QMap<QString, QLineEdit*> fusionParamEdits_;
    QVector<TrackedTarget>    lastTrackedTargets_;
    uint32_t            fusionCamCount_;

    void load_config_();
    bool init_camera_(int camNum);
    void start_preview_(int camNum);
    void stop_preview_(int camNum);
    void scan_videos_();
    void update_camera_button_states_(int camNum);
    void update_button_states_();
    void set_status_(const QString& msg, const QString& color);
    void refresh_status_label_();

    // ---- Fusion helpers ----
    void load_lidar_config_();
    void load_fusion_config_();
    void build_fusion_param_ui_();
    void sync_ui_to_fusion_config_();
    void sync_fusion_config_to_ui_();
    void save_fusion_config_();

    bool eventFilter(QObject* obj, QEvent* event) override;
};
#endif // WIDGET_H
