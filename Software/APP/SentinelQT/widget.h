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

class Icm45686Reader;
class EisStabilizer;
class SentinelVisioner;
class SentinelStreamer;
class SentinelLslidarer;
class SentinelYoloInfer;
class PreviewWorker;
class FusionWorker;
class NvmeWorker;
class NVMeDataManager;
class TopDownView;
class VirtualKeyboard;
class WebServer;
class QThread;
class QTimer;
class QTableWidget;
class QComboBox;
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

    void on_fusion_alert_backtrack_(int targetId, uint64_t alertTsUs);

    /** @brief Web 命令处理（由 WebServer 线程通过 BlockingQueuedConnection 调用）
     *  @return JSON 响应字符串 */
    std::string handle_web_command(const std::string& method,
                                   const std::string& path,
                                   const std::string& body);

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
    void on_btn_backtrack_();
    void on_btn_refresh_backtrack_();
    void on_btn_backtrack_page_();
    void on_btn_back_from_backtrack_();
    void update_clock_();
    void update_hw_usage_();
    void update_record_info_(int camNum);

    // ---- Fusion slots ----
    void on_btn_fusion_toggle_();
    void on_btn_back_from_fusion_();
    void on_tracking_updated_(const QVector<TrackedTarget>& targets);
    void on_fusion_param_changed_();
    void on_fusion_status_update_();

    // ---- OSD ----
    void on_btn_osd_(int camNum);
    void on_btn_eis_(int camNum);
    void on_btn_lidar_osd_(int camNum);

private:
    Ui::Widget *ui;

    SentinelVisioner* visioner_;
    SentinelStreamer* streamer_;
    WebServer* webServer_;

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
    double lastFps_[2];
    bool previewActive_[2];
    bool cameraPaused_[2];
    QString camStatus_[2];
    uint64_t prevCpuTotal_;
    uint64_t prevCpuIdle_;

    static Widget* instance_;

    // ---- Fusion ----
    SentinelLslidarer*  lidar_;
    LidarCameraFusion*  fusion_;
    SentinelYoloInfer*  yoloInfer_ = nullptr;
    bool                osdEnabled_[2] = {false, false};
    bool                eisEnabled_[2] = {false, false};
    bool                lidarOsdEnabled_[2] = {false, false};
    FusionWorker*       fusionWorker_;
    QThread*            fusionThread_;
    QTimer*             fusionStatusTimer_;
    bool                fusionEnabled_;
    TopDownView*        topDownView_;
    VirtualKeyboard*    virtualKeyboard_;
    TrackerConfig       fusionTrackerCfg_;
    int                  fusionConfigVersion_;
    CameraConfig        fusionCamCfg_[2];
    LidarConfig         lidarCfg_;
    QMap<QString, QLineEdit*> fusionParamEdits_;
    QVector<TrackedTarget>    lastTrackedTargets_;
    uint32_t            fusionCamCount_;

    // ---- EIS ----
    Icm45686Reader*     eisReader_ = nullptr;
    EisStabilizer*      eisStabilizer_ = nullptr;
    float               eisFocalX_[2];
    float               eisFocalY_[2];
    float               eisAxisSignX_[2];
    float               eisAxisSignY_[2];
    int32_t             eisMaxOffsetPixel_;
    uint32_t            eisHalfWindowMs_;

    void load_config_();
    void init_eis_();
    void deinit_eis_();
    void setup_lidar_osd_provider_();
    bool eis_offset_callback_(uint64_t timestampUs, int camNum, int32_t& offsetX, int32_t& offsetY);
    bool init_camera_(int camNum);
    void start_preview_(int camNum);
    void stop_preview_(int camNum);
    void scan_videos_();
    void update_camera_button_states_(int camNum);
    void update_button_states_();
    void set_status_(const QString& msg, const QString& color);
    void refresh_status_label_();
    std::string get_status_json_() const;
    std::string get_hw_json_() const;
    std::string get_videos_json_() const;
    std::string get_fusion_config_json_() const;

    // ---- Web 控制 helpers ----
    std::string web_start_preview_(int camNum);
    std::string web_stop_preview_(int camNum);
    std::string web_start_stream_(int camNum);
    std::string web_stop_stream_(int camNum);
    std::string web_start_record_(int camNum);
    std::string web_stop_record_(int camNum);
    std::string web_pause_(int camNum);
    std::string web_resume_(int camNum);
    std::string web_osd_start_(int camNum);
    std::string web_osd_stop_(int camNum);
    std::string web_eis_start_(int camNum);
    std::string web_eis_stop_(int camNum);
    std::string web_lidar_osd_start_(int camNum);
    std::string web_lidar_osd_stop_(int camNum);
    std::string web_system_start_();
    std::string web_system_stop_();
    std::string web_delete_video_(const std::string& path);
    std::string web_set_record_resolution_(int camNum, const std::string& body);
    std::string web_lidar_start_();
    std::string web_lidar_stop_();
    std::string web_fusion_start_();
    std::string web_fusion_stop_();
    std::string web_fusion_config_(const std::string& body);
    std::string web_fusion_intrinsics_(int camNum, const std::string& body);
    std::string web_backtrack_query_(const std::string& body);
    std::string web_delete_backtrack_(const std::string& body);
    std::string get_backtrack_files_json_() const;

    // ---- NVMe ----
    NVMeDataManager*    nvme_manager_ = nullptr;
    NvmeWorker*         nvme_worker_ = nullptr;
    QThread*            nvme_thread_ = nullptr;
    QString             nvmeDevicePath_;
    void init_nvme_();
    void deinit_nvme_();
    void do_backtrack_(uint64_t triggerTimestampUs, int cameraId, const QString& label);

    // ---- Backtrack helpers ----
    QTableWidget*       backtrackTable_;
    QLineEdit*          backtrackSecsEdit_;
    QComboBox*          backtrackCamCombo_;
    QString             backtrackDir_;
    void build_backtrack_page_();


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
