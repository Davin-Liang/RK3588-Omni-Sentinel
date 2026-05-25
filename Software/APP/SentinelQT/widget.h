#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QImage>
#include <QSettings>
#include <QDateTime>
#include <memory>

class SentinelVisioner;
class SentinelStreamer;
class PreviewWorker;
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
    void on_frame_ready_(const QImage& image);
    void on_btn_stream_();
    void on_btn_record_();
    void on_btn_system_();
    void on_btn_toggle_preview_();
    void on_btn_videos_();
    void on_btn_back_();
    void on_btn_refresh_videos_();
    void update_clock_();
    void update_hw_usage_();
    void update_record_info_();

private:
    Ui::Widget *ui;

    SentinelVisioner* visioner_;
    SentinelStreamer* streamer_;
    PreviewWorker* previewWorker_;
    QThread* previewThread_;

    QSettings config_;

    QString rtspUrl_;
    QString recordDir_;

    QTimer* clockTimer_;
    QTimer* recordTimer_;
    QDateTime recordStartTime_;
    QString currentRecordPath_;

    int frameCount_;
    uint64_t lastFpsTsUs_;
    bool previewActive_;
    uint64_t prevCpuTotal_;
    uint64_t prevCpuIdle_;
    bool systemRunning_;

    static Widget* instance_;
    bool init_camera_();
    void start_preview_();
    void stop_preview_();
    void scan_videos_();
    void update_button_states_();
    void set_status_(const QString& msg, const QString& color);
};
#endif // WIDGET_H
