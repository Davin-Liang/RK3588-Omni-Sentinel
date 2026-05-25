#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QImage>
#include <QSettings>
#include <memory>

class SentinelVisioner;
class SentinelStreamer;
class PreviewWorker;
class QThread;
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
    void on_btn_start_stream_();
    void on_btn_stop_stream_();
    void on_btn_start_record_();
    void on_btn_stop_record_();

private:
    Ui::Widget *ui;

    SentinelVisioner* visioner_;
    SentinelStreamer* streamer_;
    PreviewWorker* previewWorker_;
    QThread* previewThread_;

    QSettings config_;

    QString rtspUrl_;
    QString recordPath_;

    int frameCount_;
    uint64_t lastFpsTsUs_;

    static Widget* instance_;
    bool init_camera_();
    void update_button_states_();
    void set_status_(const QString& msg, const QString& color);
};
#endif // WIDGET_H
