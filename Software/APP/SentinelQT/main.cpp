#include "widget.h"
#include "lidar_tracking_types.h"

#include <QApplication>
#include <QVector>
#include <cstdio>

int main(int argc, char *argv[])
{
    // 禁用 stderr 缓冲，确保崩溃前日志不丢失
    setbuf(stderr, NULL);

    QApplication a(argc, argv);

    qRegisterMetaType<TrackedTarget>("TrackedTarget");
    qRegisterMetaType<QVector<TrackedTarget>>("QVector<TrackedTarget>");

    Widget w;
    w.setWindowFlags(Qt::FramelessWindowHint);
    w.showFullScreen();
    return a.exec();
}
