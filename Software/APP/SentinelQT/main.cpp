#include "widget.h"
#include "lidar_tracking_types.h"

#include <QApplication>
#include <QVector>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    qRegisterMetaType<TrackedTarget>("TrackedTarget");
    qRegisterMetaType<QVector<TrackedTarget>>("QVector<TrackedTarget>");

    Widget w;
    w.setWindowFlags(Qt::FramelessWindowHint);
    w.showFullScreen();
    return a.exec();
}
