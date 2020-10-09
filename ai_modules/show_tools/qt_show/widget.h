#ifndef M_WIDGET_H
#define M_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QPixmap>
#include <QMutex>
#include <QList>
#include <QString>
#include <memory>

#include "cyber/cyber.h"
#include "base/object.h"
#include "proto/sensor_image.pb.h"
#include "base/object_types.h"
#include "proto/perception_obstacle.pb.h"
#include "show_tools/qt_show/channel_reader.h"

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = 0);
    ~Widget();

protected:
    void paintEvent(QPaintEvent *e);

private:

    void imageReaderCallback(const std::shared_ptr<drivers::Image>& imgData);
    void objectReaderCallback(const std::shared_ptr<perception::PerceptionObstacles>& objectData);

private:

    QList<QRect> rectList;
    QList<QString> strList;
    QImage currentImage;
    QPixmap imagePixmap;
    QMutex reader_mutex_;
    CyberChannReader<drivers::Image>* image_reader_;
    CyberChannReader<perception::PerceptionObstacles>* object_reader_;

private:

    void init();
    void drawPixmap();
};

#endif // M_WIDGET_H
