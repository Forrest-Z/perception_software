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
#include "modules/perception/base/object.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/camera/tools/qt_show/channel_reader.h"

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = 0);
    ~Widget();

protected:
    void paintEvent(QPaintEvent *e);

private:

    void imageReaderCallback(const std::shared_ptr<apollo::drivers::Image>& imgData);
    void objectReaderCallback(const std::shared_ptr<apollo::perception::PerceptionObstacles>& objectData);

private:

    QList<QRect> rectList;
    QList<QString> strList;
    QImage currentImage;
    QPixmap imagePixmap;
    QMutex reader_mutex_;
    CyberChannReader<apollo::drivers::Image>* image_reader_;
    CyberChannReader<apollo::perception::PerceptionObstacles>* object_reader_;

private:

    void init();
    void drawPixmap();
};

#endif // M_WIDGET_H
