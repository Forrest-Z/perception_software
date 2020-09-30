#include "modules/perception/camera/tools/qt_show/widget.h"
#include <QPainter>
#include <QMessageBox>
#include <strstream>
#include <iostream>
#include <iomanip>
#include "Eigen/Dense"

#define OBJECT_CHANNEL "/apollo/perception/obstacles"
#define CAMERA_IMAGE_CHANNEL "/apollo/sensor/camera/front_6mm/image"

Widget::Widget(QWidget *parent)
    : QWidget(parent){
    init();
}

Widget::~Widget()
{
    image_reader_->CloseChannel();
    object_reader_->CloseChannel();
    delete image_reader_;
    delete object_reader_;
    image_reader_ = nullptr;
    object_reader_ = nullptr;
}

void Widget::drawPixmap(){
    QPainter painter;
    QPen pen(QColor("#FF0000"), 2 ,Qt::DashLine);
    QFont font("Decorative", 15);
    imagePixmap = QPixmap::fromImage(currentImage);
    painter.begin(&imagePixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(pen);
    painter.setFont(font);
    for(int index = 0; index < rectList.count(); index++){
        painter.drawRect(rectList[index]);
        painter.drawText(rectList[index].topLeft(), strList[index]);
    }
    painter.end();
    this->update();
}

void Widget::paintEvent(QPaintEvent *e){
    QPainter painter(this);
    int width = static_cast<int>(imagePixmap.width() * 0.6);
    int height = static_cast<int>(imagePixmap.height() * 0.6);
    QPixmap temp = imagePixmap.scaled(width, height, Qt::KeepAspectRatio);
    painter.drawPixmap(QPoint(0,0), temp);
    painter.end();
    this->resize(temp.width(), temp.height());
}

void Widget::imageReaderCallback(const std::shared_ptr<apollo::drivers::Image>& imgData) {
  reader_mutex_.lock();
//   std::size_t imgSize = imgData->width() * imgData->height() * 3;
//   std::cout << "image size:" << imgSize << std::endl;
  currentImage = QImage(reinterpret_cast<const uint8_t *>(imgData->data().data()), imgData->width(),
                        imgData->height(), QImage::Format_RGB888);
  drawPixmap();
  reader_mutex_.unlock();
}

void Widget::objectReaderCallback(const std::shared_ptr<apollo::perception::PerceptionObstacles>& message){
    reader_mutex_.lock();
    const int count = message->perception_obstacle_size();
    rectList.clear();
    strList.clear();
    for(int index = 0; index < count; index++){
        apollo::perception::PerceptionObstacle* object = message->mutable_perception_obstacle(index);
        Eigen::Vector3d center;
        Eigen::Vector3d velocity;
        Eigen::Vector3d direction(static_cast<float>(std::cos(object->theta())),
                                  static_cast<float>(std::sin(object->theta())),
                                  0.0);
        std::ostringstream text;
        QPoint point1(static_cast<int>(object->bbox2d().xmin()), static_cast<int>(object->bbox2d().ymin()));
        QPoint point2(static_cast<int>(object->bbox2d().xmax()), static_cast<int>(object->bbox2d().ymax()));
        center[0] = (object->position()).x();
        center[1] = (object->position()).y();
        center[2] = (object->position()).z();
        //std::cout << " camera object x: "<<center[0]<<" y : "<<center[1] <<" z : "<< center[2]<<std::endl;
        velocity[0] = (object->velocity()).x();
        velocity[1] = (object->velocity()).y();
        velocity[2] = (object->velocity()).z();

        if (object->type() == 0) {
            text << " dist:" << center[2] << std::setprecision(2) << " velocity:" << velocity[2] << " class: UNKNOWN";
        } else if (object->type() == 1) {
            text << " dist:" << center[2] << std::setprecision(2) << " velocity:" << velocity[2]
                 << " class: UNKNOWN_MOVABLE";
        } else if (object->type() == 2) {
            text << " dist:" << center[2] << std::setprecision(2) << " velocity:" << velocity[2]
                 << " class: UNKNOWN_UNMOVABLE";
        } else if (object->type() == 3) {
            text << " dist:" << center[2] << std::setprecision(2) << " velocity:" << velocity[2]
                 << " class: PEDESTRIAN";
        } else if (object->type() == 4) {
            text << " dist:" << center[2] << std::setprecision(2) << " velocity:" << velocity[2] << " class: BICYCLE";
        } else if (object->type() == 5) {
            text << " dist:" << center[2] << std::setprecision(2) << " velocity:" << velocity[2]  << " class: VEHICLE";
        }

        QRect rect(point1, point2);
        rectList.append(rect);
        strList.append(QString(text.str().c_str()));
    }
  drawPixmap();
  reader_mutex_.unlock();
}

void Widget::init(){
    bool ret1 = false;
    bool ret2 = false;
    image_reader_ = new CyberChannReader<apollo::drivers::Image>();
    object_reader_ = new CyberChannReader<apollo::perception::PerceptionObstacles>();
    auto videoCallback =[this](const std::shared_ptr<apollo::drivers::Image>& pdata) {
                    this->imageReaderCallback(pdata);
                };
    auto objectCallback =[this](const std::shared_ptr<apollo::perception::PerceptionObstacles>& pdata) {
                    this->objectReaderCallback(pdata);
                };
    ret1 = image_reader_->InstallCallbackAndOpen(videoCallback, CAMERA_IMAGE_CHANNEL, "Visualizer-image");
    ret2 = object_reader_->InstallCallbackAndOpen(objectCallback, OBJECT_CHANNEL, "Visualizer-object");

    if (!ret1 || !ret2) {
        QMessageBox::warning(
                    this, tr("Settup Channel Callback"),
                    tr("Channel Callback cannot be installed!!!\nPlease check it!"),
                    QMessageBox::Ok);
    }
    this->setMinimumSize(700, 600);
    this->setWindowTitle(tr("camera radar fusion"));
}
