#ifndef MYKALMANFILTER_H
#define MYKALMANFILTER_H

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <memory>

namespace perception {
namespace lidar {

class MyKalmanFilter
{
public:
    MyKalmanFilter(cv::Mat initData, float dt = 0.1f);
    ~MyKalmanFilter();

    void setDeltatime(float time);
    cv::Mat prediction();//预测
    cv::Mat update(cv::Mat &measure);//更新

private:
    void initMaxtrix();
    void initRQMaxtrix();

private:
    std::unique_ptr<cv::KalmanFilter> kalman;
    float deltatime;
    cv::Mat initData;
    cv::Mat lastResult;//最后估计值
};

}  // namespace lidar
}  // namespace perception

#endif // MYKALMANFILTER_H
