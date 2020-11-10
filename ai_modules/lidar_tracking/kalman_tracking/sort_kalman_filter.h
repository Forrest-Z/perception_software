#ifndef SORT_KALMAN_FILTER_H
#define SORT_KALMAN_FILTER_H

#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <memory>

namespace perception {
namespace lidar {

class SortKalmanFilter
{
public:
    SortKalmanFilter(cv::Mat initData, float dt = 0.1f);
    ~SortKalmanFilter();

    cv::Mat prediction();//预测
    cv::Mat update(cv::Mat inputData);//更新

private:
    void initMaxtrix();

private:
    std::unique_ptr<cv::KalmanFilter> kalman;
    float deltatime;
    float acceleration;

    cv::Mat initData;
    cv::Mat lastResult;//最后估计值
};

}  // namespace lidar
}  // namespace perception

#endif // SORT_KALMAN_FILTER_H
