#include "lidar_tracking/kalman_tracking/sort_kalman_filter.h"

namespace perception {
namespace lidar {

SortKalmanFilter::SortKalmanFilter(cv::Mat initData, float dt)
                :deltatime(dt), initData(initData)
{
    //7 state variables, 4 measurements, 0 control variables
    kalman.reset(new cv::KalmanFilter(7, 4, 0, CV_32F));
    initMaxtrix();
}

SortKalmanFilter::~SortKalmanFilter()
{

}

cv::Mat SortKalmanFilter::prediction()
{
    if(kalman->statePre.at<float>(6) + kalman->statePre.at<float>(2) <= 0)
        kalman->statePre.at<float>(6) = 0;
    lastResult = kalman->predict();
    return lastResult;
}

cv::Mat SortKalmanFilter::update(cv::Mat measure)
{
    kalman->predict();
    //update using measurements
    lastResult = kalman->correct(measure);
    return lastResult;
}

void SortKalmanFilter::initMaxtrix()
{
    // Transition matrix A
//    [1,0,0,0,1,0,0]
//    [0,1,0,0,0,1,0]
//    [0,0,1,0,0,0,1]
//    [0,0,0,1,0,0,0]
//    [0,0,0,0,1,0,0]
//    [0,0,0,0,0,1,0]
//    [0,0,0,0,0,0,1]
    kalman->transitionMatrix = (cv::Mat_<float>(7, 7) <<
                                1,0,0,0,1,0,0,
                                0,1,0,0,0,1,0,
                                0,0,1,0,0,0,1,
                                0,0,0,1,0,0,0,
                                0,0,0,0,1,0,0,
                                0,0,0,0,0,1,0,
                                0,0,0,0,0,0,1);
    //init X vector
    //[u, v, s, r, u_, v_, s_]
    kalman->statePre.at<float>(0) = initData.at<float>(0);
    kalman->statePre.at<float>(1) = initData.at<float>(1);
    kalman->statePre.at<float>(2) = initData.at<float>(2);
    kalman->statePre.at<float>(3) = initData.at<float>(3);
    kalman->statePre.at<float>(4) = 0;
    kalman->statePre.at<float>(5) = 0;
    kalman->statePre.at<float>(6) = 0;

    //Measure matrix H
//    [1,0,0,0,0,0,0]
//    [0,1,0,0,0,0,0]
//    [0,0,1,0,0,0,0]
//    [0,0,0,1,0,0,0]
    kalman->measurementMatrix = (cv::Mat_<float>(4, 7) <<
                                 1,0,0,0,0,0,0,
                                 0,1,0,0,0,0,0,
                                 0,0,1,0,0,0,0,
                                 0,0,0,1,0,0,0);

    //Wk~(0,Q) Process Noise Covariance Matrix Q
    kalman->processNoiseCov = (cv::Mat_<float>(7, 7) <<
                               1,0,0,0,0,0,0,
                               0,1,0,0,0,0,0,
                               0,0,1,0,0,0,0,
                               0,0,0,1,0,0,0,
                               0,0,0,0,0.01,0,0,
                               0,0,0,0,0,0.01,0,
                               0,0,0,0,0,0,0.01 * 0.01);

    //Vk~(0,R) Measures Noise Covariance Maxtrix R
    kalman->measurementNoiseCov = (cv::Mat_<float>(4, 4) <<
                                   1,0,0,0,
                                   0,1,0,0,
                                   0,0,10,0,
                                   0,0,0,10);
    //priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)
    kalman->errorCovPre = (cv::Mat_<float>(7, 7) <<
                                    10,0,0,0,0,0,0,
                                    0,10,0,0,0,0,0,
                                    0,0,10,0,0,0,0,
                                    0,0,0,10,0,0,0,
                                    0,0,0,0,10000,0,0,
                                    0,0,0,0,0,10000,0,
                                    0,0,0,0,0,0,10000);
    //posteriori error estimate covariance matrix P
    //cv::setIdentity(kalman->errorCovPost, cv::Scalar::all(0.1));
}

}  // namespace lidar
}  // namespace perception