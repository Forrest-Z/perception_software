#include "lidar_tracking/kalman_tracking/mykalmanfilter.h"
#include <iostream>
#include <cmath>

namespace perception {
namespace lidar {

MyKalmanFilter::MyKalmanFilter(cv::Mat initData, float dt):
    deltatime(dt), initData(initData)
{
    initMaxtrix();
    initRQMaxtrix();
    //std::cout << "MyKalmanFilter()" << std::endl;

}

MyKalmanFilter::~MyKalmanFilter()
{
    //std::cout << "~MyKalmanFilter()" << std::endl;
}

void MyKalmanFilter::setDeltatime(float time)
{
    this->deltatime = time;
    kalman->transitionMatrix.at<float>(0, 2) = this->deltatime;
    kalman->transitionMatrix.at<float>(1, 3) = this->deltatime;
    initRQMaxtrix();
}

cv::Mat MyKalmanFilter::prediction()
{
    lastResult = kalman->predict();
    return lastResult;
}

cv::Mat MyKalmanFilter::update(cv::Mat &measure)
{
    //update using measurements
    lastResult = kalman->correct(measure);
    return lastResult;
}

void MyKalmanFilter::initRQMaxtrix()
{
    //Wk~(0,Q) Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     ]
    // [ 0    Ey  0     0     ]
    // [ 0    0   Ev_x  0     ]
    // [ 0    0   0     Ev_y  ]
    float ax = 0.1;
    float covx = ax * ax * std::pow(deltatime, 4.0f) / 4.0f;
    float ay = 0.3;
    float covy = ay * ay * std::pow(deltatime, 4.0f) / 4.0f;
    kalman->processNoiseCov = (cv::Mat_<float>(4, 4) <<
                               covx, 0, 0, 0,
                               0, covy, 0, 0,
                               0, 0, deltatime * deltatime, 0,
                               0, 0, 0, deltatime * deltatime);

    //Vk~(0,R) Measures Noise Covariance Maxtrix R
    // [ Ex   0   0     0     ]
    // [ 0    Ey  0     0     ]
    // [ 0    0   Ev_x  0     ]
    // [ 0    0   0     Ev_y  ]
    kalman->measurementNoiseCov = (cv::Mat_<float>(4, 4) <<
                                   0.0025, 0, 0, 0,
                                   0, 0.0025, 0, 0,
                                   0, 0, 4.0, 0,
                                   0, 0, 0, 4.0);
}

void MyKalmanFilter::initMaxtrix()
{
    //4 state variables, 4 measurements, 0 control variables
    kalman.reset(new cv::KalmanFilter(4, 4, 0, CV_32F));
    // Transition matrix A
    // [ 1 0 dT 0 ]
    // [ 0 1 0  dT]
    // [ 0 0 1  0 ]
    // [ 0 0 0  1 ]
    kalman->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, deltatime, 0, 0, 1, 0, deltatime, 0, 0, 1, 0, 0, 0, 0, 1);
    //init X vector
    kalman->statePre.at<float>(0) = initData.at<float>(0); // x
    kalman->statePre.at<float>(1) = initData.at<float>(1); // y
    kalman->statePre.at<float>(2) = initData.at<float>(2);
    kalman->statePre.at<float>(3) = initData.at<float>(3);

    //init Z vector
    kalman->statePost.at<float>(0) = initData.at<float>(0);
    kalman->statePost.at<float>(1) = initData.at<float>(1);
    kalman->statePost.at<float>(2) = initData.at<float>(2);
    kalman->statePost.at<float>(3) = initData.at<float>(3);
    //Measure matrix H
    // [ 1 0 0 0]
    // [ 0 1 0 0]
    // [ 0 0 1 0]
    // [ 0 0 0 1]
    cv::setIdentity(kalman->measurementMatrix);

    //priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)
    kalman->errorCovPre = (cv::Mat_<float>(4, 4) <<
                           0.04, 0, 0, 0,
                           0, 0.04, 0, 0,
                           0, 0, 4, 0,
                           0, 0, 0, 4);
}

}  // namespace lidar
}  // namespace perception

