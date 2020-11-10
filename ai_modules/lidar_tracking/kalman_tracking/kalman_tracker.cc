#include "lidar_tracking/kalman_tracking/kalman_tracker.h"
#include <iostream>
#include <cstdlib>
#include <limits>
#include <cmath>

namespace perception {
namespace lidar {

KalmanTracker::KalmanTracker(size_t id): startTracking(false), tracker_id(id)
{
    init();
    //std::cout << "KalmanTracker()" << std::endl;
}

KalmanTracker::~KalmanTracker()
{
    //std::cout << "~KalmanTracker()" << std::endl;
}

void KalmanTracker::setDeltatime(float time)
{
     if(startTracking)
     {
         this->dt = time;
         kalmanFilter->setDeltatime(dt);
     }
}

//kalman预测
void KalmanTracker::kalmanPrediction()
{
    if(startTracking)
    {
        cv::Mat result = kalmanFilter->prediction();
        this->predictObject.center[0] = result.at<float>(0);
        this->predictObject.center[1] = result.at<float>(1);
        this->predictObject.velocity[0] = result.at<float>(2);
        this->predictObject.velocity[1] = result.at<float>(3);
        this->predictObject.tracker_id = tracker_id;
        this->trace.push_back(this->predictObject);
    }
}

//kalman更新
void KalmanTracker::kalmanUpdate(const TrackedObject &object)
{
    if(startTracking)
    {
        const int count = getTraceSize();
        cv::Mat inputData = computeMeasure(object);
        /*
        std::cout << "tracker:" << this->predictObject.center[0] << "|" << this->predictObject.center[1]
                  << "|" << this->predictObject.velocity[0] << "|" << this->predictObject.velocity[1] << std::endl;
        std::cout << "update:" << inputData.at<float>(0) << "|" << inputData.at<float>(1) << "|"
                  << inputData.at<float>(2) << "|" << inputData.at<float>(3) << std::endl;
        */
        cv::Mat result = kalmanFilter->update(inputData);
        this->predictObject = object;
        this->predictObject.center[0] = result.at<float>(0);
        this->predictObject.center[1] = result.at<float>(1);
        this->predictObject.velocity[0] = result.at<float>(2);
        this->predictObject.velocity[1] = result.at<float>(3);
        this->predictObject.tracker_id = tracker_id;
        this->trace[count-1] = this->predictObject;
    }
    else
    {
        this->predictObject = object;
        this->predictObject.tracker_id = tracker_id;
        this->trace.push_back(this->predictObject);
    }
}

void KalmanTracker::setSkippedFrame(size_t num)
{
    this->skipped_frames = num;
}

void KalmanTracker::addSkippedFrame(size_t num)
{
    this->skipped_frames += num;
}

size_t KalmanTracker::getSkippedFrame()
{
    return this->skipped_frames;
}

void KalmanTracker::eraseTrace(int keepSize)
{
    if (keepSize <= this->getTraceSize())
        trace.erase(trace.begin(), trace.end() - keepSize);
}

TrackedObject KalmanTracker::getPredictObject()
{
    int cout = static_cast<int>(trace.size());
    int endCout = std::max(cout - 3, 0);
    unsigned int lane1 = 0;
    unsigned int lane2 = 0;
    TrackedObject result = this->predictObject;
    for(int loop = cout - 1; loop >= endCout; loop--)
    {
        if(this->trace[loop].laneId == 1)
        {
            lane1++;
        }
        else if(this->trace[loop].laneId == 2)
        {
            lane2++;
        }
    }
    if(lane1 > lane2)
    {
        result.laneId = 1;
    }
    else if(lane2 > lane1)
    {
        result.laneId = 2;
    }
    return result;
}

size_t KalmanTracker::getTrackerId()
{
    return this->tracker_id;
}

int KalmanTracker::getTraceSize()
{
    return static_cast<int>(this->trace.size());
}

void KalmanTracker::addTrace(TrackedObject object)
{
    this->predictObject = object;
    this->predictObject.tracker_id = tracker_id;
    this->trace.push_back(this->predictObject);
}

std::vector<TrackedObject> KalmanTracker::getTrace()
{
    return this->trace;
}

bool KalmanTracker::isTracking()
{
    return startTracking;
}

void KalmanTracker::initKalman()
{
    std::vector<float> vxList;
    std::vector<float> vyList;
    cv::Mat initData = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
    initData.at<float>(0) = this->predictObject.center[0];
    initData.at<float>(1) = this->predictObject.center[1];
    if(this->trace.size() > 1)
    {
        size_t cout = this->trace.size();
        float vx = 0;
        float vy = 0;
        float time = 0;
        float distanceX = 0;
        float distanceY = 0;
        for(size_t loop = 1; loop < cout; loop++)
        {
            distanceX = this->trace[loop].center[0] - this->trace[loop-1].center[0];
            distanceY = this->trace[loop].center[1] - this->trace[loop-1].center[1];
            time = this->trace[loop].timestamp - this->trace[loop-1].timestamp;
            time = 0.1f;
            if(time < 0.000001f)
            {
                std::cerr << "init tracker time:" << time << std::endl;
                return;
            }
            this->trace[loop].velocity[0] = distanceX / time;
            this->trace[loop].velocity[1] = distanceY / time;
            vx += this->trace[loop].velocity[0];
            vy += this->trace[loop].velocity[1];
            //std::cout << "diff time:" << time << " " << this->trace[loop-1].timestamp / 1000000.0 << " " <<
            //             this->trace[loop].timestamp / 1000000.0 << std::endl;
        }

        vx = vx / (cout - 1);
        vy = vy / (cout - 1);
        initData.at<float>(2) = vx;
        initData.at<float>(3) = vy;
        this->predictObject.velocity[0] = vx;
        this->predictObject.velocity[1] = vy;
    }
    kalmanFilter.reset(new MyKalmanFilter(initData, dt));
    startTracking = true;
}

cv::Mat KalmanTracker::computeMeasure(const TrackedObject &object)
{
    cv::Mat result = (cv::Mat_<float>(4, 1) << object.center[0], object.center[1],
            this->predictObject.velocity[0], this->predictObject.velocity[1]);
    int cout = static_cast<int>(trace.size());
    int endCout = std::max(cout - 3, 0);
    float vx = 10000;
    float vy = 10000;
    float distanceX = 0;
    float distanceY = 0;
    int number = 1;
    if(cout - 2 > endCout)
    {
        distanceX = object.center[0] - this->trace[cout - 2].center[0];
        distanceY = object.center[1] - this->trace[cout - 2].center[1];
        vx = distanceX / this->dt;
        vy = distanceY / this->dt;
    }
    if(vx < 10000 && vx > -10000
            && vy < 10000 && vy > -10000)
    {
        for(int loop = cout - 2; loop > endCout; loop--)
        {
            if(loop != 0)
            {
                vx = vx + this->trace[loop].velocity[0];
                vy = vy + this->trace[loop].velocity[1];
                number++;
            }
        }
        result.at<float>(2) = vx / number;
        result.at<float>(3) = vy / number;
        //std::cout << "number:" << number << std::endl;
    }
    return result;
}

void KalmanTracker::init()
{
    loadConfig();
    kalmanFilter = nullptr;
    skipped_frames = 0;
    startTracking = false;
    trace.clear();
}

void KalmanTracker::saveConfig()
{
    cv::FileStorage fs;
    fs.open("./KalmanTrack.xml", cv::FileStorage::WRITE);

    cv::write(fs, "dt", dt);

    fs.release();
}

void KalmanTracker::loadConfig()
{
    cv::FileStorage fs;
    fs.open("./KalmanTrack.xml", cv::FileStorage::READ);

    cv::read(fs["dt"], dt, 0.1f);

    fs.release();
}

}  // namespace lidar
}  // namespace perception

