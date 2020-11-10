#include "lidar_tracking/kalman_tracking/sort_tracker.h"
#include <iostream>
#include <cstdlib>

namespace perception {
namespace lidar {

size_t SortTracker::NextTrackerID = 0;

SortTracker::SortTracker(): startTracking(false)
{
    NextTrackerID++;
    init();
    //std::cout << "SortTracker()" << std::endl;
}

SortTracker::~SortTracker()
{
    //std::cout << "~SortTracker()" << std::endl;
}

//kalman预测
void SortTracker::kalmanPrediction()
{
    if(startTracking)
    {
        cv::Mat result = sortKalman->prediction();
        float w = std::sqrt(result.at<float>(2) * result.at<float>(3));
        float h = result.at<float>(2) / w;
        this->predictObject.center[0] = result.at<float>(0);
        this->predictObject.center[1] = result.at<float>(1);
        this->predictObject.size[0] = w;
        this->predictObject.size[1] = h;
        this->predictObject.velocity[0] = result.at<float>(4);
        this->predictObject.velocity[1] = result.at<float>(5);
        this->predictObject.tracker_id = tracker_id;
        this->trace.push_back(this->predictObject);
    }
}

//kalman更新
void SortTracker::kalmanUpdate(const TrackedObject &object)
{
    if(startTracking)
    {
        cv::Mat inputData = computeMeasure(object);
        cv::Mat result = sortKalman->update(inputData);
        float w = std::sqrt(result.at<float>(2) * result.at<float>(3));
        float h = result.at<float>(2) / w;
        this->predictObject = object;
        this->predictObject.center[0] = result.at<float>(0);
        this->predictObject.center[1] = result.at<float>(1);
        this->predictObject.size[0] = w;
        this->predictObject.size[1] = h;
        this->predictObject.velocity[0] = result.at<float>(4);
        this->predictObject.velocity[1] = result.at<float>(5);
        this->predictObject.tracker_id = tracker_id;
        this->trace.push_back(this->predictObject);
    }
    else
    {
        this->predictObject = object;
        this->predictObject.tracker_id = tracker_id;
        this->trace.push_back(this->predictObject);
    }
}

void SortTracker::setSkippedFrame(size_t num)
{
    this->skipped_frames = num;
}

void SortTracker::addSkippedFrame(size_t num)
{
    this->skipped_frames += num;
}

size_t SortTracker::getSkippedFrame()
{
    return this->skipped_frames;
}

void SortTracker::eraseTrace(int keepSize)
{
    if (keepSize <= this->getTraceSize())
        trace.erase(trace.begin(), trace.end() - keepSize);
}

TrackedObject SortTracker::getPredictObject()
{
    return this->predictObject;
}

size_t SortTracker::getTrackerId()
{
    return this->tracker_id;
}

int SortTracker::getTraceSize()
{
    return static_cast<int>(this->trace.size());
}

void SortTracker::addTrace(TrackedObject object)
{
    this->predictObject = object;
    this->predictObject.tracker_id = tracker_id;
    this->trace.push_back(this->predictObject);
}

std::vector<TrackedObject> SortTracker::getTrace()
{
    return this->trace;
}

bool SortTracker::isTracking()
{
    return startTracking;
}

void SortTracker::initKalman()
{
    cv::Mat initData = (cv::Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);
    float s = this->predictObject.size[0] * this->predictObject.size[1]; //scale is just area
    float r = this->predictObject.size[0] / this->predictObject.size[1];
    initData.at<float>(0) = this->predictObject.center[0];
    initData.at<float>(1) = this->predictObject.center[1];
    initData.at<float>(2) = s;
    initData.at<float>(3) = r;
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
            time = (this->trace[loop].timestamp - this->trace[loop-1].timestamp) / 1000000.0f;
            this->trace[loop].velocity[0] = distanceX / time;
            this->trace[loop].velocity[1] = distanceY / time;
            vx += this->trace[loop].velocity[0];
            vy += this->trace[loop].velocity[1];

//            std::cout << "diff time:" << time << " " << this->trace[loop-1].timestamp / 1000000.0 << " " <<
//                         this->trace[loop].timestamp / 1000000.0 << std::endl;
        }

        vx = vx / (cout - 1);
        vy = vy / (cout - 1);

        //std::cout << "vx:" << vx << "vy:" << vy << std::endl;
        initData.at<float>(4) = vx;
        initData.at<float>(5) = vy;
    }
    sortKalman.reset(new SortKalmanFilter(initData, dt));
    startTracking = true;
}

cv::Mat SortTracker::computeMeasure(const TrackedObject &object)
{
    float s = object.size[0] * object.size[1]; //scale is just area
    float r = object.size[0] / object.size[1];
    cv::Mat result = (cv::Mat_<float>(6, 1) << object.center[0], object.center[1],
            s, r, this->predictObject.velocity[0], this->predictObject.velocity[1]);
    int cout = static_cast<int>(trace.size());
    int endCout = std::max(cout - 4, 0);
    float vx = 0;
    float vy = 0;
    float time = 0;
    float distanceX = 0;
    float distanceY = 0;
    for(int loop = cout - 1; loop > endCout; loop--)
    {
        distanceX = object.center[0] - this->trace[loop].center[0];
        distanceY = object.center[1] - this->trace[loop].center[1];
        time = (object.timestamp - this->trace[loop].timestamp) / 1000000.0f;
        if(time < 0.0001f)
        {
            time = dt;
        }
        vx = vx + (distanceX / time);
        vy = vy + (distanceY / time);
    }
    if(vx != 0 && vy != 0)
    {
        result.at<float>(4) = vx / 3;
        result.at<float>(5) = vy / 3;
    }
    return result;
}

void SortTracker::init()
{
    tracker_id = NextTrackerID;
    loadConfig();
    sortKalman = nullptr;
    skipped_frames = 0;
    startTracking = false;
    trace.clear();
}

void SortTracker::saveConfig()
{
    cv::FileStorage fs;
    fs.open("./SortTracker.xml", cv::FileStorage::WRITE);

    cv::write(fs, "dt", dt);

    fs.release();
}

void SortTracker::loadConfig()
{
    cv::FileStorage fs;
    fs.open("./SortTracker.xml", cv::FileStorage::READ);

    cv::read(fs["dt"], dt, 0.1f);

    fs.release();
}

}  // namespace lidar
}  // namespace perception
