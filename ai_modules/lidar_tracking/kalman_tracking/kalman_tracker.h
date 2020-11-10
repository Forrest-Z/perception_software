#ifndef KALMANTRACKER_H
#define KALMANTRACKER_H

#include <memory>
#include <vector>
#include "lidar_common/tracked_object.h"
#include "lidar_tracking/kalman_tracking/mykalmanfilter.h"

namespace perception {
namespace lidar {

class KalmanTracker
{
public:
    KalmanTracker(size_t id);
    ~KalmanTracker();

    void setDeltatime(float time);
    void kalmanPrediction();//kalman预测
    void kalmanUpdate(const TrackedObject &object);//kalman更新
    void setSkippedFrame(size_t num);
    void addSkippedFrame(size_t num);
    size_t getSkippedFrame();

    void eraseTrace(int keepSize);
    int getTraceSize();
    void addTrace(TrackedObject object);
    std::vector<TrackedObject> getTrace();

    TrackedObject getPredictObject();

    size_t getTrackerId();

    bool isTracking();

    void initKalman();

private:

    cv::Mat computeMeasure(const TrackedObject &object);

private:

    bool startTracking;
    size_t tracker_id;
    size_t skipped_frames;

    std::shared_ptr<MyKalmanFilter> kalmanFilter;
    std::vector<TrackedObject> trace;
    TrackedObject predictObject;

    float dt;

private:
    void init();
    void saveConfig();
    void loadConfig();
};

}  // namespace lidar
}  // namespace perception

#endif // KALMANTRACKER_H
