#ifndef KALMANMULTIPLETRACKER_H
#define KALMANMULTIPLETRACKER_H

#include "base/object.h"
#include "lidar_common/tracked_object.h"
#include "lidar_tracking/kalman_tracking/kalman_tracker.h"
#include "lidar_tracking/kalman_tracking/tracker_merge.h"

namespace perception {
namespace lidar {

class KalmanMultipleTracker
{
public:
    KalmanMultipleTracker();
    ~KalmanMultipleTracker();

    //0-front 1-behind
    void mutilpleTracking(const double stamp, 
                          const std::vector<base::ObjectPtr>& detections, 
                          const int tracker_flag);

    void detectFrontObjectsToTreackedObjects(const double stamp,
                                             const std::vector<base::ObjectPtr> &detections,
                                             std::vector<TrackedObject> &trackingObjects);
    void detectBehindObjectsToTreackedObjects(const double stamp,
                                              const std::vector<base::ObjectPtr> &detections,
                                              std::vector<TrackedObject> &trackingObjects);
    void objectsToTreackedObjects(const double stamp,
                                  const std::vector<base::ObjectPtr> &detections,
                                  std::vector<TrackedObject> &trackingObjects);

    void mutilpleTracking(const double stamp, const std::vector<TrackedObject>& detections);
    void getTrackedObjectList(std::vector<base::ObjectPtr> &result);
    int getTrackersCount();

private:

    void matchAndTrackingObject(const double dt, const std::vector<TrackedObject>& detections);
    void calculateEuclideanDistance(const std::vector<TrackedObject>& detections, std::vector< std::vector<double> > &costMatrix);

    void init();

    void saveConfig();
    void loadConfig();

private:
    bool isFirstRun;//第一次运行
    size_t nextTrackerID;
    std::vector<std::shared_ptr<KalmanTracker> > listTrackers;//卡尔曼滤波跟踪算法

    double perStamp;

    double dist_thres;//两帧之间目标最大的移动距离
    int maximum_allowed_skipped_frames;//允许目标消失的最大帧数
    int max_trace_length;//跟踪轨迹的最大长度

    int tracker_flag;
    TrackerMerge mergeObject;
};

}  // namespace lidar
}  // namespace perception

#endif // KALMANMULTIPLETRACKER_H
