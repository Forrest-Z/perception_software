/*
 * @inproceedings{Bewley2016_sort,
  author={Bewley, Alex and Ge, Zongyuan and Ott, Lionel and Ramos, Fabio and Upcroft, Ben},
  booktitle={2016 IEEE International Conference on Image Processing (ICIP)},
  title={Simple online and realtime tracking},
  year={2016},
  pages={3464-3468},
  keywords={Benchmark testing;Complexity theory;Detectors;Kalman filters;Target tracking;Visualization;Computer Vision;Data Association;Detection;Multiple Object Tracking},
  doi={10.1109/ICIP.2016.7533003}
 */
#ifndef SORT_TRACKER_H
#define SORT_TRACKER_H

#include <memory>
#include <vector>
#include "lidar_common/tracked_object.h"
#include "lidar_tracking/kalman_tracking/sort_kalman_filter.h"

namespace perception {
namespace lidar {

class SortTracker
{
public:
    SortTracker();
    ~SortTracker();

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
    std::shared_ptr<SortKalmanFilter> sortKalman;
    bool startTracking;
    size_t tracker_id;
    size_t skipped_frames;

    std::vector<TrackedObject> trace;
    TrackedObject predictObject;

    float dt;

    static size_t NextTrackerID;

private:
    void init();
    void saveConfig();
    void loadConfig();
};

}  // namespace lidar
}  // namespace perception

#endif // SORT_TRACKER_H
