#ifndef TRACKER_MERGE_H
#define TRACKER_MERGE_H

#include "lidar_tracking/kalman_tracking/kalman_tracker.h"

namespace perception {
namespace lidar {

class TrackerMerge
{
public:
  TrackerMerge();
  ~TrackerMerge();

  void mergeAll(std::vector<std::shared_ptr<KalmanTracker> > &trackerList, const float mergeThreshold);

private:

  void checkObjectMerge(size_t inputId, const std::vector<Eigen::Vector3f> &inputData,
                         std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
                         float mergeThreshold);

  void mergeTracker(const std::vector<std::shared_ptr<KalmanTracker> > &trackerList,
                    std::vector<std::shared_ptr<KalmanTracker> > &result,
                    std::vector<size_t> in_merge_indices,
                    std::vector<bool>& in_out_merged_clusters);

};

}  // namespace lidar
}  // namespace perception

#endif // TRACKER_MERGE_H
