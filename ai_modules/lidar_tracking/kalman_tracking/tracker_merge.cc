#include "lidar_tracking/kalman_tracking/tracker_merge.h"
#include <cmath>
#include <cstdlib>

namespace perception {
namespace lidar {

TrackerMerge::TrackerMerge()
{

}

TrackerMerge::~TrackerMerge()
{

}

void TrackerMerge::mergeAll(std::vector<std::shared_ptr<KalmanTracker> > &trackerList, const float mergeThreshold)
{
    std::vector<std::shared_ptr<KalmanTracker> > result;
    std::vector<bool> visited(trackerList.size(), false);
    std::vector<bool> mergedTrackers(trackerList.size(), false);
    std::vector<Eigen::Vector3f> inputData;
    inputData.clear();
    result.clear();
    for(size_t i = 0; i < trackerList.size(); i++)
    {
        inputData.push_back(trackerList[i]->getPredictObject().center);
        if(!trackerList[i]->isTracking())
        {
            visited[i] = true;
        }
    }
    for (size_t i = 0; i < trackerList.size(); i++)
    {
      if (!visited[i])
      {
        std::vector<size_t> mergeIndices;
        visited[i] = true;
        mergeIndices.push_back(i);
        checkObjectMerge(i, inputData, visited, mergeIndices, mergeThreshold);
        mergeTracker(trackerList, result, mergeIndices, mergedTrackers);
      }
    }
    for (size_t i = 0; i < trackerList.size(); i++)
    {
      // check for clusters not merged, add them to the output
      if (!mergedTrackers[i])
      {
        result.push_back(trackerList[i]);
      }
    }
    trackerList = result;
}

void TrackerMerge::checkObjectMerge(size_t inputId, const std::vector<Eigen::Vector3f> &inputData,
                       std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
                       float mergeThreshold)
{
    Eigen::Vector3f point_a = inputData[inputId];
    for (size_t i = 0; i < inputData.size(); i++)
    {
        if (i != inputId && !in_out_visited_clusters[i])
        {
            Eigen::Vector3f point_b = inputData[i];
            float hightDiff = std::abs(point_b[2] - point_a[2]);
            float distance = std::sqrt(std::pow(point_b[0] - point_a[0], 2) + std::pow(point_b[1] - point_a[1], 2));
            //std::cout << " with " << i << " dist:" << distance << " " << hightDiff << std::endl;
            if (distance <= mergeThreshold && hightDiff <= 1.5f)
            {
                in_out_visited_clusters[i] = true;
                out_merge_indices.push_back(i);
                //std::cout << "Merging " << inputId << " with " << i << " dist:" << distance << std::endl;
                checkObjectMerge(i, inputData, in_out_visited_clusters, out_merge_indices, mergeThreshold);
            }
        }
    }
}

void TrackerMerge::mergeTracker(const std::vector<std::shared_ptr<KalmanTracker> > &trackerList,
                               std::vector<std::shared_ptr<KalmanTracker> > &result,
                               std::vector<size_t> in_merge_indices,
                               std::vector<bool>& in_out_merged_clusters)
{
    std::shared_ptr<KalmanTracker> tracker = trackerList[in_merge_indices[0]];
    in_out_merged_clusters[in_merge_indices[0]] = true;
    for (size_t i = 1; i < in_merge_indices.size(); i++)
    {
        if(trackerList[in_merge_indices[i]]->getTrackerId() > tracker->getTrackerId())
        {
            tracker = trackerList[in_merge_indices[i]];
        }
        in_out_merged_clusters[in_merge_indices[i]] = true;
    }
    result.push_back(tracker);
}

}  // namespace lidar
}  // namespace perception

