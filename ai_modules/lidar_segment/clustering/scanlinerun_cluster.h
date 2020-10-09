/*
 * 2017: Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications
 */
#ifndef SCANLINE_CLUSTER_H
#define SCANLINE_CLUSTER_H

#include <vector>
#include <forward_list>
#include "base/object.h"
#include "lidar_common/pcl_common.h"
#include "lidar_common/geometry_util.h"

namespace perception {
namespace lidar {

class ScanLineRunCluster
{
public:
    ScanLineRunCluster();
    ~ScanLineRunCluster();

    void clustering(const pcl::PointCloud<PointXYZIRL>::Ptr &srcPointCloud, std::vector<std::shared_ptr<base::Object>> &objects);

private:
    void clusteringToSegment(const pcl::PointCloud<PointXYZIRL>::Ptr &srcPointCloud);
    void findRuns(size_t scanLine);
    void updateLabels(size_t scanLine);
    void mergeRuns(int currentLabel, int targetLabel);

private:
    size_t sensorModel;// also means number of sensor scan line.
    float runThreshold;// thresold of distance of points belong to the same run.
    float mergeThreshold;// threshold of distance of runs to be merged.
    int maxLabel;// max run labels, for disinguish different runs.

    // For organization of points.
    std::vector<std::vector<PointXYZIRL> > laserFrame;
    std::vector<std::vector<int> > pointsIndex;

    // Dummy object to occupy idx 0.
    std::forward_list<PointXYZIRL*> dummy;
    std::vector<std::forward_list<PointXYZIRL*> > runs;// For holding all runs.
};

}  // namespace lidar
}  // namespace perception

#endif // SCANLINE_CLUSTER_H
