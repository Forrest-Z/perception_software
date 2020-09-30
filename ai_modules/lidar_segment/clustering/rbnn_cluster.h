/*
2008:A Clustering Method for Efficient Segmentation of 3D Laser Data
*/
#ifndef RBNN_CLUSTER_H_
#define RBNN_CLUSTER_H_

#include <vector>
#include <map>

#include "lidar_common/lidar_cloud_filter.h"

namespace perception {
namespace lidar {

class RBNNCluster
{
public:
    RBNNCluster();
    ~RBNNCluster();

    void clustering(const PCLPointCloud::Ptr &srcCloudPoints, const double radius, std::vector<int> &result);

private:
    std::vector<int> rbnn(const PCLPointCloud::Ptr &srcCloudPoints, const double radius)
    void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);
    int mostFrequentValue(const std::vector<int> &values);

private:

    LidarCloudFilter<PCLPoint> cloudFilter;

};

}  // namespace lidar
}  // namespace perception

#endif // RBNN_CLUSTER_H_