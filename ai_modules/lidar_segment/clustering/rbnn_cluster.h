/*
2008:A Clustering Method for Efficient Segmentation of 3D Laser Data
*/
#ifndef RBNN_CLUSTER_H_
#define RBNN_CLUSTER_H_

#include "lidar_segment/clustering/abstract_cluster.h"

namespace perception {
namespace lidar {

class RBNNCluster: public AbstractCluster
{
public:
    RBNNCluster();
    ~RBNNCluster();

    void clustering(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>> &objects);

private:
    std::vector<int> rbnn(const PCLPointCloud::Ptr &srcCloudPoints, const float radius);
    void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);

private:

    std::vector<float> seg_distance_;
    std::vector<float> cluster_radius_;
};

}  // namespace lidar
}  // namespace perception

#endif // RBNN_CLUSTER_H_