#ifndef EUCLIDEAN_CLUSTER_H_
#define EUCLIDEAN_CLUSTER_H_

#include "lidar_segment/clustering/abstract_cluster.h"

namespace perception {
namespace lidar {

class EuclideanCluster: public AbstractCluster
{
public:
	EuclideanCluster();
    ~EuclideanCluster();

    void clustering(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>> &objects);

private:

    void clusteringToSegment(const PCLPointCloud::Ptr &srcPointCloud, const float maxClusterDistance,
                             std::vector<std::shared_ptr<base::Object>>* objects);

    void differenceNormalsSegmentation(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &result);

private:

    std::vector<float> seg_distance_;
    std::vector<float> cluster_distance_;
};

}  // namespace lidar
}  // namespace perception

#endif //EUCLIDEAN_CLUSTER_H_
