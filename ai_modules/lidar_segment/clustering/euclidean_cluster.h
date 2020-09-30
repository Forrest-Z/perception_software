#ifndef EUCLIDEAN_CLUSTER_H_
#define EUCLIDEAN_CLUSTER_H_

#include <vector>
#include "base/object.h"
#include "lidar_common/pcl_common.h"

namespace perception {
namespace lidar {

class EuclideanCluster
{
public:
	EuclideanCluster();
    virtual ~EuclideanCluster();

    void clustering(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>> &objects);

private:

    void clusterByDistance(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>>* objects);

    void clusteringToSegment(const PCLPointCloud::Ptr &srcPointCloud, const float maxClusterDistance,
                             std::vector<std::shared_ptr<base::Object>>* objects);

    void computeCentroid(const PCLPointCloud::Ptr &srcPointCloud,
                         const std::vector<pcl::PointIndices> &localIndices,
                         std::vector<std::shared_ptr<base::Object>>* objects);

    void differenceNormalsSegmentation(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &result);

private:

    std::vector<float> seg_distance_;
    std::vector<float> cluster_distance_;
};

}  // namespace lidar
}  // namespace perception

#endif //EUCLIDEAN_CLUSTER_H_
