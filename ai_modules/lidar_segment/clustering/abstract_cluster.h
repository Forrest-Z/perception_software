#ifndef ABSTRACT_CLUSTER_H_
#define ABSTRACT_CLUSTER_H_

#include <vector>
#include <map>
#include "base/object.h"
#include "lidar_common/pcl_common.h"
#include "lidar_common/lidar_cloud_filter.h"

namespace perception {
namespace lidar {

class AbstractCluster
{
public:
    AbstractCluster() {};
    virtual ~AbstractCluster() {};

    virtual void clustering(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>> &objects) = 0;

protected:
    void clusterByDistance(const PCLPointCloud::Ptr &srcPointcloud, const std::vector<float> &seg_distance,
                           std::vector<PCLPointCloud::Ptr> &segmentPointCloudArray);
    
    void filteringByFrequentValue(const std::vector<int> &values, const int min_size, const int max_size,
                                  std::vector<int> &cluster_result);

    void computeCentroid(const PCLPointCloud::Ptr &srcPointCloud,
                         const std::vector<int> &localIndices,
                         const int min_size, const int max_size,
                         std::vector<std::shared_ptr<base::Object>>* objects);

    void computeCentroid(const PCLPointCloud::Ptr &srcPointCloud,
                         const std::vector<pcl::PointIndices> &localIndices,
                         std::vector<std::shared_ptr<base::Object>>* objects);

private:

    LidarCloudFilter<PCLPoint> cloudFilter;
};


}  // namespace lidar
}  // namespace perception


#endif /* ABSTRACT_CLUSTER_H_ */