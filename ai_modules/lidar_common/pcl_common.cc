#include "lidar_common/pcl_common.h"

namespace perception {
namespace lidar {

void pclToAttributePointCloud(
    const PCLPointCloud::Ptr &srcCloud,
    base::AttributePointCloud<base::PointF> &dstcloud) {
  dstcloud.clear();
  for (const auto point : *srcCloud) {
    base::PointF tempPoint;
    tempPoint.x = point.x;
    tempPoint.y = point.y;
    tempPoint.z = point.z;
    tempPoint.intensity = point.intensity;
    dstcloud.push_back(tempPoint);
  }
}

void typeToPCLCloud(const base::PointFCloudPtr &cloud,
                    PCLPointCloud::Ptr &dstCloud) {
  dstCloud->clear();
  for (size_t i = 0; i < cloud->size(); i++) {
    const auto &point = cloud->at(i);
    PCLPoint tempPoint;
    tempPoint.x = point.x;
    tempPoint.y = point.y;
    tempPoint.z = point.z;
    tempPoint.intensity = static_cast<float>(point.intensity);
    dstCloud->push_back(tempPoint);
  }
}

}  // namespace lidar
}  // namespace perception