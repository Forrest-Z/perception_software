#ifndef PCL_COMMON_H
#define PCL_COMMON_H

#include "common/lidar_processing/pcl_util.h"
#include <iostream>

namespace perception {
namespace lidar {

void pclToAttributePointCloud(const PCLPointCloud::Ptr &srcCloud, 
                              base::AttributePointCloud<base::PointF> &dstcloud);

void typeToPCLCloud(const base::PointFCloudPtr &cloud, 
                    PCLPointCloud::Ptr &dstCloud);

}  // namespace lidar
}  // namespace perception

#endif // PCL_COMMON_H