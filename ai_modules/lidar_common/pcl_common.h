#ifndef PCL_COMMON_H
#define PCL_COMMON_H

#include "common/pcl_util.h"
#include <iostream>

namespace perception {
namespace lidar {

void pclToAttributePointCloud(const PCLPointCloud::Ptr &srcCloud, 
                              base::AttributePointCloud<base::PointF> &dstcloud);

}  // namespace lidar
}  // namespace perception

#endif // PCL_COMMON_H