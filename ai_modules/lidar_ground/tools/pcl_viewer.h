#ifndef PCL_VIEWER_H
#define PCL_VIEWER_H

#include "common/pcl_util.h"
#include <pcl/visualization/cloud_viewer.h>

namespace perception {
namespace lidar {

class PCLViewer
{
public:
    PCLViewer();
    ~PCLViewer();

    pcl::visualization::PCLVisualizer::Ptr createViewer(const PCLPointCloud::Ptr &srcCloudPoint);
    void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer, const PCLPointCloud::Ptr &srcCloud);
    
    void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const PCLPointCloud &srcCloud, const PCLPointCloud &srcCloud1);
    void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const pcl::PointCloud<PointXYZIRL> &srcCloud, const pcl::PointCloud<PointXYZIRL> &srcCloud1);

private:
    
};

}  // namespace lidar
}  // namespace perception

#endif // PCL_VIEWER_H
