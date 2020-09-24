/*
 * 2017: Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications
 */
#ifndef LIDARPLANEFITGROUND_H
#define LIDARPLANEFITGROUND_H

#include "lidar_common/lidar_cloud_filter.h"

namespace perception {
namespace lidar {

class LidarPlaneFitGround
{
public:
    LidarPlaneFitGround();
    virtual ~LidarPlaneFitGround();

    void filteringGround(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &dstPointcloud);
    void filteringGround(const pcl::PointCloud<PointXYZIRL>::Ptr &srcPointcloud, 
                         pcl::PointCloud<PointXYZIRL>::Ptr &dstPointcloud);

private:
    void extractGround(const pcl::PointCloud<PointXYZIRL>::Ptr &inputCloud, pcl::PointIndices &groundIndices);
    float estimatePlane();
    void extractInitialSeeds(const pcl::PointCloud<PointXYZIRL>::Ptr &inputCloud);
    void typeToXYZIRL(const PCLPointCloud::Ptr &srcPointcloud, const pcl::PointIndices::Ptr &inputIndices,
                      pcl::PointCloud<PointXYZIRL>::Ptr &resultCloud);

private:
    LidarCloudFilter<PCLPoint> cloudFilter;

    int numIter;
    int numLPR;
    float thresholdSeeds;
    float thresholdDist;

    Eigen::MatrixXf normal_;
    float d_ = 0;

    pcl::PointCloud<PointXYZIRL>::Ptr seedsCloud;
    pcl::PointCloud<PointXYZIRL>::Ptr groundCloud;
};

}  // namespace lidar
}  // namespace perception

#endif // LIDARPLANEFITGROUND_H
