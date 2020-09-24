/*
2010:Fast Segmentation of 3D Point Clouds for Ground Vehicles
*/
#ifndef LIDAR_GROUND_EXTRACTOR_H_
#define LIDAR_GROUND_EXTRACTOR_H_

#include "lidar_common/lidar_cloud_filter.h"

namespace perception {
namespace lidar {

class LidarGroundExtractor
{
public:

    struct PointXYZIRTColor
    {
        PCLPoint point;

        float radius; //cylindric coords on XY Plane
        float theta;  //angle deg on XY plane

        size_t radial_div;     //index of the radial divsion to which this point belongs to
        size_t concentric_div; //index of the concentric division to which this points belongs to
        size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

    LidarGroundExtractor();
    virtual ~LidarGroundExtractor();

    void filteringGround(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &dstPointcloud);

private:
    void typeToRTZColor(const PCLPointCloud::Ptr &srcCloudPoints, const pcl::PointIndices::Ptr &inputIndices,
                        std::vector<PointCloudXYZIRTColor> &radialOrderedClouds);
    void classifyGroundPoint(std::vector<PointCloudXYZIRTColor> &radialOrderedClouds,
                             pcl::PointIndices &out_ground_indices);

private:
    LidarCloudFilter<PCLPoint> cloudFilter;
    size_t radialDividersNum;
};

}  // namespace lidar
}  // namespace perception

#endif //LIDAR_GROUND_EXTRACTOR_H_
