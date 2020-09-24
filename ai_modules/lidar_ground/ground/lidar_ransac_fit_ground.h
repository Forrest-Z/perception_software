
#ifndef LIDARRANSACFITGROUND_H
#define LIDARRANSACFITGROUND_H

#include <unordered_set>
#include <iostream>
#include "lidar_common/lidar_cloud_filter.h"

namespace perception {
namespace lidar {

class Platform
{
public:
    Platform(){}
    ~Platform(){}
    Eigen::Vector3d calnormal()
	{
        return Eigen::Vector3d(a,b,c);
    }
    double a;
    double b;
    double c;
    double d;
    int num_points;
   
};

class LidarRANSACFitGround{

public:

    LidarRANSACFitGround();
    ~LidarRANSACFitGround();

    void filteringGround(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &dstPointcloud);

    void removeFloor(const PCLPointCloud::Ptr &srcPointcloud, const int flag, PCLPointCloud::Ptr &dstPointcloud);

private:

    Platform RANSACPlane(const PCLPointCloud::Ptr &cloud);

private:

    LidarCloudFilter<PCLPoint> cloudFilter;

};

}  // namespace lidar
}  // namespace perception

#endif //LIDARRANSACFITGROUND_H