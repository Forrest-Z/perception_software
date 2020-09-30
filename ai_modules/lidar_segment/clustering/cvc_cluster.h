/*******************************
 *An implementation on "Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance" from IROS 2019 
 ********************************/
#ifndef CVC_CLUSTER_H_
#define CVC_CLUSTER_H_

#include <vector>
#include <unordered_map>
#include <map>

#include "lidar_common/lidar_cloud_filter.h"

namespace perception {
namespace lidar {

class CVCCluster
{
public:

    struct PointAPR{
        float azimuth;
        float polar_angle;
        float range;
    };

    struct Voxel{
        bool haspoint = false;
        int cluster = -1;
        std::vector<int> index;
    };

    CVCCluster();
    ~CVCCluster();

    void clustering(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<int> &result);

private:
    void calculateAPR(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<PointAPR>& vapr);
    void buildHashTable(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out);
    std::vector<int>  CVC(std::unordered_map<int, Voxel> &map_in, const std::vector<PointAPR>& vapr);
    bool mostFrequentValue(std::vector<int> values, std::vector<int> &cluster_index);

    float polarAngleCalculate(const float x, const float y);
    
private:

    LidarCloudFilter<PCLPoint> cloudFilter;

};

}  // namespace lidar
}  // namespace perception

#endif // RBNN_CLUSTER_H_