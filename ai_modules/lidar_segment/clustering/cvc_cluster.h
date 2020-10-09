/*******************************
 *An implementation on "Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance" from IROS 2019 
 ********************************/
#ifndef CVC_CLUSTER_H_
#define CVC_CLUSTER_H_

#include <unordered_map>

#include "lidar_segment/clustering/abstract_cluster.h"

namespace perception {
namespace lidar {

class CVCCluster : public AbstractCluster
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

    void clustering(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<std::shared_ptr<base::Object>> &objects);

private:
    void calculateAPR(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<PointAPR>& vapr);
    void buildHashTable(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out);
    std::vector<int>  CVC(std::unordered_map<int, Voxel> &map_in, const std::vector<PointAPR>& vapr);

    float polarAngleCalculate(const float x, const float y);
    void findNeighbors(const int polar, const int range, const int azimuth, std::vector<int>& neighborindex);
    void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);
    
private:

};

}  // namespace lidar
}  // namespace perception

#endif // RBNN_CLUSTER_H_