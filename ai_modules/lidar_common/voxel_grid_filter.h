#ifndef VOXEL_GRID_FILTER_H
#define VOXEL_GRID_FILTER_H

#include <vector>

#include "common/lidar_processing/pcl_util.h"
#include "lidar_common/geometry_util.h"

namespace perception {
namespace lidar {

//存放计算每个点云的idx和cloud_point_index的结构体
struct CloudPointIndexIdx
{
        unsigned int idx;
        unsigned int cloud_point_index;

        CloudPointIndexIdx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}
        bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

class VoxelGridFilter
{
public:
  VoxelGridFilter();
  virtual ~VoxelGridFilter();

  void applyFilter(const RawPointcloud::Ptr &InputCloudPoint, const RawPointcloud::Ptr &OutPointCloud,
                   float X_Voxel, float Y_Voxel, float Z_Voxel);
};

}  // namespace lidar
}  // namespace perception

#endif // VOXEL_GRID_FILTER_H
