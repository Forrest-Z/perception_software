#ifndef LIDAR_CLOUD_SAMPLE_H
#define LIDAR_CLOUD_SAMPLE_H

#include "common/pcl_util.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <random>
#include <iomanip>
#include <iostream>

namespace perception {
namespace lidar {

template <typename PointType>
class LidarCloudSample
{
public:

  LidarCloudSample() = default;
  ~LidarCloudSample() = default;

  void voxelSamplingPointcloud(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                               const float leafSize, typename pcl::PointCloud<PointType>::Ptr &result);

  void uniformSamplingPointcloud(const typename pcl::PointCloud<PointType>::Ptr &srcCloudPoint,
                                 const double radius, typename pcl::PointCloud<PointType>::Ptr &result);

  void randomSamplingPointCloud(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                const float keepRatio, pcl::PointIndices::Ptr &indices,
                                typename pcl::PointCloud<PointType>::Ptr &result);

};

template <typename PointType>
void LidarCloudSample<PointType>::voxelSamplingPointcloud(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                                          const float leafSize,
                                                          typename pcl::PointCloud<PointType>::Ptr &result)
{
    pcl::VoxelGrid<PointType> voxelGrid;
    result->clear();
    voxelGrid.setInputCloud(srcPointcloud);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
    voxelGrid.filter(*result);
    result->header = srcPointcloud->header;
}

template <typename PointType>
void LidarCloudSample<PointType>::uniformSamplingPointcloud(const typename pcl::PointCloud<PointType>::Ptr &srcCloudPoint,
                                                            const double radius,
                                                            typename pcl::PointCloud<PointType>::Ptr &result)
{
    // We need an additional object to store the indices of surviving points.
    pcl::PointCloud<int> keypointIndices;
    pcl::UniformSampling<PointType> filter;
    filter.setInputCloud(srcCloudPoint);
    filter.setRadiusSearch(radius);
    filter.compute(keypointIndices);
    pcl::copyPointCloud(*srcCloudPoint, keypointIndices.points, *result);
    result->header = srcCloudPoint->header;
}

template <typename PointType>
void LidarCloudSample<PointType>::randomSamplingPointCloud(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                                           const float keepRatio, pcl::PointIndices::Ptr &indices,
                                                           typename pcl::PointCloud<PointType>::Ptr &result)
{
    std::default_random_engine generator(srcPointcloud->header.stamp);
    std::uniform_real_distribution<float> distribution(0, 1);
    pcl::ExtractIndices<PointType> cloudIndices;
    result->clear();
    indices->indices.clear();
    cloudIndices.setInputCloud(srcPointcloud);
    for (size_t loop = 0; loop < srcPointcloud->points.size(); loop++)
    {
        if(distribution(generator) < keepRatio)
        {
            indices->indices.push_back(static_cast<int>(loop));
        }
    }
    cloudIndices.setIndices(indices);
    cloudIndices.setNegative(false);
    cloudIndices.filter(*result);
    result->header = srcPointcloud->header;
}

}  // namespace lidar
}  // namespace perception

#endif // LIDAR_CLOUD_SAMPLE_H
