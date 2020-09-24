#ifndef LIDAR_CLOUD_FILTER_H_
#define LIDAR_CLOUD_FILTER_H_

#include "common/pcl_util.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

namespace perception {
namespace lidar {

template <typename PointType>
class LidarCloudFilter
{
public:
    LidarCloudFilter() = default;
    ~LidarCloudFilter() = default;

    void removeOutlierPointcloud(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                 const int algorithmType,
                                 typename pcl::PointCloud<PointType>::Ptr &result);
    void pointcloudFilteringByZ(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                const float minZ, const float maxZ,
                                pcl::PointIndices::Ptr &indices);
    void pointcloudFilteringByY(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                const float minY, const float maxY,
                                pcl::PointIndices::Ptr &indices);
    void pointcloudFilteringByX(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                const float minX, const float maxX,
                                pcl::PointIndices::Ptr &indices);
    void pointcloudFilteringByDistance(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                       const float minDistance, const float maxDistance,
                                       pcl::PointIndices::Ptr &indices);
    void pointcloudFilteringByCrop(const typename pcl::PointCloud<PointType>::Ptr &srcPointCloud,
                                   const Eigen::Vector4f &minPoint, const Eigen::Vector4f &maxPoint,
                                   pcl::PointIndices::Ptr &indices);
    void getFilteringCloudPoint(const typename pcl::PointCloud<PointType>::Ptr &srcPointCloud,
                                const pcl::PointIndices::Ptr &indices,
                                typename pcl::PointCloud<PointType>::Ptr &result);
};

template <typename PointType>
void LidarCloudFilter<PointType>::removeOutlierPointcloud(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                                          const int algorithmType,
                                                          typename pcl::PointCloud<PointType>::Ptr &result)
{
    result->clear();
    switch(algorithmType)
    {
    case 0:
        {
            pcl::StatisticalOutlierRemoval<PointType> staticalOutlierfilter;
            staticalOutlierfilter.setInputCloud(srcPointcloud);
            staticalOutlierfilter.setMeanK(10);
            staticalOutlierfilter.setStddevMulThresh(0.5);
            staticalOutlierfilter.filter(*result);
        }
        break;
    case 1:
        {
            pcl::RadiusOutlierRemoval<PointType> radiusOutlierfilter;
            radiusOutlierfilter.setInputCloud(srcPointcloud);
            radiusOutlierfilter.setRadiusSearch(0.8);
            radiusOutlierfilter.setMinNeighborsInRadius (20);
            radiusOutlierfilter.filter(*result);
        }
        break;
    default:
        break;
    }
    result->header = srcPointcloud->header;
}

template <typename PointType>
void LidarCloudFilter<PointType>::pointcloudFilteringByZ(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                                         const float minZ, const float maxZ,
                                                         pcl::PointIndices::Ptr &indices)
{
    indices->indices.clear();
    for (size_t loop = 0; loop < srcPointcloud->points.size(); loop++)
    {
        if (srcPointcloud->points[loop].z >= minZ && srcPointcloud->points[loop].z <= maxZ)
        {
            indices->indices.push_back(static_cast<int>(loop));
        }
    }
}

template <typename PointType>
void LidarCloudFilter<PointType>::pointcloudFilteringByY(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                              const float minY, const float maxY,
                                              pcl::PointIndices::Ptr &indices)
{
    indices->indices.clear();
    for (size_t loop = 0; loop < srcPointcloud->points.size(); loop++)
    {
        if (srcPointcloud->points[loop].y >= minY && srcPointcloud->points[loop].y <= maxY)
        {
            indices->indices.push_back(static_cast<int>(loop));
        }
    }
}

template <typename PointType>
void LidarCloudFilter<PointType>::pointcloudFilteringByX(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                              const float minX, const float maxX,
                                              pcl::PointIndices::Ptr &indices)
{
    indices->indices.clear();
    for (size_t loop = 0; loop < srcPointcloud->points.size(); loop++)
    {
        if (srcPointcloud->points[loop].x >= minX && srcPointcloud->points[loop].x <= maxX)
        {
            indices->indices.push_back(static_cast<int>(loop));
        }
    }
}

template <typename PointType>
void LidarCloudFilter<PointType>::pointcloudFilteringByDistance(const typename pcl::PointCloud<PointType>::Ptr &srcPointcloud,
                                                     const float minDistance, const float maxDistance,
                                                     pcl::PointIndices::Ptr &indices)
{
    indices->indices.clear();
    for (size_t loop = 0; loop < srcPointcloud->points.size(); loop++)
    {
        float distance = std::sqrt(srcPointcloud->points[loop].x*srcPointcloud->points[loop].x +
                                   srcPointcloud->points[loop].y*srcPointcloud->points[loop].y);
        if (distance >= minDistance && distance <= maxDistance)
        {
            indices->indices.push_back(static_cast<int>(loop));
        }
    }
}

template <typename PointType>
void LidarCloudFilter<PointType>::pointcloudFilteringByCrop(const typename pcl::PointCloud<PointType>::Ptr &srcPointCloud,
                                                            const Eigen::Vector4f &minPoint, const Eigen::Vector4f &maxPoint,
                                                            pcl::PointIndices::Ptr &indices){
    pcl::CropBox<PointType> roof(true);
	roof.setMin(minPoint);
	roof.setMax(maxPoint);
	roof.setInputCloud(srcPointCloud);
	roof.filter(indices);

}

template <typename PointType>
void LidarCloudFilter<PointType>::getFilteringCloudPoint(const typename pcl::PointCloud<PointType>::Ptr &srcPointCloud,
                                              const pcl::PointIndices::Ptr &indices,
                                              typename pcl::PointCloud<PointType>::Ptr &result)
{
    pcl::ExtractIndices<PointType> cloudIndices;
    result->clear();
    cloudIndices.setInputCloud(srcPointCloud);
    cloudIndices.setIndices(indices);
    cloudIndices.setNegative(false);
    cloudIndices.filter(*result);
    result->header = srcPointCloud->header;
}

}  // namespace lidar
}  // namespace perception

#endif //LIDAR_CLOUD_FILTER_H_
