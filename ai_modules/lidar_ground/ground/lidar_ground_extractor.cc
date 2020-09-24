#include "lidar_ground/ground/lidar_ground_extractor.h"
#include <algorithm>
#include <iostream>
#include <cmath>

#define RADIAL_DIVIDER_ANGLE 0.2f

#define concentric_divider_distance_ 0.1f //0.1 meters default
#define min_height_threshold_ 0.3f
#define local_max_slope_ 30   //max slope of the ground between points, degree
#define general_max_slope_ 20 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.3f

namespace perception {
namespace lidar {

LidarGroundExtractor::LidarGroundExtractor(){
    radialDividersNum = static_cast<size_t>(std::ceil(360.0f / RADIAL_DIVIDER_ANGLE)) + 1;
}

LidarGroundExtractor::~LidarGroundExtractor(){
}

void LidarGroundExtractor::filteringGround(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &dstPointcloud){
    std::vector<PointCloudXYZIRTColor> radialOrderedClouds;
    pcl::PointIndices::Ptr inputIndices(new pcl::PointIndices);
    pcl::PointIndices groundIndices;
    pcl::ExtractIndices<PCLPoint> resultIndices;

    cloudFilter.pointcloudFilteringByZ(srcPointcloud, -10.0f, 3.0f, inputIndices);

    typeToRTZColor(srcPointcloud, inputIndices, radialOrderedClouds);
    classifyGroundPoint(radialOrderedClouds, groundIndices);

    resultIndices.setInputCloud(srcPointcloud);
    resultIndices.setIndices(boost::make_shared<pcl::PointIndices>(groundIndices));
    resultIndices.setNegative(true);
    resultIndices.filter(*dstPointcloud);
    dstPointcloud->header = srcPointcloud->header;
}

void LidarGroundExtractor::typeToRTZColor(const PCLPointCloud::Ptr &srcCloudPoints, const pcl::PointIndices::Ptr &inputIndices,
                                          std::vector<PointCloudXYZIRTColor> &radialOrderedClouds)
{
    radialOrderedClouds.clear();
    radialOrderedClouds.resize(radialDividersNum);

    for (size_t loop = 0; loop < inputIndices->indices.size(); loop++)
    {
        PointXYZIRTColor new_point;
        const int index = inputIndices->indices[loop];
        float radius = std::sqrt(srcCloudPoints->points[index].x * srcCloudPoints->points[index].x +
                                 srcCloudPoints->points[index].y * srcCloudPoints->points[index].y);
        float theta = static_cast<float>(std::atan2(srcCloudPoints->points[index].x, srcCloudPoints->points[index].y) * 180 / M_PI);
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)std::floor(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        auto concentric_div = (size_t)std::floor(fabs(radius / concentric_divider_distance_));

        new_point.point = srcCloudPoints->points[index];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = index;

        radialOrderedClouds[radial_div].push_back(new_point);

    } //end for

    //将同一根射线上的点按照半径（距离）排序
    for (size_t i = 0; i < radialDividersNum; i++)
    {
        std::sort(radialOrderedClouds[i].begin(), radialOrderedClouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
    }
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param radialOrderedClouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void LidarGroundExtractor::classifyGroundPoint(std::vector<PointCloudXYZIRTColor> &radialOrderedClouds,
                              pcl::PointIndices &out_ground_indices)
{
    out_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < radialOrderedClouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = 0;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < radialOrderedClouds[i].size(); j++) //loop through each point in the radial div
        {
            float pointsDistance = radialOrderedClouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * pointsDistance;
            float current_height = radialOrderedClouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * radialOrderedClouds[i][j].radius;

            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (pointsDistance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
                    if (current_height <= general_height_threshold && current_height >= -general_height_threshold)
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                //check if previous point is too far from previous one, if so classify again
                if (pointsDistance > reclass_distance_threshold_ &&
                    (current_height <= height_threshold && current_height >= -height_threshold))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if(current_ground)
            {
                out_ground_indices.indices.push_back(radialOrderedClouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                prev_ground = false;
            }

            prev_radius = radialOrderedClouds[i][j].radius;
            prev_height = radialOrderedClouds[i][j].point.z;
        }
    }
}

}  // namespace lidar
}  // namespace perception
