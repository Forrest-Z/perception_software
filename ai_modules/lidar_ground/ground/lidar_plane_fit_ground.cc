#include "lidar_ground/ground/lidar_plane_fit_ground.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <pcl/common/centroid.h>

namespace perception {
namespace lidar {

bool point_cmp(PointXYZIRL a, PointXYZIRL b)
{
    return a.z < b.z;
}

LidarPlaneFitGround::LidarPlaneFitGround()
{
    seedsCloud = pcl::PointCloud<PointXYZIRL>::Ptr(new pcl::PointCloud<PointXYZIRL>);
    groundCloud = pcl::PointCloud<PointXYZIRL>::Ptr(new pcl::PointCloud<PointXYZIRL>);

    numIter = 3;
    numLPR = 1000;
    thresholdSeeds = 1.2f;
    thresholdDist = 0.35f;
}

LidarPlaneFitGround::~LidarPlaneFitGround()
{

}

void LidarPlaneFitGround::filteringGround(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &dstPointcloud)
{
    pcl::PointCloud<PointXYZIRL>::Ptr inputCloud(new pcl::PointCloud<PointXYZIRL>);
    pcl::PointIndices::Ptr inputIndices(new pcl::PointIndices);
    pcl::PointIndices groundIndices;
    pcl::ExtractIndices<PCLPoint> resultIndices;

    cloudFilter.pointcloudFilteringByZ(srcPointcloud, -10.0f, 3.0f, inputIndices);
    typeToXYZIRL(srcPointcloud, inputIndices, inputCloud);
    extractGround(inputCloud, groundIndices);

    resultIndices.setInputCloud(srcPointcloud);
    resultIndices.setIndices(boost::make_shared<pcl::PointIndices>(groundIndices));
    resultIndices.setNegative(true);
    resultIndices.filter(*dstPointcloud);
    dstPointcloud->header = srcPointcloud->header;
}

void LidarPlaneFitGround::filteringGround(const pcl::PointCloud<PointXYZIRL>::Ptr &srcPointcloud, 
                                          pcl::PointCloud<PointXYZIRL>::Ptr &dstPointcloud)
{
    pcl::PointCloud<PointXYZIRL>::Ptr inputCloud(new pcl::PointCloud<PointXYZIRL>);
    pcl::PointIndices groundIndices;
    pcl::ExtractIndices<PointXYZIRL> resultIndices;

    for (size_t loop = 0; loop < srcPointcloud->points.size(); loop++)
    {
        if (srcPointcloud->points[loop].z >= -10.0f && srcPointcloud->points[loop].z <= 2.0f)
        {
            PointXYZIRL point;
            point.x = srcPointcloud->points[loop].x;
            point.y = srcPointcloud->points[loop].y;
            point.z = srcPointcloud->points[loop].z;
            point.intensity = srcPointcloud->points[loop].intensity;
            point.ring = srcPointcloud->points[loop].ring;
            point.index = static_cast<unsigned int>(loop);
            inputCloud->push_back(point);
        }
    }

    extractGround(inputCloud, groundIndices);

    resultIndices.setInputCloud(srcPointcloud);
    resultIndices.setIndices(boost::make_shared<pcl::PointIndices>(groundIndices));
    resultIndices.setNegative(true);
    resultIndices.filter(*dstPointcloud);
    dstPointcloud->header = srcPointcloud->header;
}

void LidarPlaneFitGround::extractGround(const pcl::PointCloud<PointXYZIRL>::Ptr &inputCloud, pcl::PointIndices &groundIndices)
{
    float distance = 0;
    std::sort(inputCloud->begin(), inputCloud->end(), point_cmp);
    numLPR = std::min(numLPR, static_cast<int>(inputCloud->size()));

    //Extract init ground seeds.
    extractInitialSeeds(inputCloud);
    groundCloud = seedsCloud;
    //Ground plane fitter mainloop
    for (int i = 0; i < numIter; i++)
    {
        distance = estimatePlane();
        groundCloud->clear();
        groundIndices.indices.clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(inputCloud->size(), 3);
        int j = 0;
        for (auto p : inputCloud->points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int r = 0; r < result.rows(); r++)
        {
            if (/*std::abs*/(result[r]) < distance)
            {
                groundCloud->points.push_back(inputCloud->points[r]);
                groundIndices.indices.push_back(inputCloud->points[r].index);
            }
        }
    }
}

float LidarPlaneFitGround::estimatePlane()
{
    float th_dist_d_ = 0;
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*groundCloud, cov, pc_mean);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d
    th_dist_d_ = thresholdDist - d_;
    return th_dist_d_;
}

void LidarPlaneFitGround::extractInitialSeeds(const pcl::PointCloud<PointXYZIRL>::Ptr &inputCloud)
{
    // LPR is the mean of low point representative
    float sum = 0;
    int count = 0;
    // Calculate the mean height value.
    for (size_t loop = 0; loop < numLPR; loop++)
    {
        sum += inputCloud->points[loop].z;
        count++;
    }

    //double lpr_height = count != 0 ? sum / count : 0; // in case divide by 0
    float lpr_height = inputCloud->points[count / 2].z;
    seedsCloud->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (size_t loop = 0; loop < inputCloud->size(); loop++)
    {
        if (inputCloud->points[loop].z < lpr_height + thresholdSeeds)
        {
            seedsCloud->points.push_back(inputCloud->points[loop]);
        }
    }
}

void LidarPlaneFitGround::typeToXYZIRL(const PCLPointCloud::Ptr &srcPointcloud, const pcl::PointIndices::Ptr &inputIndices,
                  pcl::PointCloud<PointXYZIRL>::Ptr &resultCloud)
{
    for (size_t loop = 0; loop < inputIndices->indices.size(); loop++)
    {
        const int index = static_cast<unsigned int>(inputIndices->indices[loop]);
        PointXYZIRL point;
        point.x = srcPointcloud->points[index].x;
        point.y = srcPointcloud->points[index].y;
        point.z = srcPointcloud->points[index].z;
        point.intensity = srcPointcloud->points[index].intensity;
        point.index = index;
        resultCloud->push_back(point);
    }
}

}  // namespace lidar
}  // namespace perception