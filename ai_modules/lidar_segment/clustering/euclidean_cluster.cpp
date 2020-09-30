#include "lidar_segment/clustering/euclidean_cluster.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <cmath>

#define MIN_CLUSTER_SIZE (9)
#define MAX_CLUSTER_SIZE (5000)

namespace perception {
namespace lidar {

EuclideanCluster::EuclideanCluster()
{
    seg_distance_ = {17, 38, 70};
    cluster_distance_ = {0.4f, 0.7f, 1.0f, 1.5f};
}

EuclideanCluster::~EuclideanCluster()
{

}

void EuclideanCluster::clustering(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>> &objects)
{
    PCLPointCloud::Ptr inputPointCloud(new PCLPointCloud);
    objects.clear();
    if(srcPointcloud->size() <= 0)
        return;
    //differenceNormalsSegmentation(srcPointcloud, inputPointCloud);
    //pcl::io::savePCDFileASCII("./temp.pcd", *inputPointCloud);
    clusterByDistance(srcPointcloud, objects);
}

void EuclideanCluster::clusterByDistance(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>>* objects)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered
    std::vector<PCLPointCloud::Ptr> segmentPointCloudArray(4);
    objects->clear();

    for (size_t i = 0; i < segmentPointCloudArray.size(); i++)
    {
        PCLPointCloud::Ptr temp(new PCLPointCloud);
        temp->header = srcPointcloud->header;
        segmentPointCloudArray[i] = temp;
    }

    for (size_t i = 0; i < srcPointcloud->points.size(); i++)
    {
        PCLPoint current_point;
        current_point.x = srcPointcloud->points[i].x;
        current_point.y = srcPointcloud->points[i].y;
        current_point.z = srcPointcloud->points[i].z;

        float origin_distance = std::sqrt(std::pow(current_point.x, 2.0f) + std::pow(current_point.y, 2.0f));

        // if distance > 120m, ingnore
        if (origin_distance >= 120)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0])
        {
            segmentPointCloudArray[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segmentPointCloudArray[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segmentPointCloudArray[2]->points.push_back(current_point);
        }
        else
        {
            segmentPointCloudArray[3]->points.push_back(current_point);
        }
    }

    for (size_t i = 0; i < segmentPointCloudArray.size(); i++)
    {
        if(segmentPointCloudArray[i]->size() > 0)
        {
            clusteringToSegment(segmentPointCloudArray[i], cluster_distance_[i], objects);
        }
    }
}

void EuclideanCluster::clusteringToSegment(const PCLPointCloud::Ptr &srcPointCloud, const float maxClusterDistance,
                                           std::vector<std::shared_ptr<base::Object>>* objects)
{
    pcl::search::KdTree<PCLPoint>::Ptr tree(new pcl::search::KdTree<Point>);
    std::vector<pcl::PointIndices> localIndices;
    //create 2d cloud
    PCLPointCloud::Ptr cloud2D(new Pointcloud);
    pcl::copyPointCloud(*srcPointCloud, *cloud2D);
    for (size_t i = 0; i < cloud2D->points.size(); i++)
    {
        cloud2D->points[i].z = 0;
        cloud2D->points[i].intensity = 0;
    }

    if (cloud2D->points.size() > 0)
        tree->setInputCloud(cloud2D);

    pcl::EuclideanClusterExtraction<PCLPoint> euclid;
    euclid.setInputCloud(cloud2D);
    euclid.setClusterTolerance(static_cast<double>(maxClusterDistance));
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(localIndices);

    computeCentroid(srcPointCloud, localIndices, objects);
}

void EuclideanCluster::computeCentroid(const PCLPointCloud::Ptr &srcPointCloud,
                     const std::vector<pcl::PointIndices> &localIndices,
                     std::vector<std::shared_ptr<base::Object>>* objects)
{
    for (size_t i = 0; i < localIndices.size(); i++)
    {
        base::ObjectPtr object(new base::Object);
        PCLPointCloud::Ptr tempCloud(new PCLPointCloud);
        double tempX = 0;
        double tempY = 0;
        double tempZ = 0;
        size_t count = localIndices[i].indices.size();
        tempCloud->header = srcPointCloud->header;
        for (auto iter = localIndices[i].indices.begin(); iter != localIndices[i].indices.end(); ++iter)
        {
            tempX += srcPointCloud->points[*iter].x;
            tempY += srcPointCloud->points[*iter].y;
            tempZ += srcPointCloud->points[*iter].z;
            tempCloud->push_back(srcPointCloud->points[*iter]);
        }
        object->anchor_point[0] = tempX / static_cast<double>(count);
        object->anchor_point[1] = tempY / static_cast<double>(count);
        object->anchor_point[2] = tempZ / static_cast<double>(count);
        object->lidar_supplement.num_points_in_roi = count;
        pclToAttributePointCloud(tempCloud, object->lidar_supplement.cloud);
        objects->push_back(object);
    }
}

void EuclideanCluster::differenceNormalsSegmentation(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &result)
{
    double small_scale = 0.5;
    double large_scale = 2.0;
    double angle_threshold = 0.5;
    pcl::search::Search<PCLPoint>::Ptr tree;
    if (srcPointcloud->isOrganized())
    {
        tree.reset(new pcl::search::OrganizedNeighbor<PCLPoint>());
    }
    else
    {
        tree.reset(new pcl::search::KdTree<PCLPoint>(false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud(srcPointcloud);

    pcl::NormalEstimationOMP<PCLPoint, pcl::PointNormal> normal_estimation;
    //pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    normal_estimation.setInputCloud(srcPointcloud);
    normal_estimation.setSearchMethod(tree);

    normal_estimation.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                   std::numeric_limits<float>::max());

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

    normal_estimation.setRadiusSearch(small_scale);
    normal_estimation.compute(*normals_small_scale);

    normal_estimation.setRadiusSearch(large_scale);
    normal_estimation.compute(*normals_large_scale);

    pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud<Point, pcl::PointNormal>(*srcPointcloud, *diffnormals_cloud);

    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PCLPoint, pcl::PointNormal, pcl::PointNormal> diffnormals_estimator;
    diffnormals_estimator.setInputCloud(srcPointcloud);
    diffnormals_estimator.setNormalScaleLarge(normals_large_scale);
    diffnormals_estimator.setNormalScaleSmall(normals_small_scale);

    diffnormals_estimator.initCompute();

    diffnormals_estimator.computeFeature(*diffnormals_cloud);

    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
                                  new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, angle_threshold)));
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
    cond_removal.setCondition(range_cond);
    cond_removal.setInputCloud(diffnormals_cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    cond_removal.filter(*diffnormals_cloud_filtered);

    pcl::copyPointCloud<pcl::PointNormal, PCLPoint>(*diffnormals_cloud, *result);
    result->header = srcPointcloud->header;
}

}  // namespace lidar
}  // namespace perception
