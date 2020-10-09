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

EuclideanCluster::EuclideanCluster(){
    // if distance >= 120m, ingnore
    seg_distance_ = {17, 38, 70, 120};
    cluster_distance_ = {0.4f, 0.7f, 1.0f, 1.5f};
}

EuclideanCluster::~EuclideanCluster(){

}

void EuclideanCluster::clustering(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>> &objects){
    PCLPointCloud::Ptr inputPointCloud(new PCLPointCloud);
    std::vector<PCLPointCloud::Ptr> segmentPointCloudArray;
    objects.clear();
    if(srcPointcloud->size() <= 0)
        return;
    //differenceNormalsSegmentation(srcPointcloud, inputPointCloud);
    //pcl::io::savePCDFileASCII("./temp.pcd", *inputPointCloud);
    clusterByDistance(srcPointcloud, seg_distance_, segmentPointCloudArray);
    for (size_t i = 0; i < segmentPointCloudArray.size(); i++)
    {
        if(segmentPointCloudArray[i]->size() > 0)
        {
            clusteringToSegment(segmentPointCloudArray[i], cluster_distance_[i], &objects);
        }
    }
}

void EuclideanCluster::clusteringToSegment(const PCLPointCloud::Ptr &srcPointCloud, const float maxClusterDistance,
                                           std::vector<std::shared_ptr<base::Object>>* objects)
{
    pcl::search::KdTree<PCLPoint>::Ptr tree(new pcl::search::KdTree<PCLPoint>);
    std::vector<pcl::PointIndices> localIndices;
    //create 2d cloud
    PCLPointCloud::Ptr cloud2D(new PCLPointCloud);
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
    pcl::copyPointCloud<PCLPoint, pcl::PointNormal>(*srcPointcloud, *diffnormals_cloud);

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
