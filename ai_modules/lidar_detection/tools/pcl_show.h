#pragma once

#include <vector>
#include <iostream>
#include "Eigen/Dense"
#include <pcl/visualization/cloud_viewer.h>

#include "base/object.h"
#include "base/object_types.h"
#include "base/point_cloud.h"
#include "common/pcl_util.h"
#include "proto/pointcloud.pb.h"
#include "proto/perception_obstacle.pb.h"

namespace perception {
namespace lidar {

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Pointcloud;

void typeToPCLCloud(const std::shared_ptr<drivers::PointCloud>& message, 
                    Pointcloud::Ptr &dstCloud);

void typeToPCLCloud(const base::PointFCloudPtr &cloud, 
                    Pointcloud::Ptr &dstCloud);                           

void pclToAttributePointCloud(const Pointcloud::Ptr &srcCloud, 
                              base::AttributePointCloud<base::PointF> &dstcloud);

Eigen::Quaternionf getRotation(float angularX, float angularY, float angularZ);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer, 
                      const Pointcloud::Ptr &srcCloud);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer, 
                      const Pointcloud::Ptr &srcCloud1,
                      const Pointcloud::Ptr &srcCloud2);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const std::vector<base::PolygonDType> &polygons);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const base::PolygonDType &polygon);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const std::vector<base::ObjectPtr> &objectList, int begin_index =0);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const std::vector<base::ObjectPtr> &objectList, const std::vector<base::ObjectPtr> &trackedObjects);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const std::shared_ptr<perception::PerceptionObstacles> &objectList);

}  // namespace lidar
}  // namespace perception