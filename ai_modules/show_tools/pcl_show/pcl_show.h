#pragma once

#include <vector>
#include <iostream>
#include "Eigen/Dense"
#include <pcl/visualization/cloud_viewer.h>

#include "proto/pointcloud.pb.h"
#include "proto/perception_obstacle.pb.h"

#include "base/object.h"
#include "base/object_types.h"
#include "base/point_cloud.h"
#include "common/lidar_processing/pcl_util.h"

namespace perception {
namespace lidar {

void typeToPCLCloud(const std::shared_ptr<drivers::PointCloud>& message, 
                    PCLPointCloud::Ptr &dstCloud);

Eigen::Quaternionf getRotation(float angularX, float angularY, float angularZ);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer, 
                      const PCLPointCloud::Ptr &srcCloud);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer, 
                      const PCLPointCloud::Ptr &srcCloud1,
                      const PCLPointCloud::Ptr &srcCloud2);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const std::vector<base::PolygonDType> &polygons);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const base::PolygonDType &polygon);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const std::vector<base::ObjectPtr> &objectList, int begin_index=0);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const std::vector<base::ObjectPtr> &objectList, const std::vector<base::ObjectPtr> &trackedObjects);

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                          const std::shared_ptr<perception::PerceptionObstacles> &objectList);

}  // namespace lidar
}  // namespace perception