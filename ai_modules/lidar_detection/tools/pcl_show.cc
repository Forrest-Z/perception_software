#include "lidar_detection/tools/pcl_show.h"

namespace perception {
namespace lidar {

void typeToPCLCloud(const std::shared_ptr<drivers::PointCloud> &message,
                    PCLPointCloud::Ptr &dstCloud) {
  dstCloud->clear();
  for (int i = 0; i < message->point_size(); ++i) {
    const drivers::PointXYZIT &pt = message->point(i);

    if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
      continue;
    }

    if (fabs(pt.x()) > 1000 || fabs(pt.y()) > 1000 || fabs(pt.z()) > 1000) {
      continue;
    }

    if (pt.z() > 5.0) {
      continue;
    }
    PCLPoint point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    point.intensity = static_cast<float>(pt.intensity());
    dstCloud->push_back(point);
  }
}

void typeToPCLCloud(const base::PointFCloudPtr &cloud,
                    PCLPointCloud::Ptr &dstCloud) {
  dstCloud->clear();
  for (size_t i = 0; i < cloud->size(); i++) {
    const auto &point = cloud->at(i);
    PCLPoint tempPoint;
    tempPoint.x = point.x;
    tempPoint.y = point.y;
    tempPoint.z = point.z;
    tempPoint.intensity = static_cast<float>(point.intensity);
    dstCloud->push_back(tempPoint);
  }
}

void pclToAttributePointCloud(
    const PCLPointCloud::Ptr &srcCloud,
    base::AttributePointCloud<base::PointF> &dstcloud) {
  dstcloud.clear();
  for (const auto point : *srcCloud) {
    base::PointF tempPoint;
    tempPoint.x = point.x;
    tempPoint.y = point.y;
    tempPoint.z = point.z;
    tempPoint.intensity = point.intensity;
    dstcloud.push_back(tempPoint);
  }
}

Eigen::Quaternionf getRotation(float angularX, float angularY, float angularZ) {
  float w = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  w = std::cos(angularX / 2) * std::cos(angularY / 2) * std::cos(angularZ / 2) +
      std::sin(angularX / 2) * std::sin(angularY / 2) * std::sin(angularZ / 2);
  x = std::sin(angularX / 2) * std::cos(angularY / 2) * std::cos(angularZ / 2) -
      std::cos(angularX / 2) * std::sin(angularY / 2) * std::sin(angularZ / 2);
  y = std::cos(angularX / 2) * std::sin(angularY / 2) * std::cos(angularZ / 2) +
      std::sin(angularX / 2) * std::cos(angularY / 2) * std::sin(angularZ / 2);
  z = std::cos(angularX / 2) * std::cos(angularY / 2) * std::sin(angularZ / 2) -
      std::sin(angularX / 2) * std::sin(angularY / 2) * std::cos(angularZ / 2);
  return Eigen::Quaternionf(w, x, y, z);
}

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const PCLPointCloud::Ptr &srcCloud) {
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerCustom<PCLPoint> colorHandler(
      srcCloud, 0, 100, 255);
  viewer->addPointCloud<PCLPoint>(srcCloud, colorHandler, "srcCloud", 0);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);
}

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const PCLPointCloud::Ptr &srcCloud1,
                      const PCLPointCloud::Ptr &srcCloud2) {
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerCustom<PCLPoint> colorHandler1(
      srcCloud1, 0, 100, 255);
  viewer->addPointCloud<PCLPoint>(srcCloud1, colorHandler1, "srcCloud1", 0);
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
  // 2, "srcCloud1", 0);
  pcl::visualization::PointCloudColorHandlerCustom<PCLPoint> colorHandler2(
      srcCloud2, 255, 0, 0);
  viewer->addPointCloud<PCLPoint>(srcCloud2, colorHandler2, "srcCloud2", 0);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud2", 0);
}

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const std::vector<base::PolygonDType> &polygons) {
  int loop = 0;
  // viewer->removeAllShapes();
  for (const auto &polygon : polygons) {
    size_t pointSize = polygon.size();
    for (size_t i = 0; i < pointSize; i++) {
      std::ostringstream idString;
      auto tempPoint1 = polygon[i];
      auto tempPoint2 = polygon[(i + 1) % pointSize];
      pcl::PointXYZ point1((float)tempPoint1.x, (float)tempPoint1.y, -1.5f);
      pcl::PointXYZ point2((float)tempPoint2.x, (float)tempPoint2.y, -1.5f);
      idString << "line" << loop;
      viewer->addLine<pcl::PointXYZ>(point1, point2, 255, 255, 0,
                                     idString.str());
      loop++;
    }
  }
}

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const base::PolygonDType &polygon, const int index) {
  int loop = 0;
  size_t pointSize = polygon.size();
  for (size_t i = 0; i < pointSize; i++) {
    std::ostringstream idString;
    auto tempPoint1 = polygon[i];
    auto tempPoint2 = polygon[(i + 1) % pointSize];
    pcl::PointXYZ point1((float)tempPoint1.x, (float)tempPoint1.y,
                         (float)tempPoint1.z);
    pcl::PointXYZ point2((float)tempPoint2.x, (float)tempPoint2.y,
                         (float)tempPoint2.z);
    idString << index << "line" << loop;
    viewer->addLine<pcl::PointXYZ>(point1, point2, 255, 255, 0, idString.str());
    loop++;
  }
}

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const std::vector<base::ObjectPtr> &objectList,
                      int begin_index) {
  int index = begin_index;
  // viewer->removeAllShapes();
  for (const auto &object : objectList) {
    Eigen::Quaternionf boxQ(1, 0, 0, 0);
    Eigen::Vector3f center;
    Eigen::Vector3f velocity;
    pcl::PointXYZI point;
    std::ostringstream text;
    std::ostringstream idString;
    std::ostringstream textidString;

    int red = 0;
    int green = 0;
    int blue = 0;
    boxQ = getRotation(0, 0, object->theta);
    center[0] = static_cast<float>(object->center[0]);
    center[1] = static_cast<float>(object->center[1]);
    center[2] = static_cast<float>(object->center[2] + +object->size[2] / 2.0);
    center[2] = -2.0;

    velocity[0] = static_cast<float>(object->velocity[0]);
    velocity[1] = static_cast<float>(object->velocity[1]);
    velocity[2] = 0;

    point.x = center[0];
    point.y = center[1];

    if (object->type == perception::base::ObjectType::UNKNOWN) {
      text << " y:" << center[1] << std::setprecision(2) << " class: UNKNOWN";
      red = 255;
      green = 0;
      blue = 255;
    } else if (object->type ==
               perception::base::ObjectType::UNKNOWN_MOVABLE) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_MOVABLE";
      red = 0;
      green = 0;
      blue = 255;
    } else if (object->type ==
               perception::base::ObjectType::UNKNOWN_UNMOVABLE) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_UNMOVABLE";
      red = 0;
      green = 255;
      blue = 255;
    } else if (object->type ==
               perception::base::ObjectType::PEDESTRIAN) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: PEDESTRIAN";
      red = 255;
      green = 0;
      blue = 0;
    } else if (object->type == perception::base::ObjectType::BICYCLE) {
      text << " y:" << center[1] << std::setprecision(2) << " class: BICYCLE";
      red = 255;
      green = 255;
      blue = 0;
    } else if (object->type == perception::base::ObjectType::VEHICLE) {
      text << " y:" << center[1] << std::setprecision(2) << " class: VEHICLE";
      red = 255;
      green = 255;
      blue = 255;
    }
    idString << "object" << index;
    textidString << "text" << index;
    index++;

    std::stringstream velocity_text;
    std::stringstream location_text;

    velocity_text << "(" << std::setprecision(2) << velocity[0] << ","
                  << std::setprecision(2) << velocity[1] << ")"
                  << " track_id:" << object->track_id;
    location_text << "(" << std::setprecision(2) << center[0] << ","
                  << std::setprecision(2) << center[1] << ")"
                  << " track_id:" << object->track_id;
    // Show what color you want to display according to your needs
    // red:radar_front, green:radar_left_front, magenta:radar_right_front
    // white:radar_rear, yellow:radar_left_rear, cyan:radar_right_rear,
    // yellow:fused radar objects
    int cortable[8][3] = {{255, 0, 0},     {0, 255, 0},   {255, 0, 255},
                          {255, 255, 255}, {255, 255, 0}, {0, 255, 255},
                          {255, 255, 0},   {255, 0, 0}};

    int table_index = begin_index / 500;
    int R = cortable[table_index][0];
    int G = cortable[table_index][1];
    int B = cortable[table_index][2];
    viewer->addText3D(velocity_text.str(), point, 0.3, R, G, B,
                      textidString.str());
    viewer->addCube(center, boxQ, object->size[0], object->size[1],
                    object->size[2], idString.str());
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue,
        idString.str());

    // updateViewerData(viewer, object->polygon, index);
  }
}

void updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const std::vector<base::ObjectPtr> &objectList,
                      const std::vector<base::ObjectPtr> &trackedObjects) {
  int index = 0;
  viewer->removeAllShapes();
  // std::cout << "show object count:" << objectList->size() << std::endl;
  for (const auto &object : objectList) {
    Eigen::Quaternionf boxQ(1, 0, 0, 0);
    Eigen::Vector3f center;
    pcl::PointXYZI point;
    std::ostringstream text;
    std::ostringstream idString;
    int red = 0;
    int green = 0;
    int blue = 0;
    boxQ = getRotation(0, 0, object->theta);
    center[0] = static_cast<float>(object->center[0]);
    center[1] = static_cast<float>(object->center[1]);
    center[2] = static_cast<float>(object->center[2] + object->size[2] / 2.0);
    point.x = center[0];
    point.y = center[1];
    point.z = center[2];
    if (object->type == perception::base::ObjectType::UNKNOWN) {
      text << " y:" << center[1] << std::setprecision(2) << " class: UNKNOWN";
      red = 255;
      green = 0;
      blue = 255;
    } else if (object->type ==
               perception::base::ObjectType::UNKNOWN_MOVABLE) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_MOVABLE";
      red = 0;
      green = 0;
      blue = 255;
    } else if (object->type ==
               perception::base::ObjectType::UNKNOWN_UNMOVABLE) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_UNMOVABLE";
      red = 0;
      green = 255;
      blue = 255;
    } else if (object->type ==
               perception::base::ObjectType::PEDESTRIAN) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: PEDESTRIAN";
      red = 255;
      green = 0;
      blue = 0;
    } else if (object->type == perception::base::ObjectType::BICYCLE) {
      text << " y:" << center[1] << std::setprecision(2) << " class: BICYCLE";
      red = 255;
      green = 255;
      blue = 0;
    } else if (object->type == perception::base::ObjectType::VEHICLE) {
      text << " y:" << center[1] << std::setprecision(2) << " class: VEHICLE";
      red = 255;
      green = 255;
      blue = 255;
    }
    // idString << "object" << index;
    // index++;
    // viewer->addText3D<pcl::PointXYZI>(text.str(), point, 0.3, 255, 255, 255,
    // idString.str());
    idString << "object" << index;
    // std::cout << idString.str() << std::endl;
    index++;
    viewer->addCube(center, boxQ, object->size[0], object->size[1],
                    object->size[2], idString.str());
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue,
        idString.str());
  }

  for (const auto &object : trackedObjects) {
    Eigen::Quaternionf boxQ(1, 0, 0, 0);
    Eigen::Vector3f center;
    pcl::PointXYZI point;
    std::ostringstream text;
    std::ostringstream idString;
    int red = 255;
    int green = 255;
    int blue = 0;
    boxQ = getRotation(0, 0, object->theta);
    center[0] = static_cast<float>(object->center[0]);
    center[1] = static_cast<float>(object->center[1]);
    center[2] = static_cast<float>(object->center[2] + object->size[2] / 2.0);
    point.x = center[0];
    point.y = center[1];
    point.z = center[2];
    if (object->type == perception::base::ObjectType::UNKNOWN) {
      text << object->id << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN";
    } else if (object->type ==
               perception::base::ObjectType::UNKNOWN_MOVABLE) {
      text << object->id << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_MOVABLE";
    } else if (object->type ==
               perception::base::ObjectType::UNKNOWN_UNMOVABLE) {
      text << object->id << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_UNMOVABLE";
    } else if (object->type ==
               perception::base::ObjectType::PEDESTRIAN) {
      text << object->id << " y:" << center[1] << std::setprecision(2)
           << " class: PEDESTRIAN";
    } else if (object->type == perception::base::ObjectType::BICYCLE) {
      text << object->id << " y:" << center[1] << std::setprecision(2)
           << " class: BICYCLE";
    } else if (object->type == perception::base::ObjectType::VEHICLE) {
      text << object->id << " y:" << center[1] << std::setprecision(2)
           << " class: VEHICLE";
    }
    // idString << "track" << index;
    // index++;
    // viewer->addText3D<pcl::PointXYZI>(text.str(), point, 0.3, 255, 255, 255,
    // idString.str());
    idString << "track" << index;
    // std::cout << idString.str() << std::endl;
    index++;
    viewer->addCube(center, boxQ, object->size[0], object->size[1],
                    object->size[2], idString.str());
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue,
        idString.str());
  }
}

void updateViewerData(
    const pcl::visualization::PCLVisualizer::Ptr viewer,
    const std::shared_ptr<perception::PerceptionObstacles>
        &objectList) {
  const int count = objectList->perception_obstacle_size();
  viewer->removeAllShapes();
  std::cout << "show object count:" << count << std::endl;
  for (int index = 0; index < count; index++) {
    Eigen::Quaternionf boxQ(1, 0, 0, 0);
    Eigen::Vector3f center;
    Eigen::Vector3d temp;
    Eigen::Vector3d transformCenter;
    pcl::PointXYZI point;
    std::ostringstream text;
    std::ostringstream idString;
    int red = 0;
    int green = 0;
    int blue = 0;
    perception::PerceptionObstacle *object =
        objectList->mutable_perception_obstacle(index);
    boxQ = getRotation(0, 0, static_cast<float>(object->theta()));
    center[0] = static_cast<float>((object->position()).x());
    center[1] = static_cast<float>((object->position()).y());
    center[2] =
        static_cast<float>((object->position()).z() + object->height() / 2.0);
    point.x = center[0];
    point.y = center[1];
    point.z = center[2];
    if (object->type() == 0) {
      text << " y:" << center[1] << std::setprecision(2) << " class: UNKNOWN";
      red = 255;
      green = 0;
      blue = 255;
    } else if (object->type() == 1) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_MOVABLE";
      red = 0;
      green = 0;
      blue = 255;
    } else if (object->type() == 2) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: UNKNOWN_UNMOVABLE";
      red = 0;
      green = 255;
      blue = 255;
    } else if (object->type() == 3) {
      text << " y:" << center[1] << std::setprecision(2)
           << " class: PEDESTRIAN";
      red = 255;
      green = 0;
      blue = 0;
    } else if (object->type() == 4) {
      text << " y:" << center[1] << std::setprecision(2) << " class: BICYCLE";
      red = 255;
      green = 255;
      blue = 0;
    } else if (object->type() == 5) {
      text << " y:" << center[1] << std::setprecision(2) << " class: VEHICLE";
      red = 255;
      green = 255;
      blue = 255;
    }
    // idString << "object" << index;
    // index++;
    // viewer->addText3D<pcl::PointXYZI>(text.str(), point, 0.3, 255, 255, 255,
    // idString.str());
    idString << "object" << index;
    viewer->addCube(center, boxQ, object->length(), object->width(),
                    object->height(), idString.str());
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue,
        idString.str());
  }
}

}  // namespace lidar
}  // namespace perception