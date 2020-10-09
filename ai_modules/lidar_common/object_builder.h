#pragma once

#include <memory>
#include <string>
#include <vector>

#include "base/object.h"
#include "base/point.h"
#include "base/point_cloud.h"

namespace perception {
namespace lidar {

struct ObjectBuilderInitOptions {};

struct ObjectBuilderOptions {
  Eigen::Vector3d ref_center = Eigen::Vector3d(0, 0, 0);
};

class ObjectBuilder {
 public:
  ObjectBuilder() = default;
  ~ObjectBuilder() = default;

  // @brief: initialization. Get orientation estimator instance.
  // @param [in]: ObjectBuilderInitOptions.
  bool Init(
      const ObjectBuilderInitOptions& options = ObjectBuilderInitOptions());

  bool Build(const ObjectBuilderOptions& options, const double timestamp,\
             std::vector<std::shared_ptr<base::Object>> &objectList);

  std::string Name() const { return "ObjectBuilder"; }

 private:
  // @brief: calculate 2d polygon.
  //         and fill the convex hull vertices in object->polygon.
  // @param [in/out]: ObjectPtr.
  void ComputePolygon2D(
      std::shared_ptr<perception::base::Object> object);

  // @brief: calculate the size, center of polygon.
  // @param [in/out]: ObjectPtr.
  void ComputePolygonSizeCenter(
      std::shared_ptr<perception::base::Object> object);

  // @brief: calculate and fill timestamp and anchor_point.
  // @param [in/out]: ObjectPtr.
  void ComputeOtherObjectInformation(
      std::shared_ptr<perception::base::Object> object);

  // @brief: calculate and fill default polygon value.
  // @param [in]: min and max point.
  // @param [in/out]: ObjectPtr.
  void SetDefaultValue(
      const Eigen::Vector3f& min_pt,
      const Eigen::Vector3f& max_pt,
      std::shared_ptr<perception::base::Object> object);

  // @brief: decide whether input cloud is on the same line.
  //         if ture, add perturbation.
  // @param [in/out]: pointcloud.
  // @param [out]: is line: true, not line: false.
  bool LinePerturbation(perception::base::PointCloud<
           perception::base::PointF>* cloud);

  // @brief: calculate 3D min max point
  // @param [in]: point cloud.
  // @param [in/out]: min and max points.
  void GetMinMax3D(
      const perception::base::PointCloud<
          perception::base::PointF>& cloud,
      Eigen::Vector3f* min_pt,
      Eigen::Vector3f* max_pt);

  void reconstructPolygon3d(std::shared_ptr<perception::base::Object> obj);

  double computeAreaAlongOneEdge(std::shared_ptr<perception::base::Object> obj,
                                size_t first_in_point,
                                Eigen::Vector3f *center,
                                double *length,
                                double *width,
                                Eigen::Vector3f *dir);
};  // class ObjectBuilder

}  // namespace lidar
}  // namespace perception
