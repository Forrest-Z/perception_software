/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "base/object.h"
#include "base/point_cloud.h"
#include "lidar_detection/lidar_point_pillars/point_pillars.h"


namespace perception {
namespace lidar {

struct DetectionInitOptions {};

struct DetectionOptions {};

class PointPillarsDetection {
 public:
  PointPillarsDetection();
  ~PointPillarsDetection() = default;

  bool Init(const DetectionInitOptions& options = DetectionInitOptions());

  bool Detect(const DetectionOptions& options, 
              const std::shared_ptr<base::AttributePointCloud<base::PointF>> &original_cloud,
              std::vector<std::shared_ptr<base::Object>> &segmented_objects);

  std::string Name() const { return "PointPillarsDetection"; }

 private:
  void CloudToArray(const base::PointFCloudPtr& pc_ptr,
                    float* out_points_array,
                    float normalizing_factor);

  void GetObjects(std::vector<std::shared_ptr<base::Object>>* objects,
                  const Eigen::Affine3d& pose, std::vector<float>* detections,
                  std::vector<int>* labels);

  base::ObjectSubType GetObjectSubType(int label);

  // PointPillars
  std::unique_ptr<apollo::perception::lidar::PointPillars> point_pillars_ptr_;
  base::PointFCloudPtr cur_cloud_ptr_;

  // point cloud range
  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;
  float z_min_;
  float z_max_;

  // time statistics
  double cloud_to_array_time_ = 0.0;
  double inference_time_ = 0.0;
  double collect_time_ = 0.0;
};  // class PointPillarsDetection

}  // namespace lidar
}  // namespace perception

