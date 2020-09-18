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

#include <atomic>
#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "proto/pointcloud.pb.h"
#include "proto/lidar_component_config.pb.h"
#include "proto/perception_obstacle.pb.h"
#include "lidar_detection/app/point_pillars_detection.h"

namespace perception {
namespace onboard {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class LidarDetectionComponent : public apollo::cyber::Component<drivers::PointCloud> {
 public:
  LidarDetectionComponent() = default;
  virtual ~LidarDetectionComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::PointCloud>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(
      const std::shared_ptr<const drivers::PointCloud>& in_message,
      const std::shared_ptr<PerceptionObstacles> &out_message);
  
  bool Preprocess(
      const std::shared_ptr<drivers::PointCloud const>& message,
      std::shared_ptr<base::AttributePointCloud<base::PointF>> &cloud) const;

  bool ConvertObjectToPb(const base::ObjectPtr &object_ptr,
                         PerceptionObstacle *pb_msg);

 private:
  static std::atomic<uint32_t> seq_num_;
  std::string sensor_name_;
  float lidar_query_tf_offset_ = 20.0f;
  std::string lidar2novatel_tf2_child_frame_id_;
  std::string output_channel_name_;
  std::unique_ptr<lidar::PointPillarsDetection> detector_;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
};

CYBER_REGISTER_COMPONENT(LidarDetectionComponent);

}  // namespace onboard
}  // namespace perception
