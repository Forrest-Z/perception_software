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
#include "lidar_detection/component/lidar_detection_component.h"

#include "cyber/time/time.h"
#include "base/object_pool_types.h"
#include "common/lidar_error_code.h"

namespace perception {
namespace onboard {

constexpr float kFloatMax = std::numeric_limits<float>::max();

std::atomic<uint32_t> LidarDetectionComponent::seq_num_{0};

bool LidarDetectionComponent::Init() {
  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  ADEBUG << "Lidar Component Configs: " << comp_config.DebugString();
  output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  lidar2novatel_tf2_child_frame_id_ =
      comp_config.lidar2novatel_tf2_child_frame_id();
  lidar_query_tf_offset_ =
      static_cast<float>(comp_config.lidar_query_tf_offset());
  writer_ = node_->CreateWriter<PerceptionObstacles>(output_channel_name_);
  
  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init detection component algorithm plugin.";
    return false;
  }
  return true;
}

bool LidarDetectionComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& message) {
  AINFO << std::setprecision(16)
        << "Enter detection component, message timestamp: "
        << message->measurement_time()
        << " current timestamp: " << apollo::cyber::Time::Now().ToSecond();
  std::shared_ptr<PerceptionObstacles> out_message(new (std::nothrow)
                                                       PerceptionObstacles);
  bool status = InternalProc(message, out_message);
  if (status) {
    writer_->Write(out_message);
    AINFO << "Send lidar detect output message.";
  }
  return status;
}

bool LidarDetectionComponent::InitAlgorithmPlugin() {
   detector_.reset(new lidar::PointPillarsDetection);
  // detector_.reset(
  //    BaseSegmentationRegisterer::GetInstanceByName(segmentor_name_));
  CHECK_NOTNULL(detector_.get());
  lidar::DetectionInitOptions detection_init_options;
  // segmentation_init_options.sensor_name = sensor_name;
  ACHECK(detector_->Init(detection_init_options));
  return true;
}

bool LidarDetectionComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<PerceptionObstacles> &out_message) {
  std::vector<std::shared_ptr<base::Object>> result_objects;
  std::shared_ptr<base::AttributePointCloud<base::PointF>> cloud = nullptr;
  cloud = base::PointFCloudPool::Instance().Get();
  if (Preprocess(in_message, cloud)) {
    return false;
  }
  uint32_t seq_num = seq_num_.fetch_add(1);
  const double timestamp = in_message->measurement_time();
  const double cur_time = apollo::cyber::Time::Now().ToSecond();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << std::setprecision(16) << "FRAME_STATISTICS:Lidar:Start:msg_time["
        << timestamp << "]:sensor[" << sensor_name_ << "]:cur_time[" << cur_time
        << "]:cur_latency[" << start_latency << "]";

  double publish_time = apollo::cyber::Time::Now().ToSecond();
  ::common::Header* header = out_message->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_obstacle");
  header->set_sequence_num(seq_num);
  header->set_lidar_timestamp(in_message->header().lidar_timestamp());
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  lidar::DetectionOptions detection_options;
  if (!detector_->Detect(detection_options, cloud, result_objects)) {
    AERROR << "Lidar detection process error ";
    return false;
  }

  out_message->set_error_code(common::ErrorCode::OK);
  for (const auto &obj : result_objects) {
    PerceptionObstacle *obstacle = out_message->add_perception_obstacle();
    if (!ConvertObjectToPb(obj, obstacle)) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return false;
    }
  }
  
  return true;
}

bool LidarDetectionComponent::Preprocess(
    const std::shared_ptr<drivers::PointCloud const>& message,
      std::shared_ptr<base::AttributePointCloud<base::PointF>> &cloud) const {
  if (cloud == nullptr) {
    cloud = base::PointFCloudPool::Instance().Get();
  }
  cloud->set_timestamp(message->measurement_time());
  if (message->point_size() > 0) {
    cloud->reserve(message->point_size());
    base::PointF point;
    for (int i = 0; i < message->point_size(); ++i) {
      const drivers::PointXYZIT& pt = message->point(i);
      if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
          continue;
      }
      if (fabs(pt.x()) > 100 ||
          fabs(pt.y()) > 100 ||
          fabs(pt.z()) > 100) {
          continue;
      }
      if (pt.z() > 5) {
        continue;
      }
      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();
      point.intensity = static_cast<float>(pt.intensity());
      cloud->push_back(point, static_cast<double>(pt.timestamp()) * 1e-9,
                              std::numeric_limits<float>::max(), i, 0);
    }
  }
  return true;
}

bool LidarDetectionComponent::ConvertObjectToPb(const base::ObjectPtr &object_ptr,
                                                PerceptionObstacle *pb_msg) {
  if (object_ptr == nullptr || pb_msg == nullptr) {
    return false;
  }

  pb_msg->set_id(object_ptr->track_id);
  pb_msg->set_theta(object_ptr->theta);

  common::Point3D *obj_center = pb_msg->mutable_position();
  obj_center->set_x(object_ptr->center(0));
  obj_center->set_y(object_ptr->center(1));
  obj_center->set_z(object_ptr->center(2));

  common::Point3D *obj_velocity = pb_msg->mutable_velocity();
  obj_velocity->set_x(object_ptr->velocity(0));
  obj_velocity->set_y(object_ptr->velocity(1));
  obj_velocity->set_z(object_ptr->velocity(2));

  common::Point3D *obj_acceleration = pb_msg->mutable_acceleration();
  obj_acceleration->set_x(object_ptr->acceleration(0));
  obj_acceleration->set_y(object_ptr->acceleration(1));
  obj_acceleration->set_z(object_ptr->acceleration(2));

  pb_msg->set_length(object_ptr->size(0));
  pb_msg->set_width(object_ptr->size(1));
  pb_msg->set_height(object_ptr->size(2));

  for (size_t i = 0; i < object_ptr->polygon.size(); ++i) {
    auto &pt = object_ptr->polygon.at(i);
    common::Point3D *p = pb_msg->add_polygon_point();
    p->set_x(pt.x);
    p->set_y(pt.y);
    p->set_z(pt.z);
  }

  common::Point3D *obj_anchor_point = pb_msg->mutable_anchor_point();
  obj_anchor_point->set_x(object_ptr->anchor_point(0));
  obj_anchor_point->set_y(object_ptr->anchor_point(1));
  obj_anchor_point->set_z(object_ptr->anchor_point(2));

  BBox2D *obj_bbox2d = pb_msg->mutable_bbox2d();
  const base::BBox2DF &box = object_ptr->camera_supplement.box;
  obj_bbox2d->set_xmin(box.xmin);
  obj_bbox2d->set_ymin(box.ymin);
  obj_bbox2d->set_xmax(box.xmax);
  obj_bbox2d->set_ymax(box.ymax);

  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      pb_msg->add_position_covariance(object_ptr->center_uncertainty(i, j));
      pb_msg->add_velocity_covariance(object_ptr->velocity_uncertainty(i, j));
      pb_msg->add_acceleration_covariance(
          object_ptr->acceleration_uncertainty(i, j));
    }
  }

  pb_msg->set_tracking_time(object_ptr->tracking_time);
  pb_msg->set_type(static_cast<PerceptionObstacle::Type>(object_ptr->type));
  pb_msg->set_sub_type(
      static_cast<PerceptionObstacle::SubType>(object_ptr->sub_type));
  pb_msg->set_timestamp(object_ptr->latest_tracked_time);  // in seconds.

  if (object_ptr->lidar_supplement.height_above_ground != kFloatMax) {
    pb_msg->set_height_above_ground(
        object_ptr->lidar_supplement.height_above_ground);
  } else {
    pb_msg->set_height_above_ground(std::numeric_limits<double>::quiet_NaN());
  }

  if (object_ptr->type == base::ObjectType::VEHICLE) {
    LightStatus *light_status = pb_msg->mutable_light_status();
    const base::CarLight &car_light = object_ptr->car_light;
    light_status->set_brake_visible(car_light.brake_visible);
    light_status->set_brake_switch_on(car_light.brake_switch_on);

    light_status->set_left_turn_visible(car_light.left_turn_visible);
    light_status->set_left_turn_switch_on(car_light.left_turn_switch_on);

    light_status->set_right_turn_visible(car_light.right_turn_visible);
    light_status->set_right_turn_switch_on(car_light.right_turn_switch_on);
  }

  return true;
}

}  // namespace onboard
}  // namespace perception
