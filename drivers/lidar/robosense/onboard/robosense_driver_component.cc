/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <thread>

//added by hezb for show process name
#include <stdio.h>
#include <stdlib.h>
#include <sys/prctl.h>
//added end

#include "common/util/message_util.h"
#include "lidar/robosense/onboard/robosense_driver_component.h"

namespace drivers {
namespace robosense {

bool RobosenseDriverComponent::Init() {
  AINFO << "Robosense driver component init";
  ::robosense::lidar::RSDriverParam driver_param;
  config::RobosenseConfig robosense_config;
  if (!GetProtoConfig(&robosense_config)) {
    AWARN << "Load config failed, config file" << config_file_path_;
    return false;
  }

  AINFO << "Robosense config: " << robosense_config.DebugString();
  driver_param.frame_id = "rslidar";
  driver_param.input_param.device_ip = robosense_config.device_ip();
  driver_param.input_param.msop_port = static_cast<unsigned short>(robosense_config.msop_port());
  driver_param.input_param.difop_port = static_cast<unsigned short>(robosense_config.difop_port());
  driver_param.lidar_type = driver_param.strToLidarType(robosense_config.lidar_type());
  
  writer_ =
      node_->CreateWriter<PointCloud>(robosense_config.output_channel_name());
  point_cloud_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
  point_cloud_pool_->ConstructAll();
  for (int i = 0; i < pool_size_; i++) {
    auto point_cloud = point_cloud_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "fail to getobject, i: " << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(140000);
  }

  driver_ptr_.reset(new ::robosense::lidar::LidarDriver<PointXYZI>());

  if (!driver_ptr_->init(driver_param)){
    AERROR << "Robosense Driver Initialize Error....";
    return false;
  }
  driver_ptr_->regExceptionCallback(std::bind(&RobosenseDriverComponent::LocalExceptionCallback, \
                                                this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&RobosenseDriverComponent::LidarCallback, this, std::placeholders::_1));

  driver_ptr_->start();
  config_ = robosense_config;

  return true;
}

void RobosenseDriverComponent::LidarCallback(const ::robosense::lidar::PointCloudMsg<PointXYZI>& msg){
  std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
  if (point_cloud_out == nullptr) {
    AWARN << "poin cloud pool return nullptr, will be create new.";
    point_cloud_out = std::make_shared<PointCloud>();
    point_cloud_out->mutable_point()->Reserve(140000);
  }
  if (point_cloud_out == nullptr) {
    AWARN << "point cloud out is nullptr";
    return;
  }
  common::util::FillHeader("robosense", point_cloud_out.get());
  point_cloud_out->Clear();
  point_cloud_out->mutable_header()->set_frame_id(config_.frame_id());
  point_cloud_out->mutable_header()->set_timestamp_sec(apollo::cyber::Time().Now().ToSecond());
  point_cloud_out->set_height(1);

  ConvertToPointcloud(msg, point_cloud_out);

  int size = point_cloud_out->point_size();
    
  if (size == 0) {
    // we discard this pointcloud if empty
      AERROR << "All points is NAN! Please check robosense:" << config_.lidar_type();
  } else {
    uint64_t ns_timestamp = static_cast<uint64_t>(msg.timestamp * 1e9);
    point_cloud_out->set_measurement_time(msg.timestamp);
    point_cloud_out->mutable_header()->set_lidar_timestamp(ns_timestamp);
  }
  point_cloud_out->set_width(point_cloud_out->point_size());

  if (point_cloud_out == nullptr || point_cloud_out->point_size() == 0) {
    AWARN << "point_cloud_out is empty.";
    return;
  }
  writer_->Write(point_cloud_out);
}

void RobosenseDriverComponent::LocalExceptionCallback(const ::robosense::lidar::Error& msg){
      switch (msg.error_code_type)
      {
        case ::robosense::lidar::ErrCodeType::INFO_CODE:
          AINFO << msg.toString();
          break;
        case ::robosense::lidar::ErrCodeType::WARNING_CODE:
          AWARN << msg.toString();
          break;
        case ::robosense::lidar::ErrCodeType::ERROR_CODE:
          AERROR << msg.toString();
          break;
      }
}

void RobosenseDriverComponent::ConvertToPointcloud(
    const ::robosense::lidar::PointCloudMsg<PointXYZI>& msg,
    std::shared_ptr<PointCloud> point_cloud){

  for (auto iter = msg.point_cloud_ptr->begin(); iter != msg.point_cloud_ptr->end(); ++iter){
    double dist = static_cast<double>(fabs(iter->y));
    if(dist < config_.min_range() || dist > config_.max_range())
      continue;
    PointXYZIT* point = point_cloud->add_point();
    point->set_x(iter->x);
    point->set_y(iter->y);
    point->set_z(iter->z);
    point->set_intensity(static_cast<unsigned short>(iter->intensity));
    point->set_timestamp(static_cast<uint64_t>(msg.timestamp * 1e9));
  }

  point_cloud->set_is_dense(true);
}


}  // namespace robosense
}  // namespace drivers
