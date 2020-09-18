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
#include "lidar/pandar/onboard/pandarsdk_driver_component.h"

namespace drivers {
namespace pandar {

bool PandarSDKDriverComponent::Init() {
  AINFO << "PandarSDK driver component init";
  config::PandarConfig pandar_config;
  if (!GetProtoConfig(&pandar_config)) {
    AWARN << "Load config failed, config file" << config_file_path_;
    return false;
  }

  AINFO << "PandarSDK config: " << pandar_config.DebugString();
  const unsigned short lidar_port = static_cast<unsigned short>(pandar_config.firing_data_port());
  switch (pandar_config.model()) {
    case PANDAR64: {
      pandar_sdk_.reset(new pandar_sdk::PandarGeneralSDK("", lidar_port, lidar_port,
                                                     boost::bind(&PandarSDKDriverComponent::LidarCallback, this, _1, _2),
                                                     NULL, 0, 0, 0, "Pandar64"));
      break;
    }
    case PANDAR40P: {
      pandar_sdk_.reset(new pandar_sdk::PandarGeneralSDK("", lidar_port, lidar_port,
                                                     boost::bind(&PandarSDKDriverComponent::LidarCallback, this, _1, _2),
                                                     NULL, 0, 0, 0, "Pandar40P"));
      break;
    }
    default:
      AERROR << "invalid model:" << pandar_config.model();
      break;
  }
  
  writer_ =
      node_->CreateWriter<PointCloud>(pandar_config.convert_channel_name());
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
  if (pandar_sdk_ != nullptr) {
      pandar_sdk_->Start();
      // pandar_sdk_->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
  } else {
    AWARN << "create pandar sdk fail";
  }
  // start the driver
  config_ = pandar_config;

  return true;
}

void PandarSDKDriverComponent::LidarCallback(boost::shared_ptr<pandar_sdk::PPointCloud> cld, double timestamp){
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
  common::util::FillHeader("pandarsdk", point_cloud_out.get());
  point_cloud_out->Clear();
  point_cloud_out->mutable_header()->set_frame_id(config_.frame_id());
  point_cloud_out->mutable_header()->set_timestamp_sec(apollo::cyber::Time().Now().ToSecond());
  point_cloud_out->set_height(1);

  ConvertPclToPointcloud(cld, point_cloud_out);

  int size = point_cloud_out->point_size();
    
  if (size == 0) {
    // we discard this pointcloud if empty
      AERROR << "All points is NAN! Please check pandar:" << config_.model();
  } else {
    uint64_t ns_timestamp = static_cast<uint64_t>(timestamp * 1e9);
    point_cloud_out->set_measurement_time(timestamp);
    point_cloud_out->mutable_header()->set_lidar_timestamp(ns_timestamp);
  }
  point_cloud_out->set_width(point_cloud_out->point_size());

  if (point_cloud_out == nullptr || point_cloud_out->point_size() == 0) {
    AWARN << "point_cloud_out is empty.";
    return;
  }
  writer_->Write(point_cloud_out);
}

void PandarSDKDriverComponent::ConvertPclToPointcloud(
    const boost::shared_ptr<pandar_sdk::PPointCloud> &cld_msg,
    std::shared_ptr<PointCloud> point_cloud){

  for (size_t i = 0; i < cld_msg->size(); ++i){
    double dist = static_cast<double>(fabs(cld_msg->points[i].y));
    if(dist < config_.min_range() || dist > config_.max_range())
      continue;
    PointXYZIT* point = point_cloud->add_point();
    point->set_x(cld_msg->points[i].x);
    point->set_y(cld_msg->points[i].y);
    point->set_z(cld_msg->points[i].z);
    point->set_intensity(static_cast<unsigned short>(cld_msg->points[i].intensity));
    point->set_timestamp(static_cast<uint64_t>(cld_msg->points[i].timestamp * 1e9));
  }

  if (config_.organized()) {
    point_cloud->set_is_dense(false);
  } else {
    point_cloud->set_is_dense(true);
  }
}


}  // namespace pandar
}  // namespace drivers
