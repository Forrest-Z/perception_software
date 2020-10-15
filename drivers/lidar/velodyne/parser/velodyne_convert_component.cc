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

#include "cyber/cyber.h"

#include "lidar/velodyne/parser/velodyne_convert_component.h"

//#define DEBUG_CONVERT_COST_TIME

namespace drivers {
namespace velodyne {

bool VelodyneConvertComponent::Init() {
  config::VelodyneConfig velodyne_config;
  if (!GetProtoConfig(&velodyne_config)) {
    AWARN << "Load config failed, config file" << config_file_path_;
    return false;
  }

  conv_.reset(new Convert());
  conv_->init(velodyne_config);
  writer_ =
      node_->CreateWriter<PointCloud>(velodyne_config.convert_channel_name());
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
  AINFO << "Point cloud comp convert init success";
  return true;
}

bool VelodyneConvertComponent::Proc(
    const std::shared_ptr<VelodyneScan>& scan_msg) {
#ifdef DEBUG_CONVERT_COST_TIME
  cyber::Time st_newTime;
  cyber::Time st_oldTime = cyber::Time::Now();
#endif

  std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
  if (point_cloud_out == nullptr) {
    AWARN << "poin cloud pool return nullptr, will be create new.";
    point_cloud_out = std::make_shared<PointCloud>();
    point_cloud_out->mutable_point()->Reserve(140000);
  }
  if (point_cloud_out == nullptr) {
    AWARN << "point cloud out is nullptr";
    return false;
  }
  point_cloud_out->Clear();
  conv_->ConvertPacketsToPointcloud(scan_msg, point_cloud_out);

  if (point_cloud_out == nullptr || point_cloud_out->point_size() == 0) {
    AWARN << "point_cloud_out convert is empty.";
    return false;
  }
  writer_->Write(point_cloud_out);

#ifdef DEBUG_CONVERT_COST_TIME
  st_newTime = cyber::Time::Now();
  cyber::Duration dur = (st_newTime - st_oldTime);
  std::cout <<  "\033[31m"<<"VelodyneDriverComponent write time: = "<< dur.ToNanosecond() <<"ns\033[0m"<<std::endl;
#endif

  return true;
}

}  // namespace velodyne
}  // namespace drivers