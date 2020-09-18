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
#pragma once

//#define  LIDAR_DRIVER_THREAD_PROCESS
//#define LIDAR_DRIVER_THREAD_POOL_PROCESS

#include <memory>
#include <string>
#include <thread>

#include <cyber/cyber.h>
#include <cyber/base/concurrent_object_pool.h>

#include "proto/pandar_config.pb.h"
#include "proto/pandar.pb.h"
#include "proto/pointcloud.pb.h"
#include "lidar/pandar/pandar_sdk/pandarGeneral_sdk.h"

namespace drivers {
namespace pandar {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using drivers::PointCloud;

class PandarSDKDriverComponent : public Component<> {
 public:
  bool Init() override;

 private:

  void LidarCallback(boost::shared_ptr<pandar_sdk::PPointCloud> cld, double timestamp);
  void ConvertPclToPointcloud(
    const boost::shared_ptr<pandar_sdk::PPointCloud> &cld_msg,
    std::shared_ptr<PointCloud> point_cloud);


  std::shared_ptr<Writer<PointCloud>> writer_;
  std::unique_ptr<pandar_sdk::PandarGeneralSDK> pandar_sdk_ = nullptr;
  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;
  int pool_size_ = 8;

  config::PandarConfig config_;

};

CYBER_REGISTER_COMPONENT(PandarSDKDriverComponent)

}  // namespace pandar_sdk
}  // namespace drivers
