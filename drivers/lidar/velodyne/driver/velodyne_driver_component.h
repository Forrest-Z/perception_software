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

#include "cyber/cyber.h"

#include "proto/velodyne_config.pb.h"
#include "proto/velodyne.pb.h"
#include "lidar/velodyne/driver/driver.h"

#ifdef LIDAR_DRIVER_THREAD_PROCESS
#include <deque>
#include "cyber/base/concurrent_object_pool.h"
#include "lidar/velodyne/parser/convert.h"
#endif

namespace drivers {
namespace velodyne {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using drivers::velodyne::VelodyneScan;

#ifdef LIDAR_DRIVER_THREAD_PROCESS
using drivers::PointCloud;
using apollo::cyber::base::CCObjectPool;
#endif
class VelodyneDriverComponent : public Component<> {
 public:
  ~VelodyneDriverComponent() {
    if (device_thread_->joinable()) {
      device_thread_->join();
    }
  }
  bool Init() override;

 private:
  void device_poll();
#ifdef LIDAR_DRIVER_THREAD_PROCESS
  #ifdef LIDAR_DRIVER_THREAD_POOL_PROCESS
  void convert_handle(const std::shared_ptr<VelodyneScan> arg);
  #endif
#endif
  volatile bool runing_;  ///< device thread is running
  uint32_t seq_ = 0;
  std::shared_ptr<std::thread> device_thread_;
  std::shared_ptr<VelodyneDriver> dvr_;  ///< driver implementation class
#ifndef LIDAR_DRIVER_THREAD_PROCESS 
  std::shared_ptr<apollo::cyber::Writer<VelodyneScan>> writer_;
#endif

#ifdef LIDAR_DRIVER_THREAD_PROCESS
  std::shared_ptr<Writer<PointCloud>> writer_PointCloud_;
  std::unique_ptr<Convert> conv_ = nullptr;
  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;
  int pool_size_ = 8;

  #ifdef LIDAR_DRIVER_THREAD_POOL_PROCESS
  std::shared_ptr<std::thread> convert_thread_;
  #endif
#endif
};

CYBER_REGISTER_COMPONENT(VelodyneDriverComponent)

}  // namespace velodyne
}  // namespace drivers
