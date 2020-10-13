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

#include "cyber/cyber.h"

#include "common/util/message_util.h"
#include "lidar/velodyne/driver/velodyne_driver_component.h"

//#define LIDAR_TIME_COST_COLLECT
namespace drivers {
namespace velodyne {

bool VelodyneDriverComponent::Init() {
  AINFO << "Velodyne driver component init";
  config::VelodyneConfig velodyne_config;
  if (!GetProtoConfig(&velodyne_config)) {
  #ifdef LIDAR_DRIVER_THREAD_PROCESS
    AWARN << "Load config failed, config file" << config_file_path_;
  #endif
    return false;
  }
  AINFO << "Velodyne config: " << velodyne_config.DebugString();
  // start the driver
#ifndef LIDAR_DRIVER_THREAD_PROCESS
  writer_ = node_->CreateWriter<VelodyneScan>(velodyne_config.scan_channel());  
#else
  conv_.reset(new Convert());
  conv_->init(velodyne_config);
  writer_PointCloud_ =
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
#endif
  VelodyneDriver *driver = VelodyneDriverFactory::CreateDriver(velodyne_config);
  if (driver == nullptr) {
    return false;
  }
  dvr_.reset(driver);
  dvr_->Init();
  // spawn device poll thread
  runing_ = true;
  device_thread_ = std::shared_ptr<std::thread>(
      new std::thread(std::bind(&VelodyneDriverComponent::device_poll, this)));
  device_thread_->detach();

  return true;
}

/** @brief Device poll thread main loop. */
void VelodyneDriverComponent::device_poll() {
  //added thread name by hezb
  prctl(PR_SET_NAME, "velodynePoll");
  //added end

  while (!apollo::cyber::IsShutdown()) {
    // poll device until end of file
    std::shared_ptr<VelodyneScan> scan = std::make_shared<VelodyneScan>();
    bool ret = dvr_->Poll(scan);
    if (ret) {
      #ifdef LIDAR_TIME_COST_COLLECT
      #if (defined LIDAR_DRIVER_THREAD_PROCESS) && (!defined LIDAR_DRIVER_THREAD_POOL_PROCESS)
      cyber::Time st_newTime;
      cyber::Time st_oldTime = cyber::Time::Now();
      #endif /*end of LIDAR_DRIVER_THREAD_POOL_PROCESS*/
      #endif /*end of LIDAR_TIME_COST_COLLECT*/
      common::util::FillHeader("velodyne", scan.get());
      
      #ifndef  LIDAR_DRIVER_THREAD_PROCESS
      writer_->Write(scan);
      #endif

      #ifdef LIDAR_DRIVER_THREAD_PROCESS
      #ifdef LIDAR_DRIVER_THREAD_POOL_PROCESS
        convert_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&VelodyneDriverComponent::convert_handle, this,scan)));
        convert_thread_->detach();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      #else
      std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
      if (point_cloud_out == nullptr) {
        AWARN << "poin cloud pool return nullptr, will be create new.";
        point_cloud_out = std::make_shared<PointCloud>();
        point_cloud_out->mutable_point()->Reserve(140000);
      }
      if (point_cloud_out == nullptr) {
       AWARN << "point cloud out is nullptr";
        //return false;
    } else {
        point_cloud_out->Clear();

        conv_->ConvertPacketsToPointcloud(scan, point_cloud_out);
  
        if (point_cloud_out == nullptr || point_cloud_out->point_size() == 0) {
          AWARN << "point_cloud_out convert is empty.";
        //return false;
        }else{
          writer_PointCloud_->Write(point_cloud_out);
        }        
      }
      #endif /*end of LIDAR_DRIVER_THREAD_POOL_PROCESS*/
      #endif /*end of LIDAR_DRIVER_THREAD_PROCESS*/

      #ifdef LIDAR_TIME_COST_COLLECT
      #if (defined LIDAR_DRIVER_THREAD_PROCESS) && (!defined LIDAR_DRIVER_THREAD_POOL_PROCESS)
      st_newTime = cyber::Time::Now();
      cyber::Duration dur = (st_newTime - st_oldTime);
      std::cout <<  "\033[31m"<<"VelodyneDriverComponent write time: = "<< dur.ToNanosecond() <<"ns\033[0m"<<std::endl;
      #endif /* end of LIDAR_DRIVER_THREAD_POOL_PROCESS*/
      #endif /*end of LIDAR_TIME_COST_COLLECT*/
    } else {
      AWARN << "device poll failed";
    }
  }

  AERROR << "CompVelodyneDriver thread exit";
  runing_ = false;
}

#ifdef LIDAR_DRIVER_THREAD_POOL_PROCESS
void VelodyneDriverComponent::convert_handle(const std::shared_ptr<VelodyneScan> arg) {
  //added thread name by hezb
  prctl(PR_SET_NAME, "velodyneConv");
  //added end

  std::shared_ptr<VelodyneScan> tmp_scan = arg;

  #ifdef LIDAR_TIME_COST_COLLECT
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
    //return false;
  }else{
    point_cloud_out->Clear();

    conv_->ConvertPacketsToPointcloud(tmp_scan, point_cloud_out);
 
    if (point_cloud_out == nullptr || point_cloud_out->point_size() == 0) {
      AWARN << "point_cloud_out convert is empty.";
      //return false;
    }else{
      writer_PointCloud_->Write(point_cloud_out);
    }        
  }

  #ifdef LIDAR_TIME_COST_COLLECT
  st_newTime = cyber::Time::Now();
  cyber::Duration dur = (st_newTime - st_oldTime);
  std::cout <<  "\033[31m"<<"VelodyneDriverComponent write time: = "<< dur.ToNanosecond() <<"ns\033[0m"<<std::endl;
  #endif
}
#endif
}  // namespace velodyne
}  // namespace drivers
