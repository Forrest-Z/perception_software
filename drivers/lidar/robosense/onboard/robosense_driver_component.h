#pragma once

#include <memory>
#include <string>
#include <thread>

#include <cyber/cyber.h>
#include <cyber/base/concurrent_object_pool.h>

#include "proto/robosense_config.pb.h"
#include "proto/pointcloud.pb.h"
#include "rs_driver/api/lidar_driver.h"

namespace drivers {
namespace robosense {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using drivers::PointCloud;

struct PointXYZI  ///< user defined point type
{
  float x;
  float y;
  float z;
  float intensity;
};

class RobosenseDriverComponent : public Component<> {
 public:
  bool Init() override;

 private:

  void LidarCallback(const ::robosense::lidar::PointCloudMsg<PointXYZI>& msg);
  void LocalExceptionCallback(const ::robosense::lidar::Error& msg);
  void ConvertToPointcloud(
    const ::robosense::lidar::PointCloudMsg<PointXYZI>& msg,
    std::shared_ptr<PointCloud> point_cloud);


  std::shared_ptr<Writer<PointCloud>> writer_;
  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;
  int pool_size_ = 8;

  std::shared_ptr<::robosense::lidar::LidarDriver<PointXYZI>> driver_ptr_ = nullptr;

  config::RobosenseConfig config_;

};

CYBER_REGISTER_COMPONENT(RobosenseDriverComponent)

}  // namespace robosense
}  // namespace drivers
