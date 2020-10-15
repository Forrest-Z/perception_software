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

#include <memory>
#include <string>

#include "proto/velodyne_config.pb.h"
#include "proto/velodyne.pb.h"
#include "lidar/velodyne/driver/socket_input.h"

namespace drivers {
namespace velodyne {

constexpr int BLOCKS_PER_PACKET_DRIVER = 12;
constexpr int  BLOCK_SIZE_DRIVER = 100;
constexpr double PACKET_RATE_VLP16 = 754;
constexpr double PACKET_RATE_HDL32E = 1808.0;
constexpr double PACKET_RATE_HDL64E_S2 = 3472.17;
constexpr double PACKET_RATE_HDL64E_S3S = 3472.17;
constexpr double PACKET_RATE_HDL64E_S3D = 5789;
constexpr double PACKET_RATE_VLS128 = 6250.0;

class VelodyneDriver {
 public:
  explicit VelodyneDriver(const config::VelodyneConfig &config) : config_(config) {}
  virtual ~VelodyneDriver();

  virtual bool Poll(const std::shared_ptr<VelodyneScan>& scan);
  virtual void Init();
  virtual void PollPositioningPacket();
  void SetPacketRate(const double packet_rate) { packet_rate_ = packet_rate; }

 protected:
  config::VelodyneConfig config_;
  std::unique_ptr<Input> input_ = nullptr;
  std::unique_ptr<Input> positioning_input_ = nullptr;
  std::string topic_;
  double packet_rate_ = 0.0;

  uint64_t basetime_ = 0;
  uint32_t last_gps_time_ = 0;
  uint64_t last_count_ = 0;
  static uint64_t sync_counter;

  std::thread positioning_thread_;

  virtual int PollStandard(std::shared_ptr<VelodyneScan> scan);
  bool SetBaseTime();
  void SetBaseTimeFromNmeaTime(NMEATimePtr nmea_time, uint64_t *basetime);
  void UpdateGpsTopHour(uint32_t current_time);
};

class Velodyne64Driver : public VelodyneDriver {
 public:
  explicit Velodyne64Driver(const config::VelodyneConfig &config)
      : VelodyneDriver(config) {}
  ~Velodyne64Driver() {}

  void Init() override;
  bool Poll(const std::shared_ptr<VelodyneScan>& scan) override;

 private:
  bool CheckAngle(const VelodynePacket &packet);
  int PollStandardSync(std::shared_ptr<VelodyneScan> scan);
};

class VelodyneDriverFactory {
 public:
  static VelodyneDriver *CreateDriver(const config::VelodyneConfig &config);
};

}  // namespace velodyne
}  // namespace drivers