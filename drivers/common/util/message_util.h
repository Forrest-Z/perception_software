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

/**
 * @file
 * @brief Some string util functions.
 */

#pragma once

#include <memory>
#include <string>

#include "google/protobuf/message.h"

#include "common/time/time.h"

namespace common {
namespace util {

// Expose some useful utils from protobuf.
using ::google::protobuf::Message;

template <typename T, typename std::enable_if<
                          std::is_base_of<Message, T>::value, int>::type = 0>
static void FillHeader(const std::string& module_name, T* msg) {
  static std::atomic<uint64_t> sequence_num = {0};
  auto* header = msg->mutable_header();
  double timestamp = apollo::common::time::Clock::NowInSeconds();
  header->set_module_name(module_name);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(static_cast<unsigned int>(
      sequence_num.fetch_add(1)));
}

inline size_t MessageFingerprint(const google::protobuf::Message& message) {
  static std::hash<std::string> hash_fn;
  std::string proto_bytes;
  message.SerializeToString(&proto_bytes);
  return hash_fn(proto_bytes);
}

}  // namespace util
}  // namespace common
