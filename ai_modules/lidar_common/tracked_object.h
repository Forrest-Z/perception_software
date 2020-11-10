#ifndef TRACKED_OBJECT_H_
#define TRACKED_OBJECT_H_

#include <memory>
#include <string>
#include <vector>
#include "base/object.h"

namespace perception {
namespace lidar {

struct TrackedObject {

  TrackedObject();
  TrackedObject(base::ObjectPtr obj_ptr);

  double timestamp;
  size_t tracker_id;
  int laneId;
  // bbox
  Eigen::Vector3f center;
  Eigen::Vector3f size;
  Eigen::Vector3f velocity;// m/s
  float theta;
  Eigen::Vector3f direction;

  base::ObjectPtr object_ptr;

};  // struct TrackedObject

}  // namespace lidar
}  // namespace perception

#endif //TRACKED_OBJECT_H_
