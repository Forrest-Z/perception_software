#include "lidar_common/tracked_object.h"

namespace perception {
namespace lidar {

TrackedObject::TrackedObject()
{
    timestamp = 0;
    laneId = -1;
    center[0] = 0;
    center[1] = 0;
    center[2] = 0;
    size[0] = 0;
    size[1] = 0;
    size[2] = 0;
    velocity[0] = 0;
    velocity[1] = 0;
    velocity[2] = 0;
    theta = 0;
    direction[0] = 0;
    direction[1] = 0;
    direction[2] = 0;
    object_ptr = nullptr;
}

TrackedObject::TrackedObject(base::ObjectPtr obj_ptr)
{
    timestamp = 0;
    laneId = -1;
    if(obj_ptr){
        this->object_ptr = obj_ptr;
        center = obj_ptr->center.cast<float>();
        size = obj_ptr->size;
        theta = obj_ptr->theta;
        direction = obj_ptr->direction;
        velocity[0] = 0;
        velocity[1] = 0;
        velocity[2] = 0;
    }
}

}  // namespace lidar
}  // namespace perception