#include "lidar_common/object_builder.h"

#include <algorithm>

#include "common/geometry/common.h"
#include "common/geometry/convex_hull_2d.h"

namespace perception {
namespace lidar {

static const float kEpsilon = 1e-6f;
static const float kEpsilonForSize = 1e-2f;
static const float kEpsilonForLine = 1e-3f;
static const double EPSILON = 1e-9;
using perception::base::PointF;
using perception::base::PointD;
using ObjectPtr = std::shared_ptr<perception::base::Object>;
using PointFCloud = perception::base::PointCloud<PointF>;
using PolygonDType = perception::base::PointCloud<PointD>;

bool ObjectBuilder::Init(const ObjectBuilderInitOptions& options) {
  return true;
}

bool ObjectBuilder::Build(const ObjectBuilderOptions& options, const double timestamp, \
                          std::vector<std::shared_ptr<base::Object>> &objectList)
{
  for (size_t i = 0; i < objectList.size(); ++i) {
    if (objectList.at(i)) {
      objectList.at(i)->id = static_cast<int>(i);
      ComputePolygon2D(objectList.at(i));
    }
  }
  for (size_t i = 0; i < objectList.size(); ++i) {
    reconstructPolygon3d(objectList.at(i));
    for (size_t loop = 0; loop < 3; ++loop) {
      if (objectList.at(i)->size[loop] < kEpsilonForSize) {
        objectList.at(i)->size[loop] = kEpsilonForSize;
      }
    }
    static const float quater_pi = static_cast<float>(M_PI);
    float yaw = static_cast<float>(atan2(objectList.at(i)->direction[1],
                                   objectList.at(i)->direction[0]));
    objectList.at(i)->direction[0] = cosf(yaw - quater_pi);
    objectList.at(i)->direction[1] = sinf(yaw - quater_pi);
    objectList.at(i)->direction[2] = 0;
    objectList.at(i)->theta = static_cast<float>(atan2(objectList.at(i)->direction[1],
                                           objectList.at(i)->direction[0]));
    objectList.at(i)->latest_tracked_time = timestamp;
  }
  return true;
}

void ObjectBuilder::ComputePolygon2D(ObjectPtr object) {
  Eigen::Vector3f min_pt;
  Eigen::Vector3f max_pt;
  PointFCloud& cloud = object->lidar_supplement.cloud;
  GetMinMax3D(cloud, &min_pt, &max_pt);
  object->size[2] = max_pt[2] - min_pt[2];
  if (cloud.size() < 4u) {
    SetDefaultValue(min_pt, max_pt, object);
    return;
  }
  LinePerturbation(&cloud);
  common::ConvexHull2D<PointFCloud, PolygonDType> hull;
  hull.GetConvexHull(cloud, &(object->polygon));
}

void ObjectBuilder::ComputeOtherObjectInformation(ObjectPtr object) {
  object->anchor_point = object->center;
  double timestamp = 0.0;
  size_t num_point = object->lidar_supplement.cloud.size();
  for (size_t i = 0; i < num_point; ++i) {
    timestamp += object->lidar_supplement.cloud.points_timestamp(i);
  }
  if (num_point > 0) {
    timestamp /= static_cast<double>(num_point);
  }
  object->latest_tracked_time = timestamp;
}

void ObjectBuilder::ComputePolygonSizeCenter(ObjectPtr object) {
  if (object->lidar_supplement.cloud.size() < 4u) {
    return;
  }
  Eigen::Vector3f dir = object->direction;
  common::CalculateBBoxSizeCenter2DXY(object->lidar_supplement.cloud, dir,
                                      &(object->size), &(object->center));
  if (object->lidar_supplement.is_background) {
    float length = object->size[0];
    float width = object->size[1];
    Eigen::Matrix<float, 3, 1> ortho_dir(-object->direction[1],
                                         object->direction[0], 0.0);
    if (length < width) {
      object->direction = ortho_dir;
      object->size[0] = width;
      object->size[1] = length;
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    if (object->size[i] < kEpsilonForSize) {
      object->size[i] = kEpsilonForSize;
    }
  }
  object->theta = static_cast<float>(atan2(object->direction[1],
                                           object->direction[0]));
}

void ObjectBuilder::SetDefaultValue(const Eigen::Vector3f& min_pt_in,
                                    const Eigen::Vector3f& max_pt_in,
                                    ObjectPtr object) {
  Eigen::Vector3f min_pt = min_pt_in;
  Eigen::Vector3f max_pt = max_pt_in;
  // handle degeneration case
  for (int i = 0; i < 3; i++) {
    if (max_pt[i] - min_pt[i] < kEpsilonForSize) {
      max_pt[i] = max_pt[i] + kEpsilonForSize / 2;
      min_pt[i] = min_pt[i] - kEpsilonForSize / 2;
    }
  }
  Eigen::Vector3f center((min_pt[0] + max_pt[0]) / 2,
                         (min_pt[1] + max_pt[1]) / 2,
                         (min_pt[2] + max_pt[2]) / 2);
  object->center = Eigen::Vector3d(static_cast<double>(center[0]),
                                   static_cast<double>(center[1]),
                                   static_cast<double>(center[2]));
  float length = max_pt[0] - min_pt[0];
  float width = max_pt[1] - min_pt[1];
  float height = max_pt[2] - min_pt[2];
  if (length < width) {
    object->size = Eigen::Vector3f(width, length, height);
    object->direction = Eigen::Vector3f(0.0, 1.0, 0.0);
  } else {
    object->size = Eigen::Vector3f(length, width, height);
    object->direction = Eigen::Vector3f(1.0, 0.0, 0.0);
  }
  // polygon
  if (object->lidar_supplement.cloud.size() < 4) {
    object->polygon.resize(4);
    object->polygon[0].x = static_cast<double>(min_pt[0]);
    object->polygon[0].y = static_cast<double>(min_pt[1]);
    object->polygon[0].z = static_cast<double>(min_pt[2]);

    object->polygon[1].x = static_cast<double>(max_pt[0]);
    object->polygon[1].y = static_cast<double>(min_pt[1]);
    object->polygon[1].z = static_cast<double>(min_pt[2]);

    object->polygon[2].x = static_cast<double>(max_pt[0]);
    object->polygon[2].y = static_cast<double>(max_pt[1]);
    object->polygon[2].z = static_cast<double>(min_pt[2]);

    object->polygon[3].x = static_cast<double>(min_pt[0]);
    object->polygon[3].y = static_cast<double>(max_pt[1]);
    object->polygon[3].z = static_cast<double>(min_pt[2]);
  }
}

bool ObjectBuilder::LinePerturbation(PointFCloud* cloud) {
  if (cloud->size() >= 3) {
    int start_point = 0;
    int end_point = 1;
    float diff_x = cloud->at(start_point).x - cloud->at(end_point).x;
    float diff_y = cloud->at(start_point).y - cloud->at(end_point).y;
    size_t idx = 0;
    for (idx = 2; idx < cloud->size(); ++idx) {
      float tdiff_x = cloud->at(idx).x - cloud->at(start_point).x;
      float tdiff_y = cloud->at(idx).y - cloud->at(start_point).y;
      if (fabs(diff_x * tdiff_y - tdiff_x * diff_y) > kEpsilonForLine) {
        return false;
      }
    }
    cloud->at(0).x += kEpsilonForLine;
    cloud->at(1).y += kEpsilonForLine;
    return true;
  }
  return true;
}

void ObjectBuilder::GetMinMax3D(const PointFCloud& cloud,
                                Eigen::Vector3f* min_pt,
                                Eigen::Vector3f* max_pt) {
  (*min_pt)[0] = (*min_pt)[1] = (*min_pt)[2] = FLT_MAX;
  (*max_pt)[0] = (*max_pt)[1] = (*max_pt)[2] = -FLT_MAX;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (std::isnan(cloud[i].x) || std::isnan(cloud[i].y) ||
        std::isnan(cloud[i].z)) {
      continue;
    }
    (*min_pt)[0] = std::min((*min_pt)[0], cloud[i].x);
    (*max_pt)[0] = std::max((*max_pt)[0], cloud[i].x);
    (*min_pt)[1] = std::min((*min_pt)[1], cloud[i].y);
    (*max_pt)[1] = std::max((*max_pt)[1], cloud[i].y);
    (*min_pt)[2] = std::min((*min_pt)[2], cloud[i].z);
    (*max_pt)[2] = std::max((*max_pt)[2], cloud[i].z);
  }
}

void ObjectBuilder::reconstructPolygon3d(std::shared_ptr<perception::base::Object> obj){
    if (obj->polygon.size() <= 0){
        return;
    }
    size_t max_point_index = 0;
    size_t min_point_index = 0;
    Eigen::Vector3f p;
    p[0] = static_cast<float>(obj->polygon[0].x);
    p[1] = static_cast<float>(obj->polygon[0].y);
    p[2] = static_cast<float>(obj->polygon[0].z);
    Eigen::Vector3f max_point = p;
    Eigen::Vector3f min_point = p;
    for (size_t i = 1; i < obj->polygon.size(); ++i)
    {
        Eigen::Vector3f p;
        p[0] = static_cast<float>(obj->polygon[i].x);
        p[1] = static_cast<float>(obj->polygon[i].y);
        p[2] = static_cast<float>(obj->polygon[i].z);
        Eigen::Vector3f ray = p;
        // clock direction
        if (max_point[0] * ray[1] - ray[0] * max_point[1] < EPSILON)
        {
            max_point = ray;
            max_point_index = i;
        }
        // anti-clock direction
        if (min_point[0] * ray[1] - ray[0] * min_point[1] > EPSILON)
        {
            min_point = ray;
            min_point_index = i;
        }
    }
    Eigen::Vector3f line = max_point - min_point;

    double total_len = 0;
    double max_dis = 0;
    bool has_out = false;
    for (size_t i = min_point_index, count = 0;
         count < obj->polygon.size();
         i = (i + 1) % obj->polygon.size(), ++count)
    {
        Eigen::Vector3f p_x;
        p_x[0] = static_cast<float>(obj->polygon[i].x);
        p_x[1] = static_cast<float>(obj->polygon[i].y);
        p_x[2] = static_cast<float>(obj->polygon[i].z);
        size_t j = (i + 1) % obj->polygon.size();
        if (j != min_point_index && j != max_point_index)
        {
            Eigen::Vector3f p;
            p[0] = static_cast<float>(obj->polygon[j].x);
            p[1] = static_cast<float>(obj->polygon[j].y);
            p[2] = static_cast<float>(obj->polygon[j].z);
            Eigen::Vector3f ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < EPSILON)
            {
                double dist = std::sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                                   (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist - max_dis > EPSILON)
                {
                    max_dis = dist;
                }
            }
            else
            {
                // outline
                has_out = true;
            }
        }
        else if ((i == min_point_index && j == max_point_index) ||
                   (i == max_point_index && j == min_point_index))
        {
            size_t k = (j + 1) % obj->polygon.size();
            Eigen::Vector3f p_k;
            p_k[0] = static_cast<float>(obj->polygon[k].x);
            p_k[1] = static_cast<float>(obj->polygon[k].y);
            p_k[2] = static_cast<float>(obj->polygon[k].z);
            Eigen::Vector3f p_j;
            p_j[0] = static_cast<float>(obj->polygon[j].x);
            p_j[1] = static_cast<float>(obj->polygon[j].y);
            p_j[2] = static_cast<float>(obj->polygon[j].z);
            Eigen::Vector3f ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
            } else {
                // outline
                has_out = true;
            }
        } else if (j == min_point_index || j == max_point_index) {
            Eigen::Vector3f p;
            p[0] = static_cast<float>(obj->polygon[j].x);
            p[1] = static_cast<float>(obj->polygon[j].y);
            p[2] = static_cast<float>(obj->polygon[j].z);
            Eigen::Vector3f ray = p_x - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < EPSILON) {
                float dist = sqrtf((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                                   (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist > max_dis) {
                    max_dis = dist;
                }
            } else {
                // outline
                has_out = true;
            }
        }
    }

    size_t count = 0;
    double min_area = std::numeric_limits<double>::max();
    for (size_t i = min_point_index; count < obj->polygon.size();
         i = (i + 1) % obj->polygon.size(), ++count) {
        Eigen::Vector3f p_x;
        p_x[0] = static_cast<float>(obj->polygon[i].x);
        p_x[1] = static_cast<float>(obj->polygon[i].y);
        p_x[2] = static_cast<float>(obj->polygon[i].z);
        size_t j = (i + 1) % obj->polygon.size();
        Eigen::Vector3f p_j;
        p_j[0] = static_cast<float>(obj->polygon[j].x);
        p_j[1] = static_cast<float>(obj->polygon[j].y);
        p_j[2] = static_cast<float>(obj->polygon[j].z);
        float dist = sqrtf((p_x[0] - p_j[0]) * (p_x[0] - p_j[0]) +
                           (p_x[1] - p_j[1]) * (p_x[1] - p_j[1]));
        if (dist < max_dis && (dist / total_len) < 0.5) {
            continue;
        }

        if (j != min_point_index && j != max_point_index) {
            Eigen::Vector3f p;
            p[0] = static_cast<float>(obj->polygon[j].x);
            p[1] = static_cast<float>(obj->polygon[j].y);
            p[2] = static_cast<float>(obj->polygon[j].z);
            Eigen::Vector3f ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
                Eigen::Vector3f center;
                double length = 0;
                double width = 0;
                Eigen::Vector3f dir;
                double area = computeAreaAlongOneEdge(obj, i, &center, &length,
                                                      &width, &dir);
                if (area < min_area) {
                    obj->center = center.cast<double>();
                    obj->size[0] = static_cast<float>(length);
                    obj->size[1] = static_cast<float>(width);
                    obj->direction = dir;
                    min_area = area;
                }
            } else {
                // outline
            }
        } else if ((i == min_point_index && j == max_point_index) ||
                   (i == max_point_index && j == min_point_index)) {
            if (!has_out) {
                continue;
            }
            Eigen::Vector3f center;
            double length = 0;
            double width = 0;
            Eigen::Vector3f dir;
            double area =
                computeAreaAlongOneEdge(obj, i, &center, &length, &width, &dir);
            if (area < min_area) {
                obj->center = center.cast<double>();;
                obj->size[0] = static_cast<float>(length);
                obj->size[1] = static_cast<float>(width);
                obj->direction = dir;
                min_area = area;
            }
        } else if (j == min_point_index || j == max_point_index) {
            Eigen::Vector3f p;
            p[0] = static_cast<float>(obj->polygon[i].x);
            p[1] = static_cast<float>(obj->polygon[i].y);
            p[2] = static_cast<float>(obj->polygon[i].z);
            Eigen::Vector3f ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0) {
                Eigen::Vector3f center;
                double length = 0.0;
                double width = 0.0;
                Eigen::Vector3f dir;
                double area = computeAreaAlongOneEdge(obj, i, &center, &length,
                                                      &width, &dir);
                if (area < min_area) {
                    obj->center = center.cast<double>();
                    obj->size[0] = static_cast<float>(length);
                    obj->size[1] = static_cast<float>(width);
                    obj->direction = dir;
                    min_area = area;
                }
            } else {
                // outlier
            }
        }
    }

    obj->direction.normalize();
}

/**
 * @brief compute area of obj->Polygon along <Edge: first_in_point->index>
 *  width: highest triangle
 *  length: longest distance between points projected into <Edge:
 * first_in_point->index>
 *  center: length,width extended rectangle
 *  dir: length's edge 2d direction vector
 * @param obj
 * @param first_in_point, <Edge: first_in_point->first_in_point's next>
 * @param center
 * @param length
 * @param width
 * @param dir
 * @return
 */
double ObjectBuilder::computeAreaAlongOneEdge(std::shared_ptr<perception::base::Object> obj,
                                              size_t first_in_point,
                                              Eigen::Vector3f *center,
                                              double *length,
                                              double *width,
                                              Eigen::Vector3f *dir) {
    std::vector<Eigen::Vector3f> ns;
    Eigen::Vector3f v(0.0, 0.0, 0.0);
    Eigen::Vector3f vn(0.0, 0.0, 0.0);
    Eigen::Vector3f n(0.0, 0.0, 0.0);
    double len = 0;
    double wid = 0;
    // Edge: first_in_point->index
    size_t index = (first_in_point + 1) % obj->polygon.size();
    for (size_t i = 0; i < obj->polygon.size(); ++i) {
        if (i != first_in_point && i != index) {
            // compute v
            Eigen::Vector3f o(0.0, 0.0, 0.0);
            Eigen::Vector3f a(0.0, 0.0, 0.0);
            Eigen::Vector3f b(0.0, 0.0, 0.0);
            o[0] = static_cast<float>(obj->polygon[i].x);
            o[1] = static_cast<float>(obj->polygon[i].y);
            o[2] = 0;
            b[0] = static_cast<float>(obj->polygon[first_in_point].x);
            b[1] = static_cast<float>(obj->polygon[first_in_point].y);
            b[2] = 0;
            a[0] = static_cast<float>(obj->polygon[index].x);
            a[1] = static_cast<float>(obj->polygon[index].y);
            a[2] = 0;
            float k =
                ((a[0] - o[0]) * (b[0] - a[0]) + (a[1] - o[1]) * (b[1] - a[1]));
            k = k /
                ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
            k = k * -1;
            // n is pedal of src
            // TODO(gary): Projected to <Edge: first_in_point->index>???
            n[0] = (b[0] - a[0]) * k + a[0];
            n[1] = (b[1] - a[1]) * k + a[1];
            n[2] = 0;
            // compute height from src to line
            Eigen::Vector3f edge1 = o - b;
            Eigen::Vector3f edge2 = a - b;
            // cross product
            float height = fabsf(edge1[0] * edge2[1] - edge2[0] * edge1[1]);
            height = height / sqrtf(edge2[0] * edge2[0] + edge2[1] * edge2[1]);
            // highest triangle contributes the width of the Polygon
            if (height > wid) {
                wid = height;
                v = o;
                vn = n;
            }
        } else {
            n[0] = static_cast<float>(obj->polygon[i].x);
            n[1] = static_cast<float>(obj->polygon[i].y);
            n[2] = 0;
        }
        ns.push_back(n);
    }

    // longest distance between points projected into
    //   <Edge:first_in_point->index>
    // contributes the length of the Polygon
    size_t point_num1 = 0;
    size_t point_num2 = 0;
    for (size_t i = 0; i < ns.size() - 1; ++i) {
        Eigen::Vector3f p1 = ns[i];
        for (size_t j = i + 1; j < ns.size(); ++j) {
            Eigen::Vector3f p2 = ns[j];
            double dist = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                               (p1[1] - p2[1]) * (p1[1] - p2[1]));
            if (dist > len) {
                len = dist;
                point_num1 = i;
                point_num2 = j;
            }
        }
    }
    /*
     * ground center
     *  v: vertex of the highest triangle
     *  vn: projected point of v into <Edge: first_in_point->index>
     *  ns[point_num1]/ns[point_num2]: longest distance's points projected into
     * <Edge: first_in_point->index>
     */
    Eigen::Vector3f vp1 = ns[point_num1] + (v - vn);
    Eigen::Vector3f vp2 = ns[point_num2] + (v - vn);
    (*center) = (vp1 + vp2 + ns[point_num1] + ns[point_num2]) / 4;
    (*center)[2] = static_cast<float>(obj->polygon[0].z);
    /*
     * only a 2D direction
     *  <Edge: first_in_point->index> is the direction: 2 projected points'
     * vector
     *  <Edge: first_in_point->index> vertical to the direction: wid's edge
     * vector
     */
    if (len > wid) {
        *dir = ns[point_num2] - ns[point_num1];
    } else {
        *dir = vp1 - ns[point_num1];
    }
    // length & width are interchangeable
    *length = len > wid ? len : wid;
    *width = len > wid ? wid : len;
    return (*length) * (*width);
}

}  // namespace lidar
}  // namespace perception
