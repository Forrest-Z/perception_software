#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

#include "lidar_common/transform_type.h"

namespace perception {
namespace lidar {

class CoordinateTransform
{
public:
    CoordinateTransform();
    ~CoordinateTransform();

    void setBasePose(double latitude, double longitude, double altitude);

    Eigen::Vector3d poseToEnu(double latitude, double longitude, double altitude);

    Eigen::Vector3d poseToecef(const Eigen::Vector3d &pose);
    void ecef2enu(const Eigen::Vector3d &pose, const double *r, double *e);

    Eigen::Quaternionf getRotation(float angularX, float angularY, float angularZ);

private:

    void xyz2enu(const Eigen::Vector3d &pose, double *E);
    void matmul(const char *tr, int n, int k, int m, double alpha,
                       const double *A, const double *B, double beta, double *C);

    Eigen::Vector3d basePose;
};

}  // namespace lidar
}  // namespace perception

#endif // COORDINATE_TRANSFORM_H
