#ifndef GEOMETRY_UTIL_H
#define GEOMETRY_UTIL_H

#include <cmath>
#include <algorithm>
#include <Eigen/Core>

template <class PointType>
float point3dDistance(PointType &point1, PointType &point2)
{
    float distanceX = (point1.x - point2.x) * (point1.x - point2.x);
    float distanceY = (point1.y - point2.y) * (point1.y - point2.y);
    float distanceZ = (point1.z - point2.z) * (point1.z - point2.z);
    return std::sqrt(distanceX + distanceY + distanceZ);
}

template <class PointType>
float point2dDistance(PointType &point1, PointType &point2)
{
    float distanceX = (point1.x - point2.x) * (point1.x - point2.x);
    float distanceY = (point1.y - point2.y) * (point1.y - point2.y);
    return std::sqrt(distanceX + distanceY);
}

template <class CloudType>
void getCloudMinMaxPoint(const CloudType &srcCloudPoint, Eigen::Vector4f &minPoint, Eigen::Vector4f &maxPoint)
{
    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    if (srcCloudPoint.size() == 0)
    {
        return;
    }

    for(const auto &point: srcCloudPoint)
    {
        minX = std::min(point.x, minX);
        minY = std::min(point.y, minY);
        minZ = std::min(point.z, minZ);
        maxX = std::max(point.x, maxX);
        maxY = std::max(point.y, maxY);
        maxZ = std::max(point.x, maxZ);
    }

    minPoint[0] = minX;
    minPoint[1] = minY;
    minPoint[2] = minZ;
    maxPoint[0] = maxX;
    maxPoint[1] = maxY;
    maxPoint[2] = maxZ;
}

#endif // GEOMETRY_UTIL_H
