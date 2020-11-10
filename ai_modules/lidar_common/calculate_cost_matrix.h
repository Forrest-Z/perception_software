#ifndef CALCULATE_COST_MATRIX_H
#define CALCULATE_COST_MATRIX_H

#include <opencv2/core.hpp>
#include "base/object.h"

namespace perception {
namespace lidar {

class CalculateCostMatrix
{
public:
    CalculateCostMatrix();
    ~CalculateCostMatrix();

    void calculateEuclideanDistance(const std::vector<base::ObjectPtr>& detections,
                                    const std::vector<base::ObjectPtr>& trackerObjects,
                                    std::vector< std::vector<double> > &costMatrix);

    void calculateMahalanobisDistance(const std::vector<base::ObjectPtr>& detections,
                                      const std::vector<base::ObjectPtr>& trackerObjects,
                                      std::vector< std::vector<double> > &costMatrix);
};

}  // namespace lidar
}  // namespace perception

#endif // CALCULATE_COST_MATRIX_H
