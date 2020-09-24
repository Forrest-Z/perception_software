#ifndef GAUSSIAN_DISTRIBUTION_OUTLIER_FILTER_H
#define GAUSSIAN_DISTRIBUTION_OUTLIER_FILTER_H

#include "common/pcl_util.h"

namespace perception {
namespace lidar {

class GaussianDistributionOutlierFilter
{
public:
  GaussianDistributionOutlierFilter();
  virtual ~GaussianDistributionOutlierFilter();

  void applyFilter(const RawPointcloud::Ptr &InputPointCloud, RawPointcloud::Ptr &OutPointCloud);
};

}  // namespace lidar
}  // namespace perception

#endif // GAUSSIAN_DISTRIBUTION_OUTLIER_FILTER_H
