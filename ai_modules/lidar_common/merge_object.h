#ifndef MERGE_OBJECT_H
#define MERGE_OBJECT_H

#include "base/object.h"

namespace perception {
namespace lidar {

class MergeObject
{
public:
  MergeObject();
  ~MergeObject();

  void mergeAll(std::vector<std::shared_ptr<base::Object>> &objectList, const double mergeThreshold,
                std::vector<std::shared_ptr<base::Object>>& result);

private:

  void checkObjectMerge(size_t inputId, const std::vector<Eigen::Vector3d> &inputData,
                         std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
                         double mergeThreshold);

  void  mergeObject(const std::vector<std::shared_ptr<base::Object>>& srcObjects, 
                    std::vector<std::shared_ptr<base::Object>>& out_clusters,
                    std::vector<size_t> in_merge_indices,
                    std::vector<bool>& in_out_merged_clusters);
};

}  // namespace lidar
}  // namespace perception

#endif // MERGE_OBJECT_H
