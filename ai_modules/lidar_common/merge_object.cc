#include "lidar_common/merge_object.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace perception {
namespace lidar {

MergeObject::MergeObject()
{

}

MergeObject::~MergeObject()
{

}

void MergeObject::mergeAll(std::vector<std::shared_ptr<base::Object>> &objectList, const double mergeThreshold,
                std::vector<std::shared_ptr<base::Object>>& result)
{
    std::vector<bool> visited_clusters(objectList.size(), false);
    std::vector<bool> merged_clusters(objectList.size(), false);
    std::vector<Eigen::Vector3d> inputData;
    result.clear();
    inputData.clear();
    for(const auto &object: objectList)
    {
        inputData.push_back(object->anchor_point);
        //std::cout << "centroid:" << object->anchor_point[0] << " " << object->anchor_point[1] << std::endl;
    }

    for (size_t i = 0; i < objectList.size(); i++)
    {
        if (!visited_clusters[i])
        {
            std::vector<size_t> merge_indices;
            visited_clusters[i] = true;
            merge_indices.push_back(i);
            checkObjectMerge(i, inputData, visited_clusters, merge_indices, mergeThreshold);
            mergeObject(objectList, result, merge_indices, merged_clusters);
        }
    }

    for (size_t i = 0; i < objectList.size(); i++)
    {
        // check for clusters not merged, add them to the output
        if (!merged_clusters[i])
        {
            result.push_back(objectList[i]);
        }
    }
}

void MergeObject::checkObjectMerge(size_t inputId, const std::vector<Eigen::Vector3d> &inputData,
                         std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
                         double mergeThreshold){
    Eigen::Vector3d point_a = inputData[inputId];
    for (size_t i = 0; i < inputData.size(); i++)
    {
        if (i != inputId && !in_out_visited_clusters[i])
        {
            Eigen::Vector3d point_b = inputData[i];
            double hightDiff = std::abs(point_b[2] - point_a[2]);
            double distance = std::sqrt(std::pow(point_b[0] - point_a[0], 2) + std::pow(point_b[1] - point_a[1], 2));
            //std::cout << " with " << i << " dist:" << distance << " " << hightDiff << std::endl;
            if (distance <= mergeThreshold && hightDiff <= 1.2f)
            {
                in_out_visited_clusters[i] = true;
                out_merge_indices.push_back(i);
                //std::cout << "Merging " << inputId << " with " << i << " dist:" << distance << std::endl;
                checkObjectMerge(i, inputData, in_out_visited_clusters, out_merge_indices, mergeThreshold);
            }
        }
    }
}

void  MergeObject::mergeObject(const std::vector<std::shared_ptr<base::Object>>& srcObjects, 
                    std::vector<std::shared_ptr<base::Object>>& out_clusters,
                    std::vector<size_t> in_merge_indices,
                    std::vector<bool>& in_out_merged_clusters){
    base::ObjectPtr mergedObject(new base::Object);
    std::vector<float> prob{0.0, 0, 0, 0};
    mergedObject->lidar_supplement.cloud.clear();
   
    //std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;

    for (size_t i = 0; i < in_merge_indices.size(); i++)
    {
        mergedObject->lidar_supplement.cloud += srcObjects[in_merge_indices[i]]->lidar_supplement.cloud;
        in_out_merged_clusters[in_merge_indices[i]] = true;
    }
    if (mergedObject->lidar_supplement.cloud.size() > 0)
    {
        mergedObject->lidar_supplement.num_points_in_roi =mergedObject->lidar_supplement.cloud.size();
        mergedObject->lidar_supplement.on_use = true;
        mergedObject->lidar_supplement.is_background = false;
        mergedObject->lidar_supplement.raw_classification_methods.push_back("cluster");
        mergedObject->lidar_supplement.raw_probs.push_back(prob);
        out_clusters.push_back(mergedObject);
    }
}

}  // namespace lidar
}  // namespace perception