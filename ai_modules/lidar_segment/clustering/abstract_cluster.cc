#include "lidar_segment/clustering/abstract_cluster.h"

namespace perception {
namespace lidar {

void AbstractCluster::clusterByDistance(const PCLPointCloud::Ptr &srcPointcloud, const std::vector<float> &seg_distance,
                                        std::vector<PCLPointCloud::Ptr> &segmentPointCloudArray){
    for (size_t i = 0; i < seg_distance.size(); i++){
        PCLPointCloud::Ptr temp(new PCLPointCloud);
        temp->header = srcPointcloud->header;
        segmentPointCloudArray.push_back(temp);
    }

    for (size_t i = 0; i < srcPointcloud->points.size(); i++)
    {
        PCLPoint current_point;
        current_point.x = srcPointcloud->points[i].x;
        current_point.y = srcPointcloud->points[i].y;
        current_point.z = srcPointcloud->points[i].z;

        float origin_distance = std::sqrt(std::pow(current_point.x, 2.0f) + std::pow(current_point.y, 2.0f));

        if (origin_distance < seg_distance[0]){
            segmentPointCloudArray[0]->points.push_back(current_point);
        }
        for (size_t i = 1; i < seg_distance.size(); i++){
            if(origin_distance < seg_distance[i] && origin_distance >= seg_distance[i-1]){
                segmentPointCloudArray[i]->points.push_back(current_point);
            }
        }
    }
}

void AbstractCluster::filteringByFrequentValue(const std::vector<int> &values, const int min_size, const int max_size,
                                               std::vector<int> &cluster_result){
    std::map<int, int> histcounts;
	int max = 0, maxi;
	for (size_t i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		}
		else {
			histcounts[values[i]] += 1;
		}
	}

	std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
	for (auto& val : tr) {
		std::cout << val.first << " : " << val.second << std::endl;
		if(val.second >= min_size && val.second < max_size){
			cluster_result.push_back(val.first);
		}
	}
}

void AbstractCluster::computeCentroid(const PCLPointCloud::Ptr &srcPointCloud,
                                      const std::vector<int> &localIndices,
                                      const int min_size, const int max_size,
                                      std::vector<std::shared_ptr<base::Object>>* objects){
	std::vector<int> cluster_result;
	filteringByFrequentValue(localIndices, min_size, max_size, cluster_result);
	for (size_t index = 0; index < cluster_result.size(); index++){
        base::ObjectPtr object(new base::Object);
        PCLPointCloud::Ptr tempCloud(new PCLPointCloud);
        double tempX = 0;
        double tempY = 0;
        double tempZ = 0;
        size_t count = 0;
        tempCloud->header = srcPointCloud->header;
		for(size_t i = 0; i < srcPointCloud->points.size(); i++){
			if(localIndices[i] == cluster_result[index]){
				tempX += srcPointCloud->points[i].x;
            	tempY += srcPointCloud->points[i].y;
            	tempZ += srcPointCloud->points[i].z;
				count++;
            	tempCloud->push_back(srcPointCloud->points[i]);
			}
		}
        object->anchor_point[0] = tempX / static_cast<double>(count);
        object->anchor_point[1] = tempY / static_cast<double>(count);
        object->anchor_point[2] = tempZ / static_cast<double>(count);
        object->lidar_supplement.num_points_in_roi = count;
        pclToAttributePointCloud(tempCloud, object->lidar_supplement.cloud);
        objects->push_back(object);
    }

}

void AbstractCluster::computeCentroid(const PCLPointCloud::Ptr &srcPointCloud,
                     const std::vector<pcl::PointIndices> &localIndices,
                     std::vector<std::shared_ptr<base::Object>>* objects)
{
    for (size_t i = 0; i < localIndices.size(); i++)
    {
        base::ObjectPtr object(new base::Object);
        PCLPointCloud::Ptr tempCloud(new PCLPointCloud);
        double tempX = 0;
        double tempY = 0;
        double tempZ = 0;
        size_t count = localIndices[i].indices.size();
        tempCloud->header = srcPointCloud->header;
        for (auto iter = localIndices[i].indices.begin(); iter != localIndices[i].indices.end(); ++iter)
        {
            tempX += srcPointCloud->points[*iter].x;
            tempY += srcPointCloud->points[*iter].y;
            tempZ += srcPointCloud->points[*iter].z;
            tempCloud->push_back(srcPointCloud->points[*iter]);
        }
        object->anchor_point[0] = tempX / static_cast<double>(count);
        object->anchor_point[1] = tempY / static_cast<double>(count);
        object->anchor_point[2] = tempZ / static_cast<double>(count);
        object->lidar_supplement.num_points_in_roi = count;
        pclToAttributePointCloud(tempCloud, object->lidar_supplement.cloud);
        objects->push_back(object);
    }
}

}  // namespace lidar
}  // namespace perception