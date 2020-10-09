#include "lidar_segment/clustering/rbnn_cluster.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace perception {
namespace lidar {

RBNNCluster::RBNNCluster(){
	// if distance >= 120m, ingnore
    seg_distance_ = {17, 38, 70, 120};
    cluster_radius_ = {0.5f, 1.0f, 3.0f, 5.0f};
}

RBNNCluster::~RBNNCluster(){

}

void RBNNCluster::clustering(const PCLPointCloud::Ptr &srcPointcloud, std::vector<std::shared_ptr<base::Object>> &objects){
	std::vector<PCLPointCloud::Ptr> segmentPointCloudArray;
    objects.clear();
    if(srcPointcloud->size() <= 0)
        return;
	clusterByDistance(srcPointcloud, seg_distance_, segmentPointCloudArray);
    for (size_t i = 0; i < segmentPointCloudArray.size(); i++){
        if(segmentPointCloudArray[i]->size() > 0){
			std::vector<int> cluster_indices;
			cluster_indices = rbnn(segmentPointCloudArray[i], cluster_radius_[i]);
			computeCentroid(srcPointcloud, cluster_indices, 9, 5000, &objects);
        }
    }
}

std::vector<int> RBNNCluster::rbnn(const PCLPointCloud::Ptr &srcCloudPoints, const float radius){
    std::vector<int> cluster_indices = std::vector<int>(srcCloudPoints->points.size(), -1);
	pcl::KdTreeFLANN<PCLPoint> kdtree;
	kdtree.setInputCloud(srcCloudPoints);
	int current_cluster = 0;
	for (size_t i = 0; i < srcCloudPoints->points.size(); i++) {
		// 1. skip this point if it's already within a cluster
		if (cluster_indices[i] != -1)
			continue;
		// 2. find nearest neigbours for current point
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		if (kdtree.radiusSearch(srcCloudPoints->points[i], static_cast<float>(radius), pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
			// 3. merge or assign new clusters
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				int oc = cluster_indices[i]; // original point's cluster
				int nc = cluster_indices[pointIdxRadiusSearch[j]]; // neighbor point's cluster
				if (oc != -1 && nc != -1) {
					if (oc != nc){
                        mergeClusters(cluster_indices, oc, nc);
                    }
				}
				else {
					if (nc != -1) {
						cluster_indices[i] = nc;
					}
					else {
						if (oc != -1) {
							cluster_indices[pointIdxRadiusSearch[j]] = oc;
						}
					}
				}
			}
		}
		// 4. if there is still no cluster, create a new one and assign all neighbors to it
		if (cluster_indices[i] == -1) {
			current_cluster++;
			cluster_indices[i] = current_cluster;
			for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
				cluster_indices[pointIdxRadiusSearch[j]] = current_cluster;
			}
		}
	}
	return cluster_indices;
}

void RBNNCluster::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2){
    for (size_t i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}

}  // namespace lidar
}  // namespace perception