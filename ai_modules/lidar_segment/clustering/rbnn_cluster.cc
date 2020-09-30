#include "lidar_segment/clustering/rbnn_cluster.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace perception {
namespace lidar {

RBNNCluster::RBNNCluster(){

}

RBNNCluster::~RBNNCluster(){

}

void RBNNCluster::clustering(const PCLPointCloud::Ptr &srcCloudPoints, const double radius, 
                             std::vector<int> &result){
    std::vector<int> cluster_indices;
    rbnn(srcCloudPoints, radius)
 //delete the clusters that are too small

}

std::vector<int> RBNNCluster::rbnn(const PCLPointCloud::Ptr &srcCloudPoints, const double radius){
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
		if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
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
int RBNNCluster::mostFrequentValue(const std::vector<int> &values){
    std::map<int, int> histcounts;
	for (size_t i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		}
		else {
			histcounts[values[i]] += 1;
		}
	}

    int max = 0, maxi;
	std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
	for (auto& val : tr) {
		cout << val.first << " : " << val.second << endl;
		if (val.second > max) {
			max = val.second;
			maxi = val.first;
		}
	}
	return maxi;
}

}  // namespace lidar
}  // namespace perception