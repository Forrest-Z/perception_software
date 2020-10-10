#include "lidar_ground/ground/lidar_ground_extractor.h"
#include "lidar_ground/ground/lidar_plane_fit_ground.h"
#include "lidar_ground/ground/lidar_ransac_fit_ground.h"
#include "lidar_segment/clustering/euclidean_cluster.h"
#include "lidar_segment/clustering/rbnn_cluster.h"
#include "lidar_segment/clustering/cvc_cluster.h"
#include "lidar_common/merge_object.h"
#include "lidar_common/object_builder.h"
#include "show_tools/pcl_show/pcl_show.h"

#define MERGE_THRESHOLD (2.8)

using namespace perception::lidar;

int main(int argc, char* argv[]) {
	const std::string pcd_path = "/home/lpj/github/perception_software/ai_modules/test_data/cloud_120.pcd";
	pcl::visualization::PCLVisualizer::Ptr myViewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    LidarGroundExtractor groundExtractor;
    LidarPlaneFitGround planeFitGround;
    LidarRANSACFitGround ransac_ground;

    //EuclideanCluster cluster;
    //RBNNCluster cluster;
    CVCCluster cluster;
    MergeObject mergeObject;
    ObjectBuilder builder;
    std::vector<std::shared_ptr<perception::base::Object>> clusterResult;
    std::vector<std::shared_ptr<perception::base::Object>> mergeResult;

	PCLPointCloud::Ptr cloud(new PCLPointCloud);
    PCLPointCloud::Ptr noGroundCloud(new PCLPointCloud);
	pcl::PCDReader reader;
	if(reader.read(pcd_path, *cloud) < 0)
    {
		std::cout << "can not open pcd file" << std::endl;
        return -1;
    }
	int num_points = cloud->points.size();
	std::cout <<"cloud point count："<< num_points << std::endl; 

	myViewer->setBackgroundColor(0, 0, 0);
    myViewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    myViewer->addCoordinateSystem(1.0);
    myViewer->initCameraParameters();
    myViewer->resetCamera();

	//groundExtractor.filteringGround(cloud, noGroundCloud);
    planeFitGround.filteringGround(cloud, noGroundCloud);
    //ransac_ground.filteringGround(cloud, noGroundCloud);
    //ransac_ground.removeFloor(cloud, 1, noGroundCloud);

    cluster.clustering(noGroundCloud, clusterResult);
    mergeObject.mergeAll(clusterResult, MERGE_THRESHOLD, mergeResult);

    ObjectBuilderOptions builder_options;
    if (!builder.Build(builder_options, 0.0, mergeResult)) {
      return -1;
    }
	
	updateViewerData(myViewer, cloud, noGroundCloud);
    updateViewerData(myViewer, mergeResult);
	while(!myViewer->wasStopped())
  	{
      myViewer->spinOnce(10);
  	}
	return (0);
 
}