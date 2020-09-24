#include "lidar_ground/ground/lidar_ground_extractor.h"
#include "lidar_ground/ground/lidar_plane_fit_ground.h"
#include "lidar_ground/ground/lidar_ransac_fit_ground.h"
#include "lidar_ground/tools/pcl_viewer.h"

using namespace perception::lidar;

void printCloud(const PCLPointCloud &srcCloud){
    for (const auto &point : srcCloud)
    {
        std::cout << "intensity:" << point.intensity << std::endl;
    }
}

int main(int argc, char* argv[]) {
	const std::string pcd_path = "/home/lpj/github/perception_software/ai_modules/test_data/cloud_120.pcd";
	PCLViewer pclViewer;
	pcl::visualization::PCLVisualizer::Ptr myViewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    LidarGroundExtractor groundExtractor;
    LidarPlaneFitGround planeFitGround;
    LidarRANSACFitGround ransac_ground;
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
    //planeFitGround.filteringGround(cloud, noGroundCloud);
    ransac_ground.filteringGround(cloud, noGroundCloud);
    //ransac_ground.removeFloor(cloud, 1, noGroundCloud);
	
	pclViewer.updateViewerData(myViewer, *cloud, *noGroundCloud);
	while(!myViewer->wasStopped())
  	{
      myViewer->spinOnce(10);
  	}
	return (0);
 
}