#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <chrono>
#include "lidar_common/lidar_cloud_filter.h"
#include "lidar_common/merge_object.h"
#include "lidar_common/object_builder.h"
#include "lidar_ground/ground/lidar_ground_extractor.h"
#include "lidar_ground/ground/lidar_plane_fit_ground.h"
#include "lidar_ground/ground/lidar_ransac_fit_ground.h"
#include "lidar_segment/clustering/euclidean_cluster.h"
#include "lidar_segment/clustering/rbnn_cluster.h"
#include "lidar_segment/clustering/cvc_cluster.h"
#include "lidar_tracking/kalman_tracking/kalman_multiple_tracker.h"
#include "show_tools/pcl_show/pcl_show.h"

#define MERGE_THRESHOLD (2.8)

using namespace perception::lidar;

static std::mutex mutexLock;
static pcl::visualization::PCLVisualizer::Ptr myViewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

static void viewerRunner(){
    while(!myViewer->wasStopped()){
      mutexLock.lock();
      myViewer->spinOnce(10);
      mutexLock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char* argv[]) {
    const std::string pcd_dir = "/home/lpj/github/perception_software/ai_modules/test_data/pcd/";
	const std::string pcd_test_path = "/home/lpj/github/perception_software/ai_modules/test_data/pcd/test.txt";
    std::ifstream fin;

    int frameNumber = 0;

    LidarCloudFilter<PCLPoint> cloudFilter;

    LidarGroundExtractor groundExtractor;
    LidarPlaneFitGround planeFitGround;
    LidarRANSACFitGround ransac_ground;

    EuclideanCluster cluster;
    //RBNNCluster cluster;
    //CVCCluster cluster;
    MergeObject mergeObject;
    ObjectBuilder builder;
    KalmanMultipleTracker multipleTracker;

    std::vector<std::shared_ptr<perception::base::Object>> clusterResult;
    std::vector<std::shared_ptr<perception::base::Object>> mergeResult;
    std::vector<std::shared_ptr<perception::base::Object>> trackedObjects;

	PCLPointCloud::Ptr frameCloud(new PCLPointCloud);
    pcl::PointIndices::Ptr clodeIndex(new pcl::PointIndices);
    PCLPointCloud::Ptr filteringCloud(new PCLPointCloud);
    PCLPointCloud::Ptr noGroundCloud(new PCLPointCloud);
	pcl::PCDReader reader;
    std::string pcd_name;

    fin.open(pcd_test_path, std::ifstream::in);
    if(!fin.is_open()){
        std::cout << "can not open test file" << std::endl;
    }

	myViewer->setBackgroundColor(0, 0, 0);
    myViewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    myViewer->addCoordinateSystem(1.0);
    myViewer->initCameraParameters();
    myViewer->resetCamera();

    //std::thread viewerThread(&viewerRunner);
    while (fin >> pcd_name) {
        std::string pcd_path = pcd_dir + pcd_name;
        if(reader.read(pcd_path, *frameCloud) < 0){
            std::cout << "can not open pcd file:" << pcd_path << std::endl;
            continue;
        }
        std::cout <<"cloud point count："<< frameCloud->points.size() << std::endl; 
        if(frameNumber % 1 == 0){
            cloudFilter.pointcloudFilteringByX(frameCloud, -4.0f, 6.0f, clodeIndex);
            cloudFilter.getFilteringCloudPoint(frameCloud, clodeIndex, filteringCloud);
            std::cout <<"filtering cloud point count："<< filteringCloud->points.size() << std::endl; 
            //groundExtractor.filteringGround(cloud, noGroundCloud);
            planeFitGround.filteringGround(filteringCloud, noGroundCloud);
            //ransac_ground.filteringGround(cloud, noGroundCloud);
            //ransac_ground.removeFloor(cloud, 1, noGroundCloud);

            cluster.clustering(noGroundCloud, clusterResult);
            mergeObject.mergeAll(clusterResult, MERGE_THRESHOLD, mergeResult);

            ObjectBuilderOptions builder_options;
            if (!builder.Build(builder_options, 0.0, mergeResult)) {
                return -1;
            }
        }
        multipleTracker.mutilpleTracking(0, mergeResult, -1);
        multipleTracker.getTrackedObjectList(trackedObjects);
	
	    updateViewerData(myViewer, frameCloud, noGroundCloud);
        updateViewerData(myViewer, mergeResult, trackedObjects);
        myViewer->spinOnce(20000);

        frameNumber++;
        std::cout << "frame number:" << frameNumber << std::endl;
    }
    fin.close();
	//viewerThread.join();
	return (0);
 
}