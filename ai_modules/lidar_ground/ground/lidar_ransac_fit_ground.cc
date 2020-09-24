#include "lidar_ground/ground/lidar_ransac_fit_ground.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

namespace perception {
namespace lidar {

LidarRANSACFitGround::LidarRANSACFitGround(){

}

LidarRANSACFitGround::~LidarRANSACFitGround(){

}

void LidarRANSACFitGround::filteringGround(const PCLPointCloud::Ptr &srcPointcloud, PCLPointCloud::Ptr &dstPointcloud){
    const int num_points = srcPointcloud->points.size();
    std::unordered_set<int> inliersResult;
	pcl::PointIndices::Ptr inputIndices(new pcl::PointIndices);
	PCLPointCloud::Ptr filteringCloud(new PCLPointCloud);
	Platform plane;
    cloudFilter.pointcloudFilteringByZ(srcPointcloud, -10.0f, 0.3f, inputIndices);
	cloudFilter.getFilteringCloudPoint(srcPointcloud, inputIndices, filteringCloud);
    dstPointcloud->clear();
    plane = RANSACPlane(filteringCloud);
	std::cout << "总迭代完成后找到的最大平面点数："<< plane.num_points << std::endl; 
	float sqrt_abc = sqrt(plane.a * plane.a +  plane.c * plane.c + plane.b * plane.b);
	for (int i = 0; i < num_points; i++){
		PCLPoint pt = srcPointcloud->points[i];
		// calculate the distance between other points and plane
		float dist = fabs(plane.a * pt.x + plane.b * pt.y + plane.c * pt.z + plane.d) / sqrt_abc; 
		if (dist > 0.35f){
			dstPointcloud->push_back(pt);
		}
	}
}

void LidarRANSACFitGround::removeFloor(const PCLPointCloud::Ptr &srcPointcloud, const int flag, PCLPointCloud::Ptr &dstPointcloud){
	const double maxHeight = 0.3f;
    const double floorMaxAngle = 0.1f;
    pcl::PointIndices::Ptr inputIndices(new pcl::PointIndices);
    cloudFilter.pointcloudFilteringByZ(srcPointcloud, -10.0f, 3.0f, inputIndices);
    if(flag == 0)
    {
        pcl::PointIndicesPtr ground(new pcl::PointIndices);
        // Create the filtering object
        pcl::ProgressiveMorphologicalFilter<PCLPoint> pmf;
        pmf.setInputCloud (srcPointcloud);
        pmf.setIndices(inputIndices);
        pmf.setMaxWindowSize (20);
        pmf.setSlope (0.2f);
        pmf.setInitialDistance (0.1f);
        pmf.setMaxDistance(0.4f);
        pmf.extract (ground->indices);

        // Create the filtering object
        pcl::ExtractIndices<PCLPoint> extract;
        extract.setInputCloud (srcPointcloud);
        extract.setIndices(ground);
        extract.setNegative(true);//true removes the indices, false leaves only the indices
        extract.filter(*dstPointcloud);
    }
    else if(flag == 1)
    {
       pcl::SACSegmentation<PCLPoint> seg;
       pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
       pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

       seg.setOptimizeCoefficients(true);
       seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
       seg.setMethodType(pcl::SAC_RANSAC);
       seg.setMaxIterations(100);
       seg.setAxis(Eigen::Vector3f(0, 0, 1));
       seg.setEpsAngle(floorMaxAngle);

       seg.setDistanceThreshold(maxHeight);  // floor distance
       seg.setOptimizeCoefficients(true);
       seg.setInputCloud(srcPointcloud);
       seg.setIndices(inputIndices);
       seg.segment(*inliers, *coefficients);
       if (inliers->indices.size() == 0)
       {
           std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
       }

       // REMOVE THE FLOOR FROM THE CLOUD
       pcl::ExtractIndices<PCLPoint> extract;
       extract.setInputCloud(srcPointcloud);
       extract.setIndices(inliers);
       extract.setNegative(true);  // true removes the indices, false leaves only the indices
       extract.filter(*dstPointcloud);
    }
    dstPointcloud->header = srcPointcloud->header;
}

Platform LidarRANSACFitGround::RANSACPlane(const PCLPointCloud::Ptr &cloud){
    const float distanceTol = 0.2;
	const int num_points = cloud->points.size();
	int maxIterations = 40;
	Platform plane;
	plane.num_points = 0;
	while(maxIterations--){
		std::unordered_set<int> inliers;  //存放平面的内点，平面上的点也属于平面的内点
 
		//因为一开始定义inliers，内点并没有存在，所以随机在原始点云中随机选取了三个点作为点云的求取平面系数的三个点
		while (inliers.size() < 3)  //当内点小于3 就随机选取一个点 放入内点中 也就是需要利用到三个内点
		{
			inliers.insert(rand() % num_points);   //产生 0~num_points 中的一个随机数
		}//是为了随机的选取三个点
		// 需要至少三个点 才能找到地面
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
 
		//计算平面系数
		float a, b, c, d, sqrt_abc;
		a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		d = -(a * x1 + b * y1 + c * z1);
		sqrt_abc = sqrt(a * a + b * b + c * c);
 
		//分别计算这些点到平面的距离 
		for (int i = 0; i < num_points; i++)
		{
			if (inliers.count(i) > 0) //判断一下有没有内点
			{ // that means: if the inlier in already exist, we dont need do anymore
				continue;
			}
			PCLPoint point = cloud->points[i];
			float x = point.x;
			float y = point.y;
			float z = point.z;
			float dist = fabs(a * x + b * y + c * z + d) / sqrt_abc; // calculate the distance between other points and plane
			if (dist < distanceTol)//如果距离平面的距离小于设定的阈值就把他归为平面上的点
			{
				inliers.insert(i); //如果点云中的点 距离平面距离的点比较远 那么该点则视为内点
			}
			//将inliersResult 中的内容不断更新，因为地面的点一定是最多的，
            // 所以迭代40次 找到的inliersResult最大时（即找到最大的那个平面），也就相当于找到了地面
			//inliersResult 中存储的也就是地面上的点云
			if (inliers.size() > plane.num_points)
			{
				plane.num_points = inliers.size();
				plane.a = a;
				plane.b = b;
				plane.c = c;
				plane.d = d;
			}
		}
		std::cout <<"每次迭代提取的平面点数："<< inliers.size() << std::endl;
	}
	return plane;
}

}  // namespace lidar
}  // namespace perception