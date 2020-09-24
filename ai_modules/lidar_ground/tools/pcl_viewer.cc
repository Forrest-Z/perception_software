#include "lidar_ground/tools/pcl_viewer.h"

namespace perception {
namespace lidar {

PCLViewer::PCLViewer()
{

}

PCLViewer::~PCLViewer()
{

}

pcl::visualization::PCLVisualizer::Ptr PCLViewer::createViewer(const PCLPointCloud::Ptr &srcCloudPoint)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    pcl::visualization::PointCloudColorHandlerCustom<PCLPoint> colorHandler(srcCloudPoint, 0, 100, 255);
    viewer->addPointCloud<PCLPoint>(srcCloudPoint, colorHandler, "srcCloud", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();
    return viewer;
}

void PCLViewer::updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer, const PCLPointCloud::Ptr &srcCloud)
{
    viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PCLPoint> colorHandler(srcCloud, 0, 100, 255);
    viewer->addPointCloud<PCLPoint>(srcCloud, colorHandler, "srcCloud", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);
}

void PCLViewer::updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const PCLPointCloud &srcCloud, const PCLPointCloud &srcCloud1)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr showCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto &point : srcCloud)
    {
      pcl::PointXYZRGB prgb;
      prgb.x = point.x;
      prgb.y = point.y;
      prgb.z = point.z;
      prgb.r = static_cast<unsigned char>(255);
      prgb.g = static_cast<unsigned char>(0);
      prgb.b = static_cast<unsigned char>(0);

      showCloud->push_back(prgb);
    }

    for(const auto &point : srcCloud1)
    {
        pcl::PointXYZRGB prgb;
        prgb.x = point.x;
        prgb.y = point.y;
        prgb.z = point.z;
        prgb.r = static_cast<unsigned char>(0);
        prgb.g = static_cast<unsigned char>(255);
        prgb.b = static_cast<unsigned char>(0);

        showCloud->push_back(prgb);
    }
    viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(showCloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(showCloud, colorHandler, "srcCloud", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);
}

void PCLViewer::updateViewerData(const pcl::visualization::PCLVisualizer::Ptr viewer,
                      const pcl::PointCloud<PointXYZIRL> &srcCloud, const pcl::PointCloud<PointXYZIRL> &srcCloud1)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr showCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto &point : srcCloud)
    {
      pcl::PointXYZRGB prgb;
      prgb.x = point.x;
      prgb.y = point.y;
      prgb.z = point.z;
      prgb.r = static_cast<unsigned char>(255);
      prgb.g = static_cast<unsigned char>(0);
      prgb.b = static_cast<unsigned char>(0);

      showCloud->push_back(prgb);
    }

    for(const auto &point : srcCloud1)
    {
        pcl::PointXYZRGB prgb;
        prgb.x = point.x;
        prgb.y = point.y;
        prgb.z = point.z;
        prgb.r = static_cast<unsigned char>(0);
        prgb.g = static_cast<unsigned char>(255);
        prgb.b = static_cast<unsigned char>(0);

        showCloud->push_back(prgb);
    }
    viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(showCloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(showCloud, colorHandler, "srcCloud", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);
}

}  // namespace lidar
}  // namespace perception