#include <mutex>
#include <thread>

#include "cyber/cyber.h"
#include "show_tools/pcl_show/pcl_show.h"

#define LIDAR_POINT_CLOUD_CHANNEL "/apollo/sensor/pandar64/PointCloud2"
#define LIDAR_OBJECT_CHANNEL "/perception/obstacles"

static perception::lidar::PCLPointCloud::Ptr frameCloud;
static std::mutex mutexLock;
static pcl::visualization::PCLVisualizer::Ptr myViewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

namespace perception {

void lidarPointMessage(const std::shared_ptr<drivers::PointCloud>& message) 
{
      mutexLock.lock();
      lidar::typeToPCLCloud(message, frameCloud);
      lidar::updateViewerData(myViewer, frameCloud);
      mutexLock.unlock();
}

void lidarTrackedObjectsMessage(const std::shared_ptr<PerceptionObstacles> &message)
{
      double timestamp = message->header().timestamp_sec();
      Eigen::Affine3d pose = Eigen::Affine3d::Identity();
      // Eigen::Translation3d translation(0.0265676, 1.09419, 1.01061);
      // Eigen::Quaterniond quater(-0.0252111, 0.0206123, -0.0129993, -0.999385);
      // Eigen::Affine3d pandarToImu = translation * quater.toRotationMatrix();
      // rtkMutexLock.lock();
      // Eigen::Matrix4d temp1 = pandarToImu.matrix().inverse();
      // Eigen::Matrix4d temp2 = rtkPose.matrix().inverse();
      // Eigen::Matrix4d temp = static_cast<Eigen::Matrix<double, 4, 4, 0, 4, 4>>(temp2 * temp1);
      // //Eigen::Matrix3d world2pandar_rotate = temp.block<3, 3>(0, 0);
      // Eigen::Affine3d finalTranform(temp);
      // rtkMutexLock.unlock();

      mutexLock.lock();
      Eigen::Matrix4d world2lidar = pose.matrix().inverse();
      Eigen::Matrix3d world2pandar_rotate = world2lidar.block<3, 3>(0, 0);
      Eigen::Affine3d finalTranform(world2lidar);
      const int count = message->perception_obstacle_size();
      for(int index = 0; index < count; index++){
        perception::PerceptionObstacle* object = message->mutable_perception_obstacle(index);
        Eigen::Vector3d center;
        Eigen::Vector3d transformCenter;
        Eigen::Vector3d direction(static_cast<float>(std::cos(object->theta())),
                              static_cast<float>(std::sin(object->theta())),
                              0.0);
        Eigen::Vector3d transformdirection = static_cast<Eigen::Matrix<double, 3, 1, 0, 3, 1>>
                                                (world2pandar_rotate * direction);

        center[0] = (object->position()).x();
        center[1] = (object->position()).y();
        center[2] = (object->position()).z();
        transformCenter = finalTranform * center;
        (object->mutable_position())->set_x(transformCenter[0]);
        (object->mutable_position())->set_y(transformCenter[1]);
        (object->mutable_position())->set_z(transformCenter[2]);
        object->set_theta(std::atan2(transformdirection(1), transformdirection(0)));
      }
      lidar::updateViewerData(myViewer, message);
      mutexLock.unlock();
}

}  // namespace perception

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("show_test");

  frameCloud.reset(new pcl::PointCloud<perception::lidar::PCLPoint>());
  myViewer->setBackgroundColor(0, 0, 0);
  myViewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
  pcl::visualization::PointCloudColorHandlerCustom<perception::lidar::PCLPoint> colorHandler(frameCloud, 0, 100, 255);
  myViewer->addPointCloud<perception::lidar::PCLPoint>(frameCloud, colorHandler, "srcCloud", 0);
  myViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "srcCloud", 0);
  myViewer->addCoordinateSystem(1.0);
  myViewer->initCameraParameters();
  myViewer->resetCamera();

  // create listener
  auto listener =
       listener_node->CreateReader<drivers::PointCloud>(LIDAR_POINT_CLOUD_CHANNEL, &perception::lidarPointMessage);

  auto listener2 =
       listener_node->CreateReader<perception::PerceptionObstacles>(LIDAR_OBJECT_CHANNEL, 
                                                                    &perception::lidarTrackedObjectsMessage);

  while(!myViewer->wasStopped() && !apollo::cyber::IsShutdown())
  {
      mutexLock.lock();
      myViewer->spinOnce(10);
      mutexLock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  apollo::cyber::WaitForShutdown();
  return 0;
}
