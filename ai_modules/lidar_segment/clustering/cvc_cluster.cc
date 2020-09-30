#include "lidar_segment/clustering/cvc_cluster.h"

namespace perception {
namespace lidar {

static const float PI = 3.1415926;

static float minrange = 3;
static float maxrange = 3;
static float minazimuth = 0;
static float maxazimuth = 0;
static float deltaA = 2;
static float deltaR = 0.35;
static float deltaP = 1.2;
static int length = 0;
static int width = 0;
static int height = 0;

CVCCluster::CVCCluster(){

}

CVCCluster::~CVCCluster(){

}

void CVCCluster::clustering(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<int> &result){

}

void CVCCluster::calculateAPR(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<PointAPR>& vapr){

    for (size_t i = 0; i <s srcCloudPoints.points.size(); ++i){
           PointAPR par;
           par.polar_angle = Polar_angle_cal(srcCloudPoints.points[i].x, srcCloudPoints.points[i].y);
           par.range = sqrt(srcCloudPoints.points[i].x * srcCloudPoints.points[i].x + \
                            srcCloudPoints.points[i].y * srcCloudPoints.points[i].y);
           par.azimuth =(float) atan2(srcCloudPoints.points[i].z,par.range);
           if(par.azimuth < minazimuth){
                 minazimuth = par.azimuth;
           }
           if(par.azimuth > maxazimuth){
                 maxazimuth = par.azimuth;
           }
           if(par.range < minrange){
                 minrange = par.range;
           }
           if(par.range > maxrange){
                 maxrange = par.range;
           }
           vapr.push_back(par);
    }

    length = round((maxrange - minrange) / deltaR);
    width = 301;
    height = round((maxazimuth - minazimuth) / deltaA);
}

float CVCCluster::polarAngleCalculate(const float x, const float y){
    float temp_tangle = 0;
    if(x== 0 && y ==0){
        temp_tangle = 0;
    }else if(y>=0){
        temp_tangle = std::atan2(y, x);
    }else if(y<=0){
        temp_tangle = std::atan2(y, x) + 2 * PI;
    }
    return temp_tangle;
}

}  // namespace lidar
}  // namespace perception