#include "lidar_common/gaussian_distribution_outlier_filter.h"

namespace perception {
namespace lidar {

GaussianDistributionOutlierFilter::GaussianDistributionOutlierFilter()
{

}

GaussianDistributionOutlierFilter::~GaussianDistributionOutlierFilter()
{

}

/*----------------------------
*功能：采用高斯分布的方法进行离群点的判别
*-----------------------------
*输入：Piont3D的原始点云数据
*输出：除去离群点之后的Point3D结构的点云数据
*/
void GaussianDistributionOutlierFilter::applyFilter(const RawPointcloud::Ptr &InputPointCloud,
                                               RawPointcloud::Ptr &OutPointCloud)
{
    //均值
    double X_Ave = 0;
    double Y_Ave = 0;
    double Z_Ave = 0;

    //方差
    double X_Var = 0;
    double Y_Var = 0;
    double Z_Var = 0;

    //求均值
    for (int i = 0; i <InputPointCloud->size(); i++)
    {
        X_Ave += InputPointCloud->points[i].x;
        Y_Ave += InputPointCloud->points[i].y;
        Z_Ave += InputPointCloud->points[i].z;
    }

    X_Ave = X_Ave / InputPointCloud->size();
    Y_Ave = Y_Ave / InputPointCloud->size();
    Z_Ave = Z_Ave / InputPointCloud->size();

    //求方差
    for (int j = 0; j <InputPointCloud->size(); j++)
    {
        X_Var += (InputPointCloud->points[j].x - X_Ave)*(InputPointCloud->points[j].x - X_Ave);
        Y_Var += (InputPointCloud->points[j].y - Y_Ave)*(InputPointCloud->points[j].y - Y_Ave);
        Z_Var += (InputPointCloud->points[j].z - Z_Ave)*(InputPointCloud->points[j].z - Z_Ave);
    }
    X_Var = X_Var / InputPointCloud->size();
    Y_Var = Y_Var / InputPointCloud->size();
    Z_Var = Z_Var / InputPointCloud->size();

    //开始判断是是否是离群点
    for (int k = 0; k <InputPointCloud->size(); k++)
    {
        if (((abs(InputPointCloud->points[k].z - Z_Ave) / sqrt(Z_Var))<1.5 || (abs(InputPointCloud->points[k].z + Z_Ave) / sqrt(Z_Var))<1.5)
                && ((abs(InputPointCloud->points[k].y - Y_Ave) / sqrt(Y_Var))<1.8 || (abs(InputPointCloud->points[k].y + Y_Ave) / sqrt(Y_Var))<1.8)
                && ((abs(InputPointCloud->points[k].x - X_Ave) / sqrt(X_Var))<1.8 || (abs(InputPointCloud->points[k].x + X_Ave) / sqrt(X_Var))<1.))
        {
            OutPointCloud->push_back(InputPointCloud->points[k]);//如果不是离群点，则把该点放入CloudPoint中
        }
    }

    OutPointCloud->header = InputPointCloud->header;
}

}  // namespace lidar
}  // namespace perception