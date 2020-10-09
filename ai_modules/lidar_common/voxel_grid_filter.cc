#include "lidar_common/voxel_grid_filter.h"
#include <iostream>
#include <algorithm>

namespace perception {
namespace lidar {

VoxelGridFilter::VoxelGridFilter()
{

}

VoxelGridFilter::~VoxelGridFilter()
{

}

void VoxelGridFilter::applyFilter(const RawPointcloud::Ptr &InputCloudPoint, const RawPointcloud::Ptr &OutPointCloud,
                                  float X_Voxel, float Y_Voxel, float Z_Voxel)
{
    if (InputCloudPoint->size()==0)
    {
        return;
    }

    Eigen::Vector4f min_p, max_p;
    getCloudMinMaxPoint<RawPointcloud>(*InputCloudPoint, min_p, max_p);

    Eigen::Vector4f inverse_leaf_size_;
    inverse_leaf_size_[0] = 1 / X_Voxel;
    inverse_leaf_size_[1] = 1 / Y_Voxel;
    inverse_leaf_size_[2] = 1 / Z_Voxel;

    //计算最小和最大边界框值
    Eigen::Vector4f min_b_, max_b_, div_b_, divb_mul_;
    min_b_[0] = static_cast<int> (floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor(min_p[1] * inverse_leaf_size_[0]));
    max_b_[1] = static_cast<int> (floor(max_p[1] * inverse_leaf_size_[0]));
    min_b_[2] = static_cast<int> (floor(min_p[2] * inverse_leaf_size_[0]));
    max_b_[2] = static_cast<int> (floor(max_p[2] * inverse_leaf_size_[0]));

    //计算沿所有轴所需的分割数
    div_b_[0] = max_b_[0] - min_b_[0] + 1;
    div_b_[1] = max_b_[1] - min_b_[1] + 1;
    div_b_[2] = max_b_[2] - min_b_[2] + 1;

    //设置除法乘数
    divb_mul_[0] = 1;
    divb_mul_[1] = div_b_[0];
    divb_mul_[2] =div_b_[0] * div_b_[1];

    //用于计算idx和pointcloud索引的存储
    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(InputCloudPoint->size());

    //第一步：遍历所有点并将它们插入到具有计算idx的index_vector向量中;具有相同idx值的点将有助于产生CloudPoint的相同点
    for (int i = 0; i < InputCloudPoint->size();i++)
    {
        int ijk0 = static_cast<int>(floor(InputCloudPoint->points[i].x * inverse_leaf_size_[0]) -
                static_cast<float> (min_b_[0]));
        int ijk1 = static_cast<int>(floor(InputCloudPoint->points[i].y * inverse_leaf_size_[1]) -
                static_cast<float> (min_b_[1]));
        int ijk2 = static_cast<int>(floor(InputCloudPoint->points[i].z * inverse_leaf_size_[2]) -
                static_cast<float> (min_b_[2]));

        //计算质心叶索引
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
        index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int> (idx), i));
    }
    //第二步：使用表示目标单元格的值作为索引对index_vector向量进行排序;实际上属于同一输出单元格的所有点都将彼此相邻
    std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

    //第三步：计数输出单元格，我们需要跳过所有相同的，相邻的idx值
    unsigned int total = 0;
    unsigned int index = 0;
    unsigned int min_points_per_voxel_ = 0;
    //first_and_last_indices_vector [i]表示属于对应于第i个输出点的体素的index_vector中的第一个点的index_vector中的索引，以及不属于第一个点的索引
    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
    first_and_last_indices_vector.reserve(index_vector.size());                              //分配内存空间

    while (index < index_vector.size())
    {
        unsigned int i = index + 1;
        while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
            ++i;
        if (i - index >= min_points_per_voxel_)
        {
            ++total;
            first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
        }
        index = i;
    }

    //第四步：计算质心，将它们插入最终位置
    float x_Sum, y_Sum, z_Sum;
    PPoint PointCloud;
    unsigned int first_index, last_index;
    for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
    {
        // 计算质心 - 来自所有输入点的和值，这些值在index_vector数组中具有相同的idx值
        first_index = first_and_last_indices_vector[cp].first;
        last_index = first_and_last_indices_vector[cp].second;
        x_Sum = 0;
        y_Sum = 0;
        z_Sum = 0;
        for (unsigned int li = first_index; li < last_index; ++li)
        {
            x_Sum += InputCloudPoint->points[index_vector[li].cloud_point_index].x;
            y_Sum += InputCloudPoint->points[index_vector[li].cloud_point_index].y;
            z_Sum += InputCloudPoint->points[index_vector[li].cloud_point_index].z;
        }
        PointCloud.x = x_Sum / (last_index - first_index);
        PointCloud.y = y_Sum / (last_index - first_index);
        PointCloud.z = z_Sum / (last_index - first_index);
        OutPointCloud->push_back(PointCloud);
    }
}

}  // namespace lidar
}  // namespace perception