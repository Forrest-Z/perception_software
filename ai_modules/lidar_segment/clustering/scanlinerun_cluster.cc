#include "lidar_segment/clustering/scanlinerun_cluster.h"
#include <cmath>

#define MIN_CLUSTER_SIZE 5
#define MAX_CLUSTER_SIZE 5000

namespace perception {
namespace lidar {

ScanLineRunCluster::ScanLineRunCluster(){
    sensorModel = 40;
    runThreshold = 0.5f;
    mergeThreshold = 1.0f;
    // Init runs, idx 0 for interest point, and idx for ground points
    maxLabel = 0;
    runs.push_back(dummy);
    for(size_t loop = 0; loop < sensorModel; loop++)
    {
        std::vector<int> temp;
        temp.clear();
        pointsIndex.push_back(temp);
    }
}

ScanLineRunCluster::~ScanLineRunCluster(){

}

void ScanLineRunCluster::clustering(const pcl::PointCloud<PointXYZIRL>::Ptr &srcPointCloud, std::vector<std::shared_ptr<base::Object>> &objects){
    objects.clear();
    // Clear runs in the previous scan.
    maxLabel = 0;
    if(!runs.empty())
    {
        runs.clear();
        runs.push_back(dummy);// dummy for index `0`
    }
    clusteringToSegment(srcPointCloud);
    for(size_t loop = 1; loop < runs.size(); loop++)
    {
        if(!runs[loop].empty())
        {
            base::ObjectPtr object(new base::Object);
            PCLPointCloud::Ptr tempCloud(new PCLPointCloud);
            double tempX = 0;
            double tempY = 0;
            double tempZ = 0;
            int count = 0;
            tempCloud->header = srcPointCloud->header;
            for(auto &point: runs[loop])
            {
                PCLPoint tempPoint;
                tempPoint.x = point->x;
                tempPoint.y = point->y;
                tempPoint.z = point->z;
                tempPoint.intensity = point->intensity;
                tempX += tempPoint.x;
                tempY += tempPoint.y;
                tempZ += tempPoint.z;
                tempCloud->push_back(tempPoint);
                count++;
            }
            if(count >= MIN_CLUSTER_SIZE && count <= MAX_CLUSTER_SIZE)
            {
                object->anchor_point[0] = tempX / static_cast<double>(count);
                object->anchor_point[1] = tempY / static_cast<double>(count);
                object->anchor_point[2] = tempZ / static_cast<double>(count);
                object->lidar_supplement.num_points_in_roi = count;
                pclToAttributePointCloud(tempCloud, object->lidar_supplement.cloud);
                objects.push_back(object);
                //std::cout << "input centroid:" << object->anchor_point[0] << " " << object->anchor_point[1] << std::endl;
            }
        }
    }
}

void ScanLineRunCluster::clusteringToSegment(const pcl::PointCloud<PointXYZIRL>::Ptr &srcPointCloud){
    // Init LiDAR frames with vectors and points
    PointXYZIRL tempPoint;
    tempPoint.intensity = -1;// Means unoccupy by any points
    std::vector<PointXYZIRL> laserRow(1801, tempPoint);
    laserFrame = std::vector<std::vector<PointXYZIRL> >(sensorModel, laserRow);

    // Init non-ground index holder.
    for(size_t loop = 0; loop < sensorModel; loop++)
    {
        pointsIndex[loop].clear();
    }

    // Organize Pointcloud in scanline
    float range = 0;
    int row = 0;
    for(auto &point: srcPointCloud->points)
    {
        if(point.ring < sensorModel)
        {
            // Compute and angle
            // In this case, `x` points right and `y` points forward.
            range = std::sqrt(point.x * point.x + point.y * point.y);
            if(point.x >= 0)
            {
                row = int(450 - asin(point.y / range) / 0.003490658);
            }
            else if(point.x < 0 && point.y <= 0)
            {
                row = int(1350 + asin(point.y / range) / 0.003490658);
            }
            else
            {
                row = int(1350 + asin(point.y / range) / 0.003490658);
            }

            if(row > 1800 || row < 0)
            {
                std::cerr << "point row error:" << row << std::endl;
                return;
            }
            else
            {
                laserFrame[point.ring][row] = point;
            }
            pointsIndex[point.ring].push_back(row);
        }
    }

    for(size_t loop = 0; loop < sensorModel; loop++)
    {
        findRuns(loop);
        updateLabels(loop);
    }
}

void ScanLineRunCluster::findRuns(size_t scanLine){
    size_t pointSize = pointsIndex[scanLine].size();

    if(pointSize <= 0)
        return;

    int startIndex = pointsIndex[scanLine][0]; // The first non ground point
    int lastIndex = pointsIndex[scanLine][pointSize - 1]; // The last non ground point

    /* Iterate all non-ground points, and compute and compare the distance
    of each two continous points. At least two non-ground points are needed.
    */
    for(size_t loop = 0; loop < pointSize - 1; loop++)
    {
        int i = pointsIndex[scanLine][loop];
        int i1 = pointsIndex[scanLine][loop+1];

        if(loop == 0)
        {
            // The first point, make a new run.
            auto &p_0 = laserFrame[scanLine][i];
            maxLabel += 1;
            runs.push_back(dummy);
            laserFrame[scanLine][i].index = maxLabel;
            runs[p_0.index].insert_after(runs[p_0.index].cbefore_begin(), &laserFrame[scanLine][i]);

            if(p_0.index == 0)
            {
                std::cerr << "point0 index==0" << std::endl;
            }
        }

        // Compare with the next point
        auto &p_i = laserFrame[scanLine][i];
        auto &p_i1 = laserFrame[scanLine][i1];
        float distance = point2dDistance<PointXYZIRL>(p_i, p_i1);

        /* If cur point is not ground and next point is within threshold,
        then make it the same run.
           Else, to make a new run.
        */
        if(distance < runThreshold)
        {
            p_i1.index = p_i.index;
        }
        else
        {
            maxLabel += 1;
            p_i1.index = maxLabel;
            runs.push_back(dummy);
        }

        // Insert the index.
        runs[p_i1.index].insert_after(runs[p_i1.index].cbefore_begin(), &laserFrame[scanLine][i1]);

        if(p_i1.index == 0)
        {
            std::cerr << "point1 index==0" << std::endl;
        }
    }

    // Compare the last point and the first point, for laser scans is a ring.
    if(pointSize > 1)
    {
        auto &p_0 = laserFrame[scanLine][startIndex];
        auto &p_l = laserFrame[scanLine][lastIndex];
        float distance = point2dDistance<PointXYZIRL>(p_0, p_l);
        if(distance < runThreshold)
        {
            if(p_0.index == 0)
            {
                std::cerr << "Ring Merge to 0 label" << std::endl;
            }
            /// If next point is within threshold, then merge it into the same run.
            mergeRuns(p_l.index, p_0.index);
        }
    }
    else if(pointSize == 1)
    {
        // The only point, make a new run.
        auto &p_0 = laserFrame[scanLine][startIndex];
        maxLabel += 1;
        runs.push_back(dummy);
        laserFrame[scanLine][startIndex].index = maxLabel;
        runs[p_0.index].insert_after(runs[p_0.index].cbefore_begin(), &laserFrame[scanLine][startIndex]);
    }
}

void ScanLineRunCluster::updateLabels(size_t scanLine){
    // Iterate each point of this scan line to update the labels.
    size_t pointSize = pointsIndex[scanLine].size();

    if(pointSize == 0)
        return;

    // Iterate each point of this scan line to update the labels.
    for(size_t loop = 0; loop < pointSize; loop++)
    {
        int j = pointsIndex[scanLine][loop];

        auto &p_j = laserFrame[scanLine][j];

        // Runs above from scan line 0 to scan_line
        for(int l = scanLine - 1; l >= 0; l--)
        {
            if(pointsIndex[l].size() == 0)
                continue;

            // Smart index for the near enough point, after re-organized these points.
            int nn_idx = j;

            if(laserFrame[l][nn_idx].intensity == -1)
            {
                continue;
            }

            // Nearest neighbour point
            auto &p_nn = laserFrame[l][nn_idx];
            // Skip, if these two points already belong to the same run.
            if(p_j.index == p_nn.index)
            {
                continue;
            }

            float dist_min = point2dDistance<PointXYZIRL>(p_j, p_nn);

            if(dist_min < mergeThreshold)
            {
                int cur_label = 0;
                int target_label = 0;

                if(p_j.index ==0 || p_nn.index == 0)
                {
                    std::cerr << "p_j p_nn index==0" << std::endl;
                }
                // Merge to a smaller label cluster
                if(p_j.index > p_nn.index)
                {
                    cur_label = p_j.index;
                    target_label = p_nn.index;
                }
                else
                {
                    cur_label = p_nn.index;
                    target_label = p_j.index;
                }

                // Merge these two runs.
                mergeRuns(cur_label, target_label);
            }
        }
    }
}

void ScanLineRunCluster::mergeRuns(int currentLabel, int targetLabel){
    if(currentLabel == 0 || targetLabel == 0)
    {
        std::cerr << "Error merging runs" << std::endl;
    }
    // First, modify the label of current run.
    for(auto &point: runs[currentLabel])
    {
        point->index = targetLabel;
    }
    // Then, insert points of current run into target run.
    runs[targetLabel].insert_after(runs[targetLabel].cbefore_begin(), runs[currentLabel].begin(),runs[currentLabel].end() );
    runs[currentLabel].clear();
}

}  // namespace lidar
}  // namespace perception