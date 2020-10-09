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

void CVCCluster::clustering(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<std::shared_ptr<base::Object>> &objects){
    std::vector<PointAPR> papr;
    std::unordered_map<int, Voxel> hvoxel;
    std::vector<int> cluster_index;
    objects.clear();
    if(srcCloudPoints->size() <= 0)
        return;
    calculateAPR(srcCloudPoints, papr);
    buildHashTable(papr, hvoxel);
    cluster_index = CVC(hvoxel, papr);
    computeCentroid(srcCloudPoints, cluster_index, 9, 5000, &objects);
}

void CVCCluster::calculateAPR(const PCLPointCloud::Ptr &srcCloudPoints, std::vector<PointAPR>& vapr){
    for (size_t i = 0; i < srcCloudPoints->points.size(); ++i){
           PointAPR par;
           par.polar_angle = polarAngleCalculate(srcCloudPoints->points[i].x, srcCloudPoints->points[i].y);
           par.range = sqrt(srcCloudPoints->points[i].x * srcCloudPoints->points[i].x + \
                            srcCloudPoints->points[i].y * srcCloudPoints->points[i].y);
           par.azimuth = static_cast<float>(atan2(srcCloudPoints->points[i].z, par.range));
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

void CVCCluster::buildHashTable(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out){
    std::vector<int> ri;
    std::vector<int> pi;
    std::vector<int> ai;
    for(int i =0; i< vapr.size(); ++i){
        int azimuth_index = round(((vapr[i].azimuth-minazimuth)*180/PI)/deltaA);
        int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
        int range_index = round((vapr[i].range-minrange)/deltaR);
        int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);
        ri.push_back(range_index);
        pi.push_back(polar_index);           
        ai.push_back(azimuth_index);
        std::unordered_map<int, Voxel>::iterator it_find;
        it_find = map_out.find(voxel_index);
        if (it_find != map_out.end()){
            it_find->second.index.push_back(i);
        }else{
            Voxel vox;
            vox.haspoint =true;
            vox.index.push_back(i); 
            vox.index.swap(vox.index);
            map_out.insert(std::make_pair(voxel_index, vox));
        }
    }
    auto maxPosition = std::max_element(ai.begin(), ai.end());
    auto maxPosition1 = std::max_element(ri.begin(), ri.end());
    auto maxPosition2 = std::max_element(pi.begin(), pi.end());
    std::cout << *maxPosition << " " << *maxPosition1 << " " << *maxPosition2 << std::endl;
}

std::vector<int>  CVCCluster::CVC(std::unordered_map<int, Voxel> &map_in, const std::vector<PointAPR>& vapr){
    int current_cluster = 0;
    std::vector<int> cluster_indices = std::vector<int>(vapr.size(), -1);

    for(int i = 0; i < vapr.size(); ++i){
        if (cluster_indices[i] != -1)
		    continue;
        int azimuth_index = round((vapr[i].azimuth+fabs(minazimuth))*180/PI/deltaA);
        int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
        int range_index = round((vapr[i].range-minrange)/deltaR);
        int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);
           
        std::unordered_map<int, Voxel>::iterator it_find;
        std::unordered_map<int, Voxel>::iterator it_find2;
        it_find = map_in.find(voxel_index);
        std::vector<int> neightbors;

        if (it_find != map_in.end()){
            std::vector<int> neighborid;
            findNeighbors(polar_index, range_index, azimuth_index, neighborid);
            for (int k = 0; k < neighborid.size(); ++k){        
                it_find2 = map_in.find(neighborid[k]);
                if (it_find2 != map_in.end()){
                    for(int j = 0 ; j < it_find2->second.index.size(); ++j){
                        neightbors.push_back(it_find2->second.index[j]);
                    }
                }
            }
        }
       
        neightbors.swap(neightbors);

        if(neightbors.size()>0){
            for(int j =0 ; j<neightbors.size(); ++j){
                int oc = cluster_indices[i] ;
                int nc = cluster_indices[neightbors[j]];
                if (oc != -1 && nc != -1) {
                    if (oc != nc)
                        mergeClusters(cluster_indices, oc, nc);
				}
		        else {
                    if (nc != -1) {
                        cluster_indices[i] = nc;
                    }
                    else {
                        if (oc != -1) {
                            cluster_indices[neightbors[j]] = oc;
                        }
                    }
			    }
            }
        }
                          
 		if (cluster_indices[i] == -1) {
		    current_cluster++;
			cluster_indices[i] = current_cluster;
            for(int s =0 ; s<neightbors.size(); ++s){             
                cluster_indices[neightbors[s]] = current_cluster;
		   }
        }
    }
	return cluster_indices;
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

void CVCCluster::findNeighbors(const int polar, const int range, const int azimuth, std::vector<int>& neighborindex){
    for (int z = azimuth - 1; z <= azimuth + 1; z++){
		if (z < 0 || z >round((maxazimuth-minazimuth)*180/PI/deltaA)){
			continue;
		}

		for (int y = range - 1; y <= range + 1; y++){
			if (y < 0 || y >round((50-minrange)/deltaR)){
				continue;
			}

			for (int x = polar - 1; x <= polar + 1; x++){
                int px = x;
				if (x < 0 ){
					px=300;
				}
                if(x>300){
                    px=0;
                }
				neighborindex.push_back((px*(length+1)+y)+z*(length+1)*(width+1));
            }
        }
    }
}

void CVCCluster::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2){
    for (size_t i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}

}  // namespace lidar
}  // namespace perception