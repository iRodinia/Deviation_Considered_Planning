#include "disturbance_loader/disturb_loader.h"

DisturbMap::DisturbMap(){
    initialized = false;
    has_map = false;
}

void DisturbMap::initSettings(const ros::NodeHandle nh){
    double x_size, y_size, z_size, ground_height_;
    nh.param("FanDisturbance/resolution", resolution_, 0.05);
    nh.param("FanDisturbance/map_size_x", x_size, 5.0);
    nh.param("FanDisturbance/map_size_y", y_size, 5.0);
    nh.param("FanDisturbance/map_size_z", z_size, 2.5);
    nh.param("FanDisturbance/ground_height", ground_height_, -0.05);
    resolution_inv_ = 1 / resolution_;
    map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, ground_height_);
    map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
    for (int i = 0; i < 3; ++i)
        map_voxel_num_(i) = ceil(map_size_(i) / resolution_);

    map_min_boundary_ = map_origin_;
    map_max_boundary_ = map_origin_ + map_size_;
    buffer_size = map_voxel_num_(0) * map_voxel_num_(1) * map_voxel_num_(2);
    disturb_buffer_ = std::vector<float>(buffer_size, 0.0);
    initialized = true;
    has_map = false;
}

void DisturbMap::loadMap(const std::vector<float> buffer){
    if(!initialized){
        ROS_INFO("Disturbance map not initialized.");
        return;
    }
    if(buffer.size() != buffer_size){
        ROS_INFO("Incompatible buffer size! [%ld] reveived but [%d] expected.", buffer.size(), buffer_size);
        return;
    }
    disturb_buffer_.assign(buffer.begin(), buffer.end());
    has_map = true;
}

bool DisturbMap::isInited(){
    return initialized;
}

bool DisturbMap::hasMap(){
    return has_map;
}

double DisturbMap::getRatio_idx(Eigen::Vector3i idx){
    if(!initialized || !isInMap(idx)){
        return 0;
    }
    return disturb_buffer_[toAddress(idx)];
}

double DisturbMap::getRatio(Eigen::Vector3d pos){
    if(!initialized || !isInMap(pos)){
        return 0;
    }
    Eigen::Vector3i near_vox;
    posToIndex(pos, near_vox);
    Eigen::Vector3d near_pt;
    indexToPos(near_vox, near_pt);
    Eigen::Vector3d d_pos = pos - near_pt;
    double vertexes[2][2][2];   // trilinear interpolation
    for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++){
                vertexes[i][j][k] = getRatio_idx(near_vox + Eigen::Vector3i(i, j, k));
            }
    double u = d_pos(0) / resolution_;
    double v = d_pos(1) / resolution_;
    double w = d_pos(2) / resolution_;
    double accum = 0.0;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			for (int k = 0; k < 2; k++)
				accum += (i*u + (1 - i)*(1 - u)) * 
						 (j*v + (1 - j)*(1 - v)) * 
						 (k*w + (1 - k)*(1 - w)) * 
						 vertexes[i][j][k];
	return accum;
}
