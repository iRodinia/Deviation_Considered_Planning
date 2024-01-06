#ifndef _DISTURB_MAP_H
#define _DISTURB_MAP_H

#include <iostream>
#include <string>
#include <memory>
#include <Eigen/Eigen>

#include <ros/ros.h>

class DisturbMap {
public:
    DisturbMap();
    ~DisturbMap() {}
    void initSettings(const ros::NodeHandle nh);
    void loadMap(const std::vector<float> buffer);
    bool isInited();
    bool hasMap();
    double getRatio(Eigen::Vector3d pos);
    
    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(int& x, int& y, int& z);
    inline bool isInMap(const Eigen::Vector3i& idx);
    inline bool isInMap(const Eigen::Vector3d& pos);

    typedef std::shared_ptr<DisturbMap> Ptr;

private:
    double getRatio_idx(Eigen::Vector3i idx);

    bool initialized, has_map;
    int buffer_size;
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
    Eigen::Vector3i map_voxel_num_;                        // map range in index
    double resolution_, resolution_inv_;
    std::vector<float> disturb_buffer_;

};

inline void DisturbMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - map_origin_(i)) * resolution_inv_);
}

inline void DisturbMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * resolution_ + map_origin_(i);
}

inline int DisturbMap::toAddress(const Eigen::Vector3i& id) {
    return id(0) * map_voxel_num_(1) * map_voxel_num_(2) + id(1) * map_voxel_num_(2) + id(2);
}

inline int DisturbMap::toAddress(int& x, int& y, int& z) {
    return x * map_voxel_num_(1) * map_voxel_num_(2) + y * map_voxel_num_(2) + z;
}

inline bool DisturbMap::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
        return false;
    }
    if (idx(0) > map_voxel_num_(0) - 1 || idx(1) > map_voxel_num_(1) - 1 ||
        idx(2) > map_voxel_num_(2) - 1) {
        return false;
    }
    return true;
}

inline bool DisturbMap::isInMap(const Eigen::Vector3d& pos) {
    if (pos(0) < map_min_boundary_(0) + 1e-4 || pos(1) < map_min_boundary_(1) + 1e-4 ||
        pos(2) < map_min_boundary_(2) + 1e-4) {
        return false;
    }
    if (pos(0) > map_max_boundary_(0) - 1e-4 || pos(1) > map_max_boundary_(1) - 1e-4 ||
        pos(2) > map_max_boundary_(2) - 1e-4) {
        return false;
    }
    return true;
}

#endif