#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

class GridMap {
public:
    GridMap(ros::NodeHandle& nh);
    ~GridMap() {};
    void reset();

    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(int& x, int& y, int& z);
    inline void boundIndex(Eigen::Vector3i& id);
    inline bool isInMap(const Eigen::Vector3d& pos);
    inline bool isInMap(const Eigen::Vector3i& idx);

    inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
    inline void setOccupancyInflate(Eigen::Vector3d pos, double occ = 1);
    inline void setOccupancyInflateExtra(Eigen::Vector3d pos, double occ = 1);
    inline int getOccupancy(Eigen::Vector3d pos);
    inline int getOccupancyInflate(Eigen::Vector3d pos);
    inline int getOccupancyInflateExtra(Eigen::Vector3d pos);
    inline bool isOccupied(const Eigen::Vector3i& id);
    inline bool isOccupiedInflate(const Eigen::Vector3i& id);
    inline bool isOccupiedInflateExtra(const Eigen::Vector3i& id);

    inline void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
    inline double getResolution();
    inline Eigen::Vector3d getOrigin();
    inline Eigen::Matrix<int, 3, 1> getVoxelDim();
    inline void getMapInflateData(std::vector<signed char>& data);
    inline void getMapInflateData(Eigen::Matrix3Xd& points);

    typedef std::shared_ptr<GridMap> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    ros::NodeHandle node;
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
    double local_detect_range_xy, local_detect_range_z;
    Eigen::Vector3i map_voxel_num_;                        // map range in index
    Eigen::Vector3i local_map_bound_;
    double resolution_, resolution_inv_;
    double radius_inflation_, extra_inflation_;
    double visualization_truncate_height_, virtual_ceil_height_, ground_height_;
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud_inflate, cloud_inflate_extra;
    bool has_global_map;
    Eigen::Vector3d uav_pos;
    string world_frame_id;

    std::vector<double> occupancy_buffer_;
    std::vector<signed char> occupancy_buffer_inflate_;
    std::vector<signed char> occupancy_buffer_inflate_extra_;

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void visCallback(const ros::TimerEvent& /*event*/);   // visualize local inflated map and local map

    inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
    void simplifyPointClouds();

    ros::Subscriber global_cloud_sub_, uav_pose_sub_;
    ros::Publisher local_map_pub_, local_map_inf_pub_;
    ros::Timer vis_timer_;

};

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - map_origin_(i)) * resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * resolution_ + map_origin_(i);
}

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
    return id(0) * map_voxel_num_(1) * map_voxel_num_(2) + id(1) * map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
    return x * map_voxel_num_(1) * map_voxel_num_(2) + y * map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), map_voxel_num_(2) - 1), 0);
    id = id1;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
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

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
        return false;
    }
    if (idx(0) > map_voxel_num_(0) - 1 || idx(1) > map_voxel_num_(1) - 1 ||
        idx(2) > map_voxel_num_(2) - 1) {
        return false;
    }
    return true;
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
    if (occ != 1 && occ != 0) {
        cout << "occ value error!" << endl;
        return;
    }
    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    occupancy_buffer_[toAddress(id)] = occ;
}

inline void GridMap::setOccupancyInflate(Eigen::Vector3d pos, double occ) {
    if (occ != 1 && occ != 0) {
        cout << "occ value error!" << endl;
        return;
    }
    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    occupancy_buffer_inflate_[toAddress(id)] = occ;
}

inline void GridMap::setOccupancyInflateExtra(Eigen::Vector3d pos, double occ) {
    if (occ != 1 && occ != 0) {
        cout << "occ value error!" << endl;
        return;
    }
    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    occupancy_buffer_inflate_extra_[toAddress(id)] = occ;
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    return occupancy_buffer_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline int GridMap::getOccupancyInflate(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    return occupancy_buffer_inflate_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline int GridMap::getOccupancyInflateExtra(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    return occupancy_buffer_inflate_extra_[toAddress(id)] > 0.5 ? 1 : 0;
}

inline bool GridMap::isOccupied(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);
    return occupancy_buffer_[adr] == 1;
}

inline bool GridMap::isOccupiedInflate(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);
    return occupancy_buffer_inflate_[adr] == 1;
}

inline bool GridMap::isOccupiedInflateExtra(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);
    return occupancy_buffer_inflate_extra_[adr] == 1;
}

inline void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size) {
    ori = map_origin_, size = map_size_;
}

inline double GridMap::getResolution() {
    return resolution_;
}

inline Eigen::Vector3d GridMap::getOrigin() {
    return map_origin_;
}

inline Eigen::Matrix<int, 3, 1> GridMap::getVoxelDim(){
    return map_voxel_num_;
}

inline void GridMap::getMapInflateData(std::vector<signed char>& data){
    data = occupancy_buffer_inflate_;
}

inline void GridMap::getMapInflateData(Eigen::Matrix3Xd& points){
    int map_pts = cloud_inflate.points.size();
    points.resize(Eigen::NoChange, map_pts);
    for(int i=0; i<map_pts; i++){
        pcl::PointXYZ pt = cloud_inflate.points[i];
        points.block<3,1>(0,i) = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
    int num = 0;
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -step; z <= step; ++z) {
                pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
            }
}

#endif