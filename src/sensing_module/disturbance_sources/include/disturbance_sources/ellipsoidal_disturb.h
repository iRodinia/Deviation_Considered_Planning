#ifndef _ELLIPSE_DISTURB_H
#define _ELLIPSE_DISTURB_H

#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <memory>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "flight_logger/logger.h"
#include "disturbance_sources/DisturbRatio.h"

using namespace std;

class EllipseDisturb{
public:
    EllipseDisturb(ros::NodeHandle* node);
    ~EllipseDisturb();

    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(int& x, int& y, int& z);
    void dumpParams();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    ros::NodeHandle nh;
    ros::Publisher disturb_map_pub, disturb_vis_pub;
    ros::ServiceServer get_disturb_ratio;
    ros::Timer timer1, timer2;

    int source_num;
    std::vector<Eigen::Vector3d> source_p, source_dir, ellipse_center;
    std::vector<double> range_a, range_r, bias_a;          // ellipse long axis a, short axises both r
    std::vector<Eigen::MatrixXd> trans_mat;
    double max_ratio;
    bool map_gen = false;
    bool cloud_gen = false;
    vector<sensor_msgs::PointCloud2> disturb_vis_msg_queue;
    string world_frame;
    int discret_num, pub_count;

    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
    Eigen::Vector3i map_voxel_num_;                        // map range in index
    double resolution_, resolution_inv_;
    std::vector<float> disturb_buffer_;

    bool log_enable;
    std::shared_ptr<FlightLogger> logger_ptr;
    ros::Time log_start_ts;

    void genDisturbMap();
    void genVisCloud();
    double getDisturbRatio_id(Eigen::Vector3d _pos, int fan_id);
    double getDisturbRatio(Eigen::Vector3d _pos);
    bool getDisturbRatioSrv(disturbance_sources::DisturbRatio::Request &req, disturbance_sources::DisturbRatio::Response &res);
    void timer1Cb(const ros::TimerEvent&);
    void timer2Cb(const ros::TimerEvent&);
};

inline void EllipseDisturb::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - map_origin_(i)) * resolution_inv_);
}

inline void EllipseDisturb::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * resolution_ + map_origin_(i);
}

inline int EllipseDisturb::toAddress(const Eigen::Vector3i& id) {
    return id(0) * map_voxel_num_(1) * map_voxel_num_(2) + id(1) * map_voxel_num_(2) + id(2);
}

inline int EllipseDisturb::toAddress(int& x, int& y, int& z) {
    return x * map_voxel_num_(1) * map_voxel_num_(2) + y * map_voxel_num_(2) + z;
}


#endif