#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "disturbance_sources/DisturbRatio.h"

using namespace std;

class EllipseDisturb{
public:
    EllipseDisturb(ros::NodeHandle* node);
    ~EllipseDisturb() {};

private:
    ros::NodeHandle nh;
    ros::Publisher disturb_vis_pub;
    ros::ServiceServer get_disturb_ratio;
    ros::Timer timer1;

    Eigen::Vector3d source_p, source_dir, ellipse_center;
    double max_ratio;
    double range_a, range_r, bias_a;     // ellipse long axis a, short axises both r
    bool cloud_gen = false;
    sensor_msgs::PointCloud2 disturb_vis_msg;
    string world_frame;

    void genVisCloud();
    bool getDisturbRatioSrv(disturbance_sources::DisturbRatio::Request &req, disturbance_sources::DisturbRatio::Response &res);
    void timer1Cb(const ros::TimerEvent&);
};


#endif