#ifndef GLOBAL_MAP_PROCESS
#define GLOBAL_MAP_PROCESS

#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>

#include "grid_map/grid_map.h"

namespace disturbance_awar_manager{

class GlobalMapProcessor{
public:
    GlobalMapProcessor(ros::NodeHandle& nh);
    ~GlobalMapProcessor() {};

    void getSegmentAndTime(const Eigen::Vector3d cur_pos, std::vector<double>& time_alloc, std::vector<Eigen::MatrixX4d>& polygons);

    std::vector<Eigen::Vector3d>* ref_path;
    std::vector<double>* ref_time_alloc;
    std::vector<Eigen::MatrixX4d>* ref_polygons;
    typedef std::shared_ptr<GlobalMapProcessor> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    ros::Subscriber global_map_sub;
    ros::Publisher globa_ref_path_pub;
    ros::Publisher global_polygons_pub;

    GridMap::Ptr map_ptr;
    bool map_initilized = false;

    void globalPclSubCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void planRefPath();

};

}

#endif