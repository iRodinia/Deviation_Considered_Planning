#ifndef GLOBAL_MAP_PROCESS
#define GLOBAL_MAP_PROCESS

#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "grid_map/grid_map.h"
#include "polytope/emvp.hpp"
#include "grid_map_planner/planner_interface.h"

namespace disturbance_aware_planner{

class GlobalMapProcessor{
public:
    GlobalMapProcessor(ros::NodeHandle& nh);
    ~GlobalMapProcessor() {};

    void getReplanInfo(const Eigen::Vector3d cur_pos, double pred_T, std::vector<double>& time_alloc, 
                        std::vector<Eigen::MatrixX4d>& polygons, Eigen::Vector3d& goal_pos, Eigen::Vector3d& goal_vel_dir);
    inline bool isPathGenerated();
    inline bool isSFCGenerated();

    std::vector<Eigen::Vector3d> ref_path;
    std::vector<double> ref_time_alloc;
    std::vector<Polytope> ref_polygons;
    int seg_num;
    typedef std::shared_ptr<GlobalMapProcessor> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    ros::Publisher global_ref_path_pub;
    ros::Publisher global_polygons_pub;
    ros::Timer planTimer, visTimer;

    GridMap::Ptr map_ptr;
    GridMapPlanner::Ptr path_planner_ptr;
    Eigen::Vector3d start_pos, goal_pos;
    double uav_vel;
    bool path_generated = false;
    bool sfcs_generated = false;

    void planRefPath(const Eigen::Vector3d& start_p, const Eigen::Vector3d& end_p);
    void planPolygons();
    void globalPlanCb(const ros::TimerEvent& /*event*/);
    void visualizationCb(const ros::TimerEvent& /*event*/);
};

inline bool GlobalMapProcessor::isPathGenerated(){
    return path_generated;
}

inline bool GlobalMapProcessor::isSFCGenerated(){
    return sfcs_generated;
}

}

#endif