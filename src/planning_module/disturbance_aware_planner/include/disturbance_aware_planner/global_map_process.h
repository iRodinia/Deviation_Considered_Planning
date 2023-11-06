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

    void getReplanInfo(const Eigen::Vector3d cur_pos, std::vector<double>& time_alloc, 
                        std::vector<Eigen::MatrixX4d>& polygons, Eigen::Vector3d& goal_pos);
    inline bool isProcessDone();

    std::vector<Eigen::Vector3d> ref_path;
    std::vector<double> ref_time_alloc;
    std::vector<Polytope> ref_polygons;
    int seg_num;
    typedef std::shared_ptr<GlobalMapProcessor> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    ros::Publisher global_ref_path_pub;
    ros::Publisher global_polygons_pub;
    ros::Timer visTimer;

    GridMap::Ptr map_ptr;
    GridMapPlanner::Ptr path_planner_ptr;
    double uav_vel;
    double pred_T;
    bool path_generated = false;
    bool sfcs_generated = false;

    void planRefPath(const Eigen::Vector3d& start_p, const Eigen::Vector3d& end_p);
    void planPolygons();
    void visualizationCb(const ros::TimerEvent& /*event*/);
};

inline bool GlobalMapProcessor::isProcessDone(){
    return sfcs_generated;
}

}

#endif