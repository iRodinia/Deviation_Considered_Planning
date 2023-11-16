#ifndef GRID_PLANNER
#define GRID_PLANNER

#include <algorithm>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <jps_planner/jps_planner.h>
#include <dyn_a_star_planner/dyn_a_star.h>

#include "grid_map/grid_map.h"

using namespace JPS;

class GridMapPlanner {
public:
    GridMapPlanner(ros::NodeHandle* nodehandle);
    ~GridMapPlanner(){};
    GridMap::Ptr getGridMap();
    bool planPath(Eigen::Vector3d start, Eigen::Vector3d end);
    std::vector<Eigen::Vector3d> getPath();
    typedef std::shared_ptr<GridMapPlanner> Ptr;

private:
    ros::NodeHandle nh_;
    int planner_type;
    int init_count = 0;
    bool plan_succeed;
    GridMap::Ptr map_ptr;

    /* used for A-star planning or JPS planning */
    std::shared_ptr<VoxelMapUtil> jps_map_util;
    std::unique_ptr<JPSPlanner3D> jps_planner_ptr;
    
    /* used for Dynamic A-star planning */
    std::unique_ptr<AStar> astar_planner_ptr;
    double step_size;

    ros::Publisher debug_pub;
    ros::Timer debug_timer;

    void timerCb(const ros::TimerEvent&);
};

#endif