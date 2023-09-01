#ifndef GRID_PLANNER
#define GRID_PLANNER

#include <algorithm>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner.h>

#include "grid_map/grid_map.h"

using namespace JPS;

class GridMapPlanner {
public:
    GridMapPlanner(ros::NodeHandle* nodehandle);
    ~GridMapPlanner(){};
    bool planPath(Eigen::Vector3d start, Eigen::Vector3d end);
    std::vector<Eigen::Vector3d> getPath();

    std::shared_ptr<GridMapPlanner> Ptr;

private:
    ros::NodeHandle nh_;
    bool plan_succeed;
    std::shared_ptr<VoxelMapUtil> map_util;
    std::unique_ptr<JPSPlanner3D> planner_ptr;
    GridMap::Ptr map_ptr;

};

#endif