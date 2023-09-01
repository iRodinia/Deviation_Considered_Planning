#include "grid_map_planner/planner_interface.h"

GridMapPlanner::GridMapPlanner(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    plan_succeed = false;
    map_util = std::make_shared<VoxelMapUtil>();
    planner_ptr = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(true));
    map_ptr = std::shared_ptr<GridMap>(new GridMap());

    map_ptr->initMap(nh_);
    ros::Duration(0.5).sleep();
    ROS_INFO("Grid Map Planner Initialized.");
}

bool GridMapPlanner::planPath(Eigen::Vector3d start, Eigen::Vector3d end){
    double res = map_ptr->getResolution();
    Eigen::Vector3d orig = map_ptr->getOrigin();
    Eigen::Matrix<int, 3, 1> voxel_dim = map_ptr->getVoxelDim();
    std::vector<signed char> map_data = map_ptr->getMapData();
    map_util->setMap(orig, voxel_dim, map_data, res);
    
    planner_ptr->setMapUtil(map_util);
    planner_ptr->updateMap();
    plan_succeed = planner_ptr->plan(start, end, 1, true);
    return plan_succeed;
}

std::vector<Eigen::Vector3d> GridMapPlanner::getPath(){
    if(!plan_succeed)
        return std::vector<Eigen::Vector3d>();
    auto path = planner_ptr->getPath();
    return std::vector<Eigen::Vector3d>(path.begin(), path.end());
}