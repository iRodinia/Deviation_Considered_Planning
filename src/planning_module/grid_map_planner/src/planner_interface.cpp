#include "grid_map_planner/planner_interface.h"

GridMapPlanner::GridMapPlanner(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    nh_.param("grid_map_planner/planner_type", planner_type, 1);   // 0: A-star, 1: JPS, 2: Dynamic-A-star
    plan_succeed = false;
    map_ptr.reset(new GridMap(nh_));
    while(!map_ptr->mapInitialized()){
        init_count++;
        ROS_INFO("Grid map planner waiting for map initialization... (round %d)", init_count);
        ros::Duration(0.5).sleep();
    }
    if(planner_type == 0 || planner_type == 1){
        jps_planner_ptr = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(true));
        jps_map_util = std::make_shared<VoxelMapUtil>();
        std::vector<signed char> map_data;
        double res = map_ptr->getResolution();
        Eigen::Vector3d orig = map_ptr->getOrigin();
        Eigen::Matrix<int, 3, 1> voxel_dim = map_ptr->getVoxelDim();
        map_ptr->getMapInflateExtraData(map_data);
        jps_map_util->setMap(orig, voxel_dim, map_data, res);
    }
    else{
        astar_planner_ptr = std::unique_ptr<AStar>(new AStar);
        astar_planner_ptr->initGridMap(map_ptr, map_ptr->getVoxelDim());
        step_size = map_ptr->getResolution();
    }
    
    ROS_INFO("Grid Map Planner Initialized.");
}

GridMap::Ptr GridMapPlanner::getGridMap(){
    return map_ptr;
}

bool GridMapPlanner::planPath(Eigen::Vector3d start, Eigen::Vector3d end){
    if(!map_ptr->mapInitialized()){
        return false;
    }
    if(planner_type == 0){
        jps_planner_ptr->updateMap();
        plan_succeed = jps_planner_ptr->plan(start, end, 1, false);
    }
    else if(planner_type == 1){
        jps_planner_ptr->updateMap();
        plan_succeed = jps_planner_ptr->plan(start, end, 1, true);
    }
    else{
        plan_succeed = astar_planner_ptr->AstarSearch(step_size, start, end);
    }
    return plan_succeed;
}

std::vector<Eigen::Vector3d> GridMapPlanner::getPath(){
    if(!plan_succeed)
        return std::vector<Eigen::Vector3d>();
    if(planner_type == 0 || planner_type == 1){
        auto path = jps_planner_ptr->getPath();
        return std::vector<Eigen::Vector3d>(path.begin(), path.end());
    }
    else{
        return astar_planner_ptr->getPath();
    }
}