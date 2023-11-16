#include "grid_map_planner/planner_interface.h"

GridMapPlanner::GridMapPlanner(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    nh_.param("grid_map_planner/planner_type", planner_type, 1);   // 0: A-star, 1: JPS, 2: Dynamic-A-star
    plan_succeed = false;
    map_ptr.reset(new GridMap(nh_));
    if(planner_type == 0 || planner_type == 1){
        jps_planner_ptr = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(true));
        jps_map_util = std::make_shared<VoxelMapUtil>();
        jps_planner_ptr->setMapUtil(jps_map_util);
        std::vector<signed char> map_data;
        double res = map_ptr->getResolution();
        Eigen::Vector3d orig = map_ptr->getOrigin();
        Eigen::Vector3i voxel_dim = map_ptr->getVoxelDim();
        map_ptr->getMapInflateExtraData(map_data);
        ROS_INFO("JPS used map info: origin (%f,%f,%f), voxel_dim (%d,%d,%d), map_data_size (%ld), resolution (%f)", 
                    orig(0), orig(1), orig(2), voxel_dim(0), voxel_dim(1), voxel_dim(2), map_data.size(), res);
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
    ROS_INFO("Begin path planning using planner of type %d.", planner_type);
    if(planner_type == 0){
        jps_planner_ptr->updateMap();
        ROS_INFO("Map update done.");
        plan_succeed = jps_planner_ptr->plan(start, end, 1, false);
    }
    else if(planner_type == 1){
        jps_planner_ptr->updateMap();
        ROS_INFO("Map update done.");
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