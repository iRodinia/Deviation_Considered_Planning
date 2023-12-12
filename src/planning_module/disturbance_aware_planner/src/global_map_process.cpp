#include "disturbance_aware_planner/global_map_process.h"

using namespace disturbance_aware_planner;

GlobalMapProcessor::GlobalMapProcessor(ros::NodeHandle& nh){
    path_planner_ptr.reset(new GridMapPlanner(&nh));
    map_ptr = path_planner_ptr->getGridMap();
    double plan_freq, vis_freq, pred_dt;
    int pred_N;
    nh.param("Task/start_pos_x", start_pos(0), 0.0);
    nh.param("Task/start_pos_y", start_pos(1), 0.0);
    nh.param("Task/hover_height", start_pos(2), 1.0);
    nh.param("Task/goal_pos_x", goal_pos(0), 1.0);
    nh.param("Task/goal_pos_y", goal_pos(1), 1.0);
    nh.param("Task/goal_pos_z", goal_pos(2), 1.0);
    nh.param("Model/nominal_vel", uav_vel, 1.2);
    nh.param("global_map_process/plan_path_sfc_frequency", plan_freq, 2.0);
    nh.param("global_map_process/visualization_frequency", vis_freq, 2.5);
    global_ref_path_pub = nh.advertise<visualization_msgs::Marker>("global_map_process/global_reference_path", 10);
    global_polygons_pub = nh.advertise<visualization_msgs::MarkerArray>("global_map_process/global_polygons", 10);
    planTimer = nh.createTimer(ros::Rate(plan_freq), &GlobalMapProcessor::globalPlanCb, this);
    visTimer = nh.createTimer(ros::Rate(vis_freq), &GlobalMapProcessor::visualizationCb, this);
}

void GlobalMapProcessor::planRefPath(const Eigen::Vector3d& start_p, const Eigen::Vector3d& end_p){
    if(path_generated){
        return;
    }
    if(path_planner_ptr->planPath(start_p, end_p)){
        ref_path = path_planner_ptr->getPath();

        /*Test Only!*/
        // std::cout << "Reference path generated with size: " << ref_path.size() << std::endl;
        // for(int i=0; i<ref_path.size(); i++){
        //     std::cout << "[" << ref_path[i].transpose() << "]" << std::endl;
        // }
        ROS_INFO("Global reference path generated with size: %ld", ref_path.size());
        seg_num = ref_path.size() - 1;
        ref_time_alloc.resize(seg_num);
        ref_polygons.resize(seg_num);
        for(int i=0; i<seg_num; i++){
            double seg_len = (ref_path[i+1]-ref_path[i]).norm();
            ref_time_alloc[i] = seg_len / uav_vel;
            if(i == 0 || i == seg_num-1){
                if(seg_len <= 1.0){
                    ref_time_alloc[i] *= 1.6;
                }
            }
        }
        path_generated = true;
    }
    else{
        ROS_INFO("Unable to plan global reference path.");
        path_generated = false;
    }
}

void GlobalMapProcessor::planPolygons(){
    if(sfcs_generated){
        return;
    }
    if(!path_generated){
        ROS_INFO("Try to generate SFCs before path generated. Not allowed!");
        sfcs_generated = false;
        return;
    }
    Eigen::Vector3d map_origion, map_size;
    map_ptr->getRegion(map_origion, map_size);
    Eigen::MatrixX4d map_bnd_consts(6, 4);   // map boundary constraints (6 faces)
    map_bnd_consts.row(0) = Eigen::RowVector4d(-1, 0, 0, map_origion(0));
    map_bnd_consts.row(1) = Eigen::RowVector4d(0, -1, 0, map_origion(1));
    map_bnd_consts.row(2) = Eigen::RowVector4d(0, 0, -1, map_origion(2));
    map_bnd_consts.row(3) = Eigen::RowVector4d(1, 0, 0, -map_origion(0)-map_size(0));
    map_bnd_consts.row(4) = Eigen::RowVector4d(0, 1, 0, -map_origion(1)-map_size(1));
    map_bnd_consts.row(5) = Eigen::RowVector4d(0, 0, 1, -map_origion(2)-map_size(2));
    Eigen::Matrix3Xd obs_pts;
    map_ptr->getMapInflateData(obs_pts);
    bool tmp_flag;
    for(int i=0; i<seg_num; i++){
        Eigen::Vector3d tmp_s, tmp_e;
        tmp_s = ref_path[i];
        tmp_e = ref_path[i+1];
        Polytope tmp_poly;
        tmp_flag = emvp::emvp(map_bnd_consts, obs_pts, tmp_s, tmp_e, tmp_poly, 0);
        if(!tmp_flag){
            ROS_WARN("SFC calculation failed! Force return.");
            break;
        }
        ref_polygons[i] = tmp_poly;
    }
    sfcs_generated = tmp_flag;
};

void GlobalMapProcessor::globalPlanCb(const ros::TimerEvent& /*event*/){
    if(!path_generated){
        ROS_INFO("Try to plan global path from (%f, %f, %f) to (%f, %f, %f)", start_pos(0), start_pos(1), start_pos(2), goal_pos(0), goal_pos(1), goal_pos(2));
        planRefPath(start_pos, goal_pos);
        ROS_INFO("Finish one path plan iteration. Current status: %s", path_generated ? "true" : "false");
    }
    else if(!sfcs_generated){
        ROS_INFO("Try to plan global sfc.");
        planPolygons();
        ROS_INFO("Finish one sfc plan iteration. Current status: %s", sfcs_generated ? "true" : "false");
    }
}

void GlobalMapProcessor::visualizationCb(const ros::TimerEvent& /*event*/){
    if(path_generated){
        visualization_msgs::Marker line_strips;
        line_strips.header.frame_id = "world";
        line_strips.type = visualization_msgs::Marker::LINE_STRIP;
        line_strips.id = 0;
        line_strips.scale.x = 0.03;
        line_strips.pose.orientation.w = 1.0;
        line_strips.color.r = 1.0;
        line_strips.color.a = 1.0;
        geometry_msgs::Point p;
        for(int i=0; i<=seg_num; i++){
            p.x = ref_path[i](0);
            p.y = ref_path[i](1);
            p.z = ref_path[i](2);
            line_strips.points.push_back(p);
        }
        global_ref_path_pub.publish(line_strips);
    }
    if(sfcs_generated){
        for(int j=0; j<seg_num; j++){
            string _ns = "Polygon_" + to_string(j+1);
            if(j == 0){
                ref_polygons[j].Visualize(global_polygons_pub, _ns, true);
            }
            ref_polygons[j].Visualize(global_polygons_pub, _ns, false);
        }
    }
}

double point2LineDist(const Eigen::Vector3d p, const Eigen::Vector3d l1, const Eigen::Vector3d l2, Eigen::Vector3d& near_point){
    double line_len = (l1 - l2).norm();
    if(line_len < 1e-6){
        near_point = l1;
        return (p - l1).norm();
    }
    double t = (p - l1).dot((l2 - l1)) / (line_len * line_len);
    if(t < 0){
        near_point = l1;
        return (p - l1).norm();
    }
    else if(t > 1){
        near_point = l2;
        return (p - l2).norm();
    }
    else{
        near_point = l1 + t*(l2 - l1);
        return (p - near_point).norm();
    }
}

void GlobalMapProcessor::getReplanInfo(const Eigen::Vector3d cur_pos, double pred_T, std::vector<double>& time_alloc, 
                                    std::vector<Eigen::MatrixX4d>& polygons, Eigen::Vector3d& goal_pos,
                                    Eigen::Vector3d& goal_vel){
    if(!sfcs_generated){
        return;
    }
    std::vector<double> line_dists(seg_num);
    std::vector<Eigen::Vector3d> near_pts(seg_num);
    for(int i=0; i<seg_num; i++){
        Eigen::Vector3d _pt;
        line_dists[i] = point2LineDist(cur_pos, ref_path[i], ref_path[i+1], _pt);
        near_pts[i] = _pt;
    }
    int min_seg = int(min_element(line_dists.begin(), line_dists.end()) - line_dists.begin());
    Eigen::Vector3d near_pt = near_pts[min_seg];
    if((near_pt - ref_path[min_seg+1]).norm() < 1e-6){
        if(min_seg < seg_num - 1){
            min_seg++;
        }
    }
    ROS_INFO("uav current pos: (%f, %f, %f)", cur_pos(0), cur_pos(1), cur_pos(2));
    ROS_INFO("nearest point on ref path[%d]: (%f, %f, %f)", min_seg, near_pt(0), near_pt(1), near_pt(2));
    if(time_alloc.size() > 0) time_alloc.resize(0);

    double time_remain = pred_T;
    int seg_idx = min_seg;
    double seg_ratio = (cur_pos-ref_path[min_seg+1]).norm() / (ref_path[min_seg]-ref_path[min_seg+1]).norm();
    double seg_time_remain = seg_ratio * ref_time_alloc[min_seg];
    if(seg_time_remain > time_remain){
        time_alloc.push_back(time_remain);
        goal_pos = cur_pos + (ref_path[min_seg+1]-cur_pos)*pred_T/seg_time_remain;
        goal_vel = (goal_pos - cur_pos).normalized() * uav_vel;
    }
    else{
        time_alloc.push_back(seg_time_remain);
    }
    time_remain -= seg_time_remain;
    seg_idx++;
    while(time_remain > 0 && seg_idx < seg_num){
        seg_time_remain = ref_time_alloc[seg_idx];
        if(seg_time_remain > time_remain){
            time_alloc.push_back(time_remain);
            goal_pos = ref_path[seg_idx] + (ref_path[seg_idx+1]-ref_path[seg_idx])*time_remain/seg_time_remain;
            goal_vel = (ref_path[seg_idx+1]-ref_path[seg_idx]).normalized() * uav_vel;
        }
        else{
            time_alloc.push_back(seg_time_remain);
        }
        time_remain -= seg_time_remain;
        seg_idx++;
    }
    if(time_remain >= 0 && seg_idx == seg_num){
        *(time_alloc.end()-1) += time_remain;
        goal_pos = ref_path[seg_num];
        goal_vel = Eigen::Vector3d::Zero();
        time_remain = 0;
    }

    polygons.resize(time_alloc.size());
    for(int i=0; i<time_alloc.size(); i++){
        polygons[i] = ref_polygons[min_seg+i].GetPlanes();
    }
}