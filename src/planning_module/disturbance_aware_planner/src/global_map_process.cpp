#include "disturbance_aware_planner/global_map_process.h"

using namespace disturbance_aware_planner;

GlobalMapProcessor::GlobalMapProcessor(ros::NodeHandle& nh){
    path_planner_ptr.reset(new GridMapPlanner(&nh));
    map_ptr = path_planner_ptr->getGridMap();
    while(!map_ptr->mapInitialized()){
        ros::Duration(0.5).sleep();
    }
    ros::Duration(0.5).sleep();
    double vis_freq, pred_dt;
    int pred_N;
    nh.param("Model/nominal_vel", uav_vel, 1.2);
    nh.param("global_map_process/visualization_frequency", vis_freq, 2.5);
    nh.param("Optimization/predict_num", pred_N, 100);
    nh.param("Optimization/predict_dt", pred_dt, 0.02);
    pred_T = pred_N * pred_dt;
    global_ref_path_pub = nh.advertise<visualization_msgs::Marker>("global_reference_path", 10);
    global_polygons_pub = nh.advertise<visualization_msgs::MarkerArray>("global_polygons", 10);
    visTimer = nh.createTimer(ros::Duration(1/vis_freq), &GlobalMapProcessor::visualizationCb, this);
}

void GlobalMapProcessor::planRefPath(const Eigen::Vector3d& start_p, const Eigen::Vector3d& end_p){
    if(path_generated){
        return;
    }
    if(path_planner_ptr->planPath(start_p, end_p)){
        ref_path = path_planner_ptr->getPath();
        seg_num = ref_path.size() - 1;
        ref_time_alloc.reserve(seg_num);
        ref_polygons.reserve(seg_num);
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
        ROS_INFO("Try to generate SFCs before path generated!");
        sfcs_generated = false;
        return;
    }
    Eigen::Vector3d map_origion, map_size;
    map_ptr->getRegion(map_origion, map_size);
    Eigen::MatrixX4d map_bnd_consts(6);   // map boundary constraints (6 faces)
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

void GlobalMapProcessor::visualizationCb(const ros::TimerEvent& /*event*/){
    if(path_generated){
        visualization_msgs::Marker line_strips;
        line_strips.type = visualization_msgs::Marker::LINE_STRIP;
        line_strips.id = 0;
        line_strips.scale.x = 0.1;
        line_strips.color.r = 1.0;
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
            ref_polygons[j].Visualize(global_polygons_pub, _ns);
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

void GlobalMapProcessor::getReplanInfo(const Eigen::Vector3d cur_pos, std::vector<double>& time_alloc, 
                                    std::vector<Eigen::MatrixX4d>& polygons, Eigen::Vector3d& goal_pos){
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
    time_alloc.reserve(seg_num - min_seg);
    double seg_ratio = (cur_pos-ref_path[min_seg+1]).norm() / (ref_path[min_seg]-ref_path[min_seg+1]).norm();
    time_alloc[0] = seg_ratio * ref_time_alloc[min_seg];
    for(int j=min_seg+1; j<seg_num; j++){
        time_alloc[j-min_seg] = ref_time_alloc[j];
    }
    polygons.reserve(seg_num - min_seg);
    for(int k=min_seg; k<seg_num; k++){
        polygons[k] = ref_polygons[k].GetPlanes();
    }
    double time_remain = 0;
    for(auto _t : time_alloc){
        time_remain += _t;
    }
    if(pred_T >= time_remain){
        goal_pos = ref_path[seg_num];
        time_alloc[seg_num - min_seg - 1] += pred_T - time_remain;
    }
    else{
        double goal_t = pred_T;
        int c = 0;
        while(goal_t > time_alloc[c]){
            goal_t -= time_alloc[c];
            c++;
        }
        if(c == 0){
            goal_pos = near_pt + (goal_t / time_alloc[0]) * (ref_path[min_seg+1] - near_pt);
        }
        else{
            goal_pos = ref_path[min_seg+c] + (goal_t / time_alloc[c]) * (ref_path[min_seg+c+1] - ref_path[min_seg+c]);
        }
    }
}