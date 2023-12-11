#include "disturbance_sources/ellipsoidal_disturb.h"

EllipseDisturb::EllipseDisturb(ros::NodeHandle* node): nh(*node){
    Eigen::Vector3d center_dir;
    nh.param("FanDisturbance/center_pos_x", source_p(0), 0.0);
    nh.param("FanDisturbance/center_pos_x", source_p(1), 0.0);
    nh.param("FanDisturbance/center_pos_x", source_p(2), 0.0);

    nh.param("FanDisturbance/center_dir_x", center_dir(0), 1.0);
    nh.param("FanDisturbance/center_dir_x", center_dir(1), 0.0);
    nh.param("FanDisturbance/center_dir_x", center_dir(2), 0.0);
    if(center_dir.norm() == 0){
        center_dir(0) = 1;
    }
    if(center_dir.topRows(2).norm() == 0){
        source_yaw = 0;
    }
    else{
        source_yaw = atan2(center_dir(1), center_dir(0));
    }
    if(center_dir(2) == 0){
        source_pitch = 0;
    }
    else{
        source_pitch = atan2(center_dir(2), center_dir.topRows(2).norm());
    }
    source_roll = 0;
    double long_axis_len;
    nh.param("FanDisturbance/wind_range", long_axis_len, 0.2);
    range_a = long_axis_len / 2;
    nh.param("FanDisturbance/fan_radius", range_r, 0.1);
    nh.param("FanDisturbance/max_disturb_ratio", max_ratio, 0.1);
    double bias_from_bottom;
    nh.param("FanDisturbance/center_bias", bias_from_bottom, 0.0);
    bias_a = range_a - bias_from_bottom;
    double vis_dense;
    nh.param("FanDisturbance/visualize_density", vis_dense, 0.8);
    vis_d = max(1 - vis_dense, 0.01);
    nh.param("/grid_map/world_frame_name", world_frame, string("world"));

    disturb_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("disturbancesFanDisturbance_vis", 5);
    get_disturb_ratio = nh.advertiseService("get_disturb_ratio", &EllipseDisturb::getDisturbRatioSrv, this);
    timer1 = nh.createTimer(ros::Rate(2.0), &EllipseDisturb::timer1Cb, this);
    cloud_gen = false;
}

void EllipseDisturb::genVisCloud(){
    if(cloud_gen){
        return;
    }

    tf2::Matrix3x3 rot_mat;
    rot_mat.setEulerZYX(source_yaw, source_pitch, source_roll);
    Eigen::MatrixXd trans_mat(4, 4);
    trans_mat.setZero();
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            trans_mat(i,j) = rot_mat[i][j];
        }
    }
    trans_mat.block<3,1>(0,3) = source_p;
    trans_mat(3,3) = 1;
    ellipse_center = (trans_mat * Eigen::Vector4d(bias_a, 0, 0, 1)).block<3,1>(0,0);

    Eigen::Vector3d surf_pt;
    vector<Eigen::Vector3d> surf_pts;
    for(int i=0; i<=15; i++){
        for(int j=0; j<30; j++){
            double theta = 1/15 * M_PI;
            double phi = j/15 * M_PI;
            surf_pt(0) = range_a * sin(theta) * cos(phi) + bias_a;
            surf_pt(1) = range_r * sin(theta) * sin(phi);
            surf_pt(2) = range_r * cos(theta);
            surf_pts.push_back(surf_pt);
        }
    }

    Eigen::Vector3d _pt;
    vector<Eigen::Vector3d> _orig_vis_cloud;
    for(auto p : surf_pts){
        double max_len = p.norm();
        double tmp_len = vis_d;
        int count = 0;
        while(tmp_len < max_len){
            double _r = tmp_len / max_len;
            _pt = p * _r;
            _orig_vis_cloud.push_back(_pt);
            tmp_len += count * count * vis_d;
            count++;
        }
    }
    pcl::PointXYZ _vis_pt;
    pcl::PointCloud<pcl::PointXYZ> _vis_cloud_raw, _vis_cloud;
    for(auto p : _orig_vis_cloud){
        Eigen::Vector4d orig_pt(p(0), p(1), p(2), 1);
        Eigen::Vector4d new_pt = trans_mat * orig_pt;
        _vis_pt.x = new_pt(0);
        _vis_pt.y = new_pt(1);
        _vis_pt.z = new_pt(2);
        _vis_cloud_raw.push_back(_vis_pt);
    }

    pcl::VoxelGrid<pcl::PointXYZ> ft;
    ft.setLeafSize(vis_d, vis_d, vis_d);
    ft.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(_vis_cloud_raw));
    ft.filter(_vis_cloud);

    _vis_cloud.header.frame_id = world_frame;
    _vis_cloud.is_dense = true;
    _vis_cloud.width = _vis_cloud.points.size();
    _vis_cloud.height = 1;
    pcl::toROSMsg(_vis_cloud, disturb_vis_msg);
    cloud_gen = true;
}

bool EllipseDisturb::getDisturbRatioSrv(disturbance_sources::DisturbRatio::Request &req, disturbance_sources::DisturbRatio::Response &res){
    Eigen::Vector3d cur_pos;
    cur_pos(0) = req.pos_x;
    cur_pos(1) = req.pos_y;
    cur_pos(2) = req.pos_z;
    double dist = (cur_pos - source_p).norm();

    if(source_p == ellipse_center){
        if(dist < range_a){
            res.ratio = max_ratio * pow(1 - dist/range_a, 2);
        }
        else{
            res.ratio = 0.0;
        }
        return true;
    }

    if(cur_pos == source_p){
        res.ratio = max_ratio;
        return true;
    }

    Eigen::Vector3d vec_CP = cur_pos - source_p;
    Eigen::Vector3d vec_CO = ellipse_center - source_p;
    double alpha = acos(vec_CP.dot(vec_CO) / (vec_CP.norm()*vec_CO.norm()));
    double sin_alpha = sin(alpha);
    double cos_alpha = cos(alpha);
    double A = pow(range_a * sin_alpha, 2) + pow(range_r * cos_alpha, 2);
    double B = 2 * range_a * bias_a * pow(sin_alpha, 2);
    double C = pow(bias_a * sin_alpha, 2) - pow(range_r * cos_alpha, 2);
    double cos_theta = (sqrt(B*B - 4*A*C) - B) / 2 / A;
    double sin_theta = sqrt(1 - cos_theta*cos_theta);
    double ellipse_range = sqrt(pow(range_a*cos_theta + bias_a, 2) + pow(range_r*sin_theta, 2));
    if(dist < ellipse_range){
        res.ratio = max_ratio * pow(1 - dist/ellipse_range, 2);
    }
    else{
        res.ratio = 0.0;
    }
    return true;
}

void EllipseDisturb::timer1Cb(const ros::TimerEvent&){
    if(!cloud_gen){
        this->genVisCloud();
        return;
    }
    disturb_vis_pub.publish(disturb_vis_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ellipse_disturb_node");
    ros::NodeHandle node;
    EllipseDisturb disturb(&node);

    ros::spin();
    return 0;
}