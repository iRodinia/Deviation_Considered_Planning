#include "disturbance_sources/ellipsoidal_disturb.h"

EllipseDisturb::EllipseDisturb(ros::NodeHandle* node): nh(*node){
    double x_size, y_size, z_size, ground_height_;
    nh.param("grid_map/resolution", resolution_, 0.05);
    nh.param("grid_map/map_size_x", x_size, 5.0);
    nh.param("grid_map/map_size_y", y_size, 5.0);
    nh.param("grid_map/map_size_z", z_size, 2.5);
    nh.param("grid_map/ground_height", ground_height_, -0.05);
    resolution_inv_ = 1 / resolution_;
    map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, ground_height_);
    map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
    for (int i = 0; i < 3; ++i)
        map_voxel_num_(i) = ceil(map_size_(i) / resolution_);

    map_min_boundary_ = map_origin_;
    map_max_boundary_ = map_origin_ + map_size_;
    int buffer_size = map_voxel_num_(0) * map_voxel_num_(1) * map_voxel_num_(2);
    disturb_buffer_ = vector<float>(buffer_size, 0.0);

    nh.param("FanDisturbance/source_num", source_num, 0);
    source_num = std::max(source_num, 0);
    std::vector<double> source_px, source_py, source_pz;
    nh.getParam("FanDisturbance/center_pos_x", source_px);
    nh.getParam("FanDisturbance/center_pos_y", source_py);
    nh.getParam("FanDisturbance/center_pos_z", source_pz);
    std::vector<double> dir_px, dir_py, dir_pz;
    nh.getParam("FanDisturbance/center_dir_x", dir_px);
    nh.getParam("FanDisturbance/center_dir_y", dir_py);
    nh.getParam("FanDisturbance/center_dir_z", dir_pz);
    std::vector<double> bias_from_bottom;
    nh.getParam("FanDisturbance/center_bias", bias_from_bottom);
    nh.getParam("FanDisturbance/fan_radius", range_r);
    std::vector<double> long_axis_len;
    nh.getParam("FanDisturbance/wind_range", long_axis_len);

    source_p = std::vector<Eigen::Vector3d>(source_num, Eigen::Vector3d());
    source_dir = std::vector<Eigen::Vector3d>(source_num, Eigen::Vector3d());
    range_a = std::vector<double>(source_num, 0.0);
    bias_a = std::vector<double>(source_num, 0.0);
    ellipse_center = std::vector<Eigen::Vector3d>(source_num, Eigen::Vector3d());
    trans_mat = std::vector<Eigen::MatrixXd>(source_num, Eigen::MatrixXd());

    for(int i=0; i<source_num; i++){
        source_p[i] = Eigen::Vector3d(source_px[i], source_py[i], source_pz[i]);
        Eigen::Vector3d _dir = Eigen::Vector3d(dir_px[i], dir_py[i], dir_pz[i]);
        if(_dir.norm() == 0){
            source_dir[i] = Eigen::Vector3d(1, 0, 0);
        }
        else{
            source_dir[i] = _dir.normalized();
        }
        range_a[i] = long_axis_len[i] / 2.0;
        bias_a[i] = long_axis_len[i] / 2.0 - bias_from_bottom[i];
        Eigen::MatrixXd _mat(4, 4);
        _mat.setZero();
        _mat.block<3,3>(0,0) = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1,0,0), source_dir[i]).toRotationMatrix();
        _mat.block<3,1>(0,3) = source_p[i];
        _mat(3,3) = 1;
        trans_mat[i] = _mat;
        ellipse_center[i] = (_mat * Eigen::Vector4d(bias_a[i], 0, 0, 1)).block<3,1>(0,0);
    }
    nh.param("/grid_map/world_frame_name", world_frame, string("world"));
    nh.param("/grid_map/max_disturb_ratio", max_ratio, 0.5);

    disturb_map_pub = nh.advertise<std_msgs::Float32MultiArray>("disturbances/FanDisturbance_map", 1);
    disturb_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("disturbances/FanDisturbance_vis", 5);
    get_disturb_ratio = nh.advertiseService("get_disturb_ratio", &EllipseDisturb::getDisturbRatioSrv, this);
    timer1 = nh.createTimer(ros::Rate(1.0), &EllipseDisturb::timer1Cb, this);
    timer2 = nh.createTimer(ros::Rate(5.0), &EllipseDisturb::timer2Cb, this);
    map_gen = false;
    cloud_gen = false;
    discret_num = 8;
    pub_count = 0;
    disturb_vis_msg_queue = vector<sensor_msgs::PointCloud2>(discret_num, sensor_msgs::PointCloud2());

    void genDisturbMap();
    void genVisCloud();

    nh.param("Log/enable_log", log_enable, false);
    if(log_enable){
        string log_folder_name;
	    nh.param("Log/log_folder_name", log_folder_name, string("default_folder"));
        logger_ptr = std::shared_ptr<FlightLogger>(new FlightLogger(log_folder_name, "disturbance_generator"));
        dumpParams();
    }
}

EllipseDisturb::~EllipseDisturb(){
    if(log_enable){
        logger_ptr->saveFile();
    }
}

void EllipseDisturb::dumpParams(){
    if(!log_enable){
        return;
    }
    log_start_ts = ros::Time::now();
    log_enable = true;

    logger_ptr->logParameter("disturb_num", source_num);
    for(int i=0; i<source_num; i++){
        logger_ptr->logParameter("disturb_"+to_string(i)+"_pos_x", source_p[i](0));
        logger_ptr->logParameter("disturb_"+to_string(i)+"_pos_y", source_p[i](1));
        logger_ptr->logParameter("disturb_"+to_string(i)+"_pos_z", source_p[i](2));
        logger_ptr->logParameter("disturb_"+to_string(i)+"_dir_x", source_dir[i](0));
        logger_ptr->logParameter("disturb_"+to_string(i)+"_dir_y", source_dir[i](1));
        logger_ptr->logParameter("disturb_"+to_string(i)+"_dir_z", source_dir[i](2));
        logger_ptr->logParameter("disturb_"+to_string(i)+"_cylinder_h", 2 * range_a[i]);
        logger_ptr->logParameter("disturb_"+to_string(i)+"_cylinder_rad", range_r[i]);
        logger_ptr->logParameter("disturb_"+to_string(i)+"_cylinder_bias", range_a[i] - bias_a[i]);
        for(int j=0; j<3; j++){
            for(int k=0; k<3; k++){
                logger_ptr->logParameter("disturb_"+to_string(i)+"_trans_mat_"+to_string(j)+to_string(k), trans_mat[i](j,k));
            }
        }
    }
}

void EllipseDisturb::genDisturbMap(){
    if(map_gen){
        return;
    }
    Eigen::Vector3d _p;
    for(int i=0; i<map_voxel_num_(0); i++){
        for(int j=0; j<map_voxel_num_(1); j++){
            for(int k=0; k<map_voxel_num_(2); k++){
                indexToPos(Eigen::Vector3i(i,j,k), _p);
                disturb_buffer_[toAddress(i, j, k)] = getDisturbRatio(_p);
            }
        }
    }
    map_gen = true;
}

void EllipseDisturb::genVisCloud(){
    if(cloud_gen){
        return;
    }

    vector<vector<Eigen::Vector3d>> surf_pts_vec;
    for(int id=0; id<source_num; id++){
        vector<Eigen::Vector3d> surf_pts;
        Eigen::Vector3d surf_pt;
        for(int i=0; i<=15; i++){
            double theta = i / 15.0 * M_PI;
            for(int j=0; j<30; j++){
                double phi = j / 15.0 * M_PI;
                surf_pt(0) = range_a[id] * sin(theta) * cos(phi) + bias_a[id];
                surf_pt(1) = range_r[id] * sin(theta) * sin(phi);
                surf_pt(2) = range_r[id] * cos(theta);
                surf_pts.push_back(surf_pt);
            }
        }
        surf_pts_vec.push_back(surf_pts);
    }
  
    vector<vector<vector<Eigen::Vector3d>>> _orig_vis_clouds_vec;
    for(int id=0; id<source_num; id++){
        Eigen::Vector3d _pt;
        vector<vector<Eigen::Vector3d>> _orig_vis_clouds(discret_num, vector<Eigen::Vector3d>());
        for(auto p : surf_pts_vec[id]){
            double p_len = p.norm();
            for(int i=0; i<discret_num; i++){
                for(int j=0; j<3; j++){
                    _pt = (j/3.0 + i/3.0/discret_num) * p;
                    _orig_vis_clouds[i].push_back(_pt);
                }
            }
        }
        _orig_vis_clouds_vec.push_back(_orig_vis_clouds);
    }

    pcl::PointXYZ _vis_pt;
    vector<pcl::PointCloud<pcl::PointXYZ>> _vis_cloud_raw_queue(discret_num, pcl::PointCloud<pcl::PointXYZ>());
    vector<pcl::PointCloud<pcl::PointXYZ>> _vis_cloud_queue(discret_num, pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> ft;
    ft.setLeafSize(0.01, 0.01, 0.01);
    for(int i=0; i<discret_num; i++){
        for(int id=0; id<source_num; id++){
            for(auto p : _orig_vis_clouds_vec[id][i]){
                Eigen::Vector4d orig_pt(p(0), p(1), p(2), 1);
                Eigen::Vector4d new_pt = trans_mat[id] * orig_pt;
                _vis_pt.x = new_pt(0);
                _vis_pt.y = new_pt(1);
                _vis_pt.z = new_pt(2);
                _vis_cloud_raw_queue[i].push_back(_vis_pt);
            }
        }
        ft.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(_vis_cloud_raw_queue[i]));
        ft.filter(_vis_cloud_queue[i]);
        _vis_cloud_queue[i].header.frame_id = world_frame;
        _vis_cloud_queue[i].is_dense = true;
        _vis_cloud_queue[i].width = _vis_cloud_queue[i].points.size();
        _vis_cloud_queue[i].height = 1;
        pcl::toROSMsg(_vis_cloud_queue[i], disturb_vis_msg_queue[i]);
    }
    
    cloud_gen = true;
}

double EllipseDisturb::getDisturbRatio_id(Eigen::Vector3d _pos, int fan_id){
    if(fan_id < 0 || fan_id + 1 > source_num){
        return 0;
    }
    double dist = (_pos - source_p[fan_id]).norm();
    if(dist == 0){
        return max_ratio;
    }

    double ratio = 0;
    Eigen::Vector3d vec_CP = _pos - source_p[fan_id];
    Eigen::Vector3d vec_CO = ellipse_center[fan_id] - source_p[fan_id];
    double alpha = acos(vec_CP.dot(vec_CO) / (vec_CP.norm()*vec_CO.norm()));
    double sin_alpha = sin(alpha);
    double cos_alpha = cos(alpha);
    double A = pow(range_a[fan_id] * sin_alpha, 2) + pow(range_r[fan_id] * cos_alpha, 2);
    double B = 2 * range_a[fan_id] * bias_a[fan_id] * pow(sin_alpha, 2);
    double C = pow(bias_a[fan_id] * sin_alpha, 2) - pow(range_r[fan_id] * cos_alpha, 2);
    double cos_theta = (sqrt(B*B - 4*A*C) - B) / 2 / A;
    double sin_theta = sqrt(1 - cos_theta*cos_theta);
    double ellipse_range = sqrt(pow(range_a[fan_id]*cos_theta + bias_a[fan_id], 2) + pow(range_r[fan_id]*sin_theta, 2));
    if(dist < ellipse_range){
        // ratio = max_ratio * pow(1 - dist/ellipse_range, 2);
        ratio = max_ratio * (1 - dist/ellipse_range);
    }
    return ratio;
}

double EllipseDisturb::getDisturbRatio(Eigen::Vector3d _pos){
    double ratio = 0;
    for(int i=0; i<source_num; i++){
        ratio += getDisturbRatio_id(_pos, i);
    }
    ratio = std::min(ratio, max_ratio);
    return ratio;
}

bool EllipseDisturb::getDisturbRatioSrv(disturbance_sources::DisturbRatio::Request &req, disturbance_sources::DisturbRatio::Response &res){
    Eigen::Vector3d cur_pos;
    cur_pos(0) = req.pos_x;
    cur_pos(1) = req.pos_y;
    cur_pos(2) = req.pos_z;
    res.ratio = getDisturbRatio(cur_pos);
    return true;
}

void EllipseDisturb::timer1Cb(const ros::TimerEvent&){
    if(!map_gen){
        this->genDisturbMap();
        return;
    }
    std_msgs::Float32MultiArray disturb_data;
    disturb_data.data = disturb_buffer_;
    disturb_map_pub.publish(disturb_data);
}

void EllipseDisturb::timer2Cb(const ros::TimerEvent&){
    if(!cloud_gen){
        this->genVisCloud();
        return;
    }
    disturb_vis_pub.publish(disturb_vis_msg_queue[pub_count]);
    pub_count = (pub_count+1) % discret_num;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ellipse_disturb_node");
    ros::NodeHandle node;
    EllipseDisturb disturb(&node);
    
	ros::spin();

    return 0;
}