#include "reference_governor/governor.h"

using namespace disturbance_aware_planner;

ReferenceGovernor::ReferenceGovernor(ros::NodeHandle* node): nh(*node){
    nh.param("Commander/cmd_frequency", cmd_freq, 10.0);
    nh.param("Commander/traj_plot_frequency", traj_plot_freq, 5.0);
    nh.param("Task/start_pos_x", init_ref_pos(0), 0.0);
    nh.param("Task/start_pos_y", init_ref_pos(1), 0.0);
    nh.param("Task/hover_height", init_ref_pos(2), 1.0);
    nh.param("Task/goal_pos_x", goal_pos(0), 1.0);
    nh.param("Task/goal_pos_y", goal_pos(1), 1.0);
    nh.param("Task/goal_pos_z", goal_pos(2), 1.0);
    nh.param("Task/land_after_complete", land_after_complete, false);
    nh.param("grid_map/world_frame_name", world_frame_id, std::string("world"));
    nh.param("grid_map/resolution", goal_pos_delta, 0.05);
    
    traj_sub = nh.subscribe<reference_governor::polyTraj>("planner/ref_polytraj", 1, &ReferenceGovernor::trajSubCb, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("crazyflie/pose_and_att", 10, &ReferenceGovernor::subPosCb, this);
    local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("crazyflie/vel_and_angrate", 10, &ReferenceGovernor::subVelCb, this);
    flight_mode_sub = nh.subscribe<std_msgs::Int16>("crazyflie/ctrl_mode", 2, &ReferenceGovernor::subModeCb, this);

    target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("crazyflie/pose_and_att_cmd", 10);
    target_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("crazyflie/vel_and_angrate_cmd", 10);
    target_acc_pub = nh.advertise<geometry_msgs::AccelStamped>("crazyflie/acc_cmd", 10);
    target_mode_pub = nh.advertise<std_msgs::Int16>("crazyflie/mode_cmd", 1);
    local_ref_traj_pub = nh.advertise<sensor_msgs::PointCloud2>("planner/ref_traj_plot", 2);
    timer1 = nh.createTimer(ros::Rate(cmd_freq), &ReferenceGovernor::timer1Cb, this);
    timer2 = nh.createTimer(ros::Rate(traj_plot_freq), &ReferenceGovernor::timer2Cb, this);

    current_ctrl_mode = 0;
    traj_set = false;
    last_cmd = Eigen::VectorXd::Zero(13);
    last_cmd.head(3) = init_ref_pos;
    goal_reached = false;
    cmd_takeoff = false;
    cmd_offb = false;
    mode_change_count = 0;
    at_goal_pos_count = 0;
    
    nh.param("Log/enable_log", enable_log, false);
    if(enable_log){
        string log_folder_name;
	    nh.param("Log/log_folder_name", log_folder_name, string("default_folder"));
        logger_ptr = std::shared_ptr<FlightLogger>(new FlightLogger(log_folder_name, "reference_governor"));
        vector<string> tags = {
            "goal_reached", "cmd_offboard", "traj_time_horizon", "traj_plan_time",
        };
        int _order;
        nh.param("Optimization/poly_order", _order, 4);
        for(int i=0; i<=_order; i++){
            tags.push_back("coef_x_"+to_string(i));
            tags.push_back("coef_y_"+to_string(i));
            tags.push_back("coef_z_"+to_string(i));
        }
		logger_ptr->setDataTags(tags);
        dumpParams();
    }
}

ReferenceGovernor::~ReferenceGovernor(){
    if(enable_log){
        logger_ptr->saveFile();
    }
}

void ReferenceGovernor::setNewTraj(Eigen::Matrix<double, 3, -1> coefs, double t_horizon, ros::Time start_time){
    tmp_traj = Polynomial3D(coefs);
    tmp_traj_d1 = tmp_traj.derivative();
    tmp_traj_d2 = tmp_traj_d1.derivative();
    traj_time_len = std::max(t_horizon, 0.0);
    traj_start_time = start_time;
    traj_set = true;
    ROS_INFO("New reference trajectory of order %ld received.", coefs.cols()-1);
    
    if(enable_log){
        double _log_t = (ros::Time::now() - start_log_ts).toSec();
        vector<string> log_tags;
        vector<double> log_data;
        for(int i=0; i<coefs.cols(); i++){
            log_tags.push_back("coef_x_"+to_string(i));
            log_tags.push_back("coef_y_"+to_string(i));
            log_tags.push_back("coef_z_"+to_string(i));
            log_data.push_back(coefs(0,i));
            log_data.push_back(coefs(1,i));
            log_data.push_back(coefs(2,i));
        }
        logger_ptr->logData(_log_t, log_tags, log_data);
        logger_ptr->logData(_log_t, "traj_time_horizon", t_horizon);
        logger_ptr->logData(_log_t, "traj_plan_time", (ros::Time::now() - start_time).toSec());
    }
}

ReferenceGovernor::RefState ReferenceGovernor::getRefCmd_Full(){
    if(!traj_set){
        return last_cmd;
    }
    double t_now = (ros::Time::now() - traj_start_time).toSec();
    RefState result = RefState::Zero();
    if(t_now > traj_time_len){
        result.segment<3>(0) = tmp_traj.evaluate(traj_time_len);
        result(9) = last_cmd(9);
    }
    else{
        result.segment<3>(0) = tmp_traj.evaluate(t_now);
        result.segment<3>(3) = tmp_traj_d1.evaluate(t_now);
        result.segment<3>(6) = tmp_traj_d2.evaluate(t_now);
        if(result(3) == 0 && result(4) == 0){
            result(9) = last_cmd(9);
        }
        else{
            result(9) = std::atan2(-1.0 * result(4), result(3));
        }
    }
    last_cmd = result;
    return result;
}

void ReferenceGovernor::dumpParams(){
	if(!enable_log){
        return;
    }
	logger_ptr->logParameter("cmd_freq", cmd_freq);
    logger_ptr->logParameter("land_after_complete", land_after_complete);
	start_log_ts = ros::Time::now();
}

void ReferenceGovernor::trajSubCb(const reference_governor::polyTraj::ConstPtr& msg){
    if(msg->poly_coefs.layout.dim[0].size != 3){
        ROS_WARN("Wrong reference trajectory dimension, got %d.", msg->poly_coefs.layout.dim[0].size);
        return;
    }
    int offset = msg->poly_coefs.layout.data_offset;
    int poly_deg = msg->poly_coefs.layout.dim[1].size;
    if(msg->poly_coefs.layout.dim[1].size != msg->poly_coefs.layout.dim[1].stride){
        ROS_WARN("Wrong reference trajectory dimension, got %d.", msg->poly_coefs.layout.dim[0].size);
        return;
    }
    ros::Time start_time = msg->start_planning_time.data;
    Eigen::MatrixXd new_coefs(3, poly_deg);
    for(int i=0; i<3; i++){
        for(int j=0; j<poly_deg; j++){
            new_coefs(i,j) = msg->poly_coefs.data[offset + i*poly_deg + j];
        }
    }
    double traj_duration = msg->poly_duration.data;
    this->setNewTraj(new_coefs, traj_duration, start_time);
}

void ReferenceGovernor::subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    current_att = Eigen::Vector4d(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void ReferenceGovernor::subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    current_angrate = Eigen::Vector3d(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
}

void ReferenceGovernor::subModeCb(const std_msgs::Int16::ConstPtr& msg){
    current_ctrl_mode = msg->data;
}

void ReferenceGovernor::timer1Cb(const ros::TimerEvent&){
    if(target_mode_pub.getNumSubscribers() <= 0 || flight_mode_sub.getNumPublishers() <= 0){
        ROS_INFO_ONCE("Flight commander waiting for quadrotor driver connection...");
        return;
    }
    else{
        ROS_INFO_ONCE("Flight commander found quadrotor driver connected!");
    }

    if(enable_log){
        double _log_t = (ros::Time::now() - start_log_ts).toSec();
        logger_ptr->logData(_log_t, "goal_reached", goal_reached);
        logger_ptr->logData(_log_t, "cmd_offboard", cmd_offb);
    }
    
    if(goal_reached){
        if(land_after_complete){
            if(current_ctrl_mode != 0){
                std_msgs::Int16 mode_cmd;
                mode_cmd.data = 0;
                target_mode_pub.publish(mode_cmd);
            }
        }
        else{
            geometry_msgs::PoseStamped pos_cmd;
            geometry_msgs::TwistStamped vel_cmd;
            geometry_msgs::AccelStamped acc_cmd;
            pos_cmd.pose.position.x = goal_pos(0);
            pos_cmd.pose.position.y = goal_pos(1);
            pos_cmd.pose.position.z = goal_pos(2);
            tf2::Quaternion quat;
            quat.setRPY(0, 0, last_cmd(9));
            pos_cmd.pose.orientation.w = quat.w();
            pos_cmd.pose.orientation.x = quat.x();
            pos_cmd.pose.orientation.y = quat.y();
            pos_cmd.pose.orientation.z = quat.z();
            vel_cmd.twist.linear.x = 0;
            vel_cmd.twist.linear.y = 0;
            vel_cmd.twist.linear.z = 0;
            acc_cmd.accel.linear.x = 0;
            acc_cmd.accel.linear.y = 0;
            acc_cmd.accel.linear.z = 0;
            vel_cmd.twist.angular.x = 0;
            vel_cmd.twist.angular.y = 0;
            vel_cmd.twist.angular.z = 0;
            target_pos_pub.publish(pos_cmd);
            target_vel_pub.publish(vel_cmd);
            target_acc_pub.publish(acc_cmd);
        }
        return;
    }
    
    if(current_ctrl_mode == 0){
        if(!cmd_takeoff){
            std_msgs::Int16 mode_cmd;
            mode_cmd.data = 1;
            target_mode_pub.publish(mode_cmd);
            cmd_takeoff = true;
        }
        else{
            mode_change_count++;
            if(mode_change_count >= 50){
                cmd_takeoff = false;
                mode_change_count = 0;
            }
        }
    }
    else if(current_ctrl_mode == 1){
        if(!cmd_offb){
            std_msgs::Int16 mode_cmd;
            mode_cmd.data = 2;
            target_mode_pub.publish(mode_cmd);
            cmd_offb = true;
        }
        else{
            mode_change_count++;
            if(mode_change_count >= 10){
                cmd_offb = false;
                mode_change_count = 0;
            }
        }
    }
    else{
        geometry_msgs::PoseStamped pos_cmd;
        geometry_msgs::TwistStamped vel_cmd;
        geometry_msgs::AccelStamped acc_cmd;

        Eigen::VectorXd ref = this->getRefCmd_Full();   // [p, v, a, yaw, omega]
        pos_cmd.pose.position.x = ref(0);
        pos_cmd.pose.position.y = ref(1);
        pos_cmd.pose.position.z = ref(2);
        vel_cmd.twist.linear.x = ref(3);
        vel_cmd.twist.linear.y = ref(4);
        vel_cmd.twist.linear.z = ref(5);
        acc_cmd.accel.linear.x = ref(6);
        acc_cmd.accel.linear.y = ref(7);
        acc_cmd.accel.linear.z = ref(8);
        tf2::Quaternion quat;
        quat.setRPY(0, 0, ref(9));
        pos_cmd.pose.orientation.w = quat.w();
        pos_cmd.pose.orientation.x = quat.x();
        pos_cmd.pose.orientation.y = quat.y();
        pos_cmd.pose.orientation.z = quat.z();

        vel_cmd.twist.angular.x = 0;
        vel_cmd.twist.angular.y = 0;
        vel_cmd.twist.angular.z = 0;
        target_pos_pub.publish(pos_cmd);
        target_vel_pub.publish(vel_cmd);
        target_acc_pub.publish(acc_cmd);

        if((current_pos - goal_pos).norm() <= 1.2*goal_pos_delta){
            if(at_goal_pos_count >= 50){
                goal_reached = true;
                at_goal_pos_count = 100;
            }
            at_goal_pos_count++;
        }
        else{
            at_goal_pos_count = 0;
            goal_reached = false;
        }
    }
}

void ReferenceGovernor::timer2Cb(const ros::TimerEvent&){
    std::vector<Eigen::Vector3d> plot_pts = tmp_traj.genTrajPlotPoints(traj_time_len, 3 / cmd_freq);
    pcl::PointCloud<pcl::PointXYZ> ref_traj_cloud;
    pcl::PointXYZ pt;
    for(auto p : plot_pts){
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        ref_traj_cloud.push_back(pt);
    }

    ref_traj_cloud.width = ref_traj_cloud.points.size();
    ref_traj_cloud.height = 1;
    ref_traj_cloud.is_dense = true;
    ref_traj_cloud.header.frame_id = world_frame_id;
    sensor_msgs::PointCloud2 ref_traj_msg;
    pcl::toROSMsg(ref_traj_cloud, ref_traj_msg);
    local_ref_traj_pub.publish(ref_traj_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "reference_governor_node");
    ros::NodeHandle nh;
    ReferenceGovernor governor(&nh);

    ros::spin();

    return 0;
}