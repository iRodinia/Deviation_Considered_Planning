#include "disturbance_aware_planner/planner_node.h"

using namespace disturbance_aware_planner;

DisturbanceAwarePlanner::DisturbanceAwarePlanner(ros::NodeHandle* nh): nh_(*nh){
    local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("crazyflie/pose_and_att", 1, &DisturbanceAwarePlanner::subPosCb, this);
    local_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("crazyflie/vel_and_angrate", 1, &DisturbanceAwarePlanner::subVelCb, this);
    flight_mode_sub = nh_.subscribe<std_msgs::Int16>("crazyflie/ctrl_mode", 1, &DisturbanceAwarePlanner::subModeCb, this);
    ref_traj_pub = nh_.advertise<reference_governor::polyTraj>("planner/ref_polytraj", 1);

    double replan_freq;   // frequency for conducting replanning
    nh_.param("Task/start_pos_x", start_pos(0), 0.0);
    nh_.param("Task/start_pos_y", start_pos(1), 0.0);
    nh_.param("Task/hover_height", start_pos(2), 1.0);
    nh_.param("Task/goal_pos_x", goal_pos(0), 1.0);
    nh_.param("Task/goal_pos_y", goal_pos(1), 1.0);
    nh_.param("Task/goal_pos_z", goal_pos(2), 1.0);
    nh_.param("Commander/replan_frequency", replan_freq, 1.0);
    int pred_N;
    double pred_dt;
    nh_.param("Optimization/predict_num", pred_N, 100);
    nh_.param("Optimization/predict_dt", pred_dt, 0.02);
    replan_t_hori = pred_N * pred_dt;
    timer1 = nh_.createTimer(ros::Rate(replan_freq), &DisturbanceAwarePlanner::timer1Cb, this);

    global_ptr.reset(new GlobalMapProcessor(nh_));
    opter_ptr.reset(new PolyTrajOptimizer);
    opter_ptr->initParameters(nh_);

    enable_log = false;
}

void DisturbanceAwarePlanner::setLogger(FlightLogger* _ptr){
	logger_ptr = _ptr;
	enable_log = true;
	logger_ptr->logParameter("start_pos_x", start_pos(0));
    logger_ptr->logParameter("start_pos_y", start_pos(1));
    logger_ptr->logParameter("start_pos_z", start_pos(2));
    logger_ptr->logParameter("goal_pos_x", goal_pos(0));
    logger_ptr->logParameter("goal_pos_y", goal_pos(1));
    logger_ptr->logParameter("goal_pos_z", goal_pos(2));
	start_log_ts = ros::Time::now();
}

void DisturbanceAwarePlanner::subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    current_att = Eigen::Vector4d(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void DisturbanceAwarePlanner::subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    current_angrate = Eigen::Vector3d(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
}

void DisturbanceAwarePlanner::subModeCb(const std_msgs::Int16::ConstPtr& msg){
    current_ctrl_mode = msg->data;
}

void DisturbanceAwarePlanner::timer1Cb(const ros::TimerEvent&){     // call at each replanning iteration
    if(!global_ptr->isSFCGenerated() || !global_ptr->isPathGenerated()){
        return;
    }
    if(current_ctrl_mode != 2){
        ROS_INFO("Replanning process waiting for offboard control mode.");
        return;
    }
    ros::Time t_start = ros::Time::now();

    vector<double> tmp_times;
    vector<Eigen::MatrixX4d> tmp_sfcs;
    Eigen::Vector3d tmp_target_p, tmp_target_v;
    ROS_INFO("=========== Replanning info ===========");
    global_ptr->getReplanInfo(current_pos, opter_ptr->getPredTimeHorizon(), tmp_times, tmp_sfcs, tmp_target_p, tmp_target_v);
    for(int i=0; i<tmp_times.size(); i++){
        ROS_INFO("Time alloc[%d]: %f ; sfc[%d] size: %ld", i, tmp_times[i], i, tmp_sfcs[i].rows());
    }
    ROS_INFO("Local target position: (%f, %f, %f)", tmp_target_p(0), tmp_target_p(1), tmp_target_p(2));
    ROS_INFO("Local target velocity: (%f, %f, %f)", tmp_target_v(0), tmp_target_v(1), tmp_target_v(2));
    opter_ptr->setStates(current_pos, current_vel, current_acc, tmp_target_p, tmp_target_v);
    opter_ptr->setCollisionConstraints(tmp_sfcs, tmp_times);

    if(enable_log){
        double _log_t = (ros::Time::now() - start_log_ts).toSec();
        logger_ptr->logData(_log_t, "ref_path_px", tmp_target_p(0));
        logger_ptr->logData(_log_t, "ref_path_py", tmp_target_p(1));
        logger_ptr->logData(_log_t, "ref_path_pz", tmp_target_p(2));
        logger_ptr->logData(_log_t, "ref_path_vx", tmp_target_v(0));
        logger_ptr->logData(_log_t, "ref_path_vy", tmp_target_v(1));
        logger_ptr->logData(_log_t, "ref_path_vz", tmp_target_v(2));
    }

    if(opter_ptr->optimize()){
        ros::Time t_finish = ros::Time::now();
        replan_dur = t_finish - t_start;
        ROS_INFO("Replan iteration takes %f seconds.", replan_dur.toSec());
        ROS_INFO("smooth cost: %f; frs cost: %f; terminal cost: %f.", 
                    opter_ptr->smooth_cost_save, opter_ptr->frs_cost_save, opter_ptr->terminal_cost_save);
        ROS_INFO("=======================================");
        auto poly_coefs = opter_ptr->getTrajectoryCoefficients();
        
        reference_governor::polyTraj traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.start_planning_time.data = t_start;
        traj_msg.poly_duration.data = replan_t_hori;
        traj_msg.poly_coefs.layout.data_offset = 0;
        traj_msg.poly_coefs.layout.dim.resize(2, std_msgs::MultiArrayDimension());
        traj_msg.poly_coefs.layout.dim[0].label = "dimension";
        traj_msg.poly_coefs.layout.dim[0].size = 3;
        traj_msg.poly_coefs.layout.dim[0].stride = poly_coefs.size();
        traj_msg.poly_coefs.layout.dim[1].label = "poly_coefs";
        traj_msg.poly_coefs.layout.dim[1].size = poly_coefs.cols();
        traj_msg.poly_coefs.layout.dim[1].stride = poly_coefs.cols();
        traj_msg.poly_coefs.data.resize(poly_coefs.size());
        for(int i=0; i<3; i++){
            for(int j=0; j<poly_coefs.cols(); j++){
                traj_msg.poly_coefs.data[i*poly_coefs.cols()+j] = poly_coefs(i,j);
            }
        }
        ref_traj_pub.publish(traj_msg);
    }
    else{
        ROS_INFO("Replanning failed.");
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh;
    DisturbanceAwarePlanner cmder(&nh);

    bool log_enable = false;
	nh.param("Log/enable_log", log_enable, false);
	string log_folder_name;
	nh.param("Log/log_folder_name", log_folder_name, string("default_folder"));
	FlightLogger logger(log_folder_name, "global_planner");
	if(log_enable){
		vector<string> tags = {
            "ref_path_px", "ref_path_py", "ref_path_pz", 
            "ref_path_vx", "ref_path_vy", "ref_path_vz", 
        };
		logger.setDataTags(tags);
		cmder.setLogger(&logger);
	}

    ros::spin();

    if(log_enable){
		logger.saveFile();
	}

    ros::spin();
    return 0;
}