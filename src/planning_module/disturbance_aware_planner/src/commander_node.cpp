#include "disturbance_aware_planner/commander_node.h"

using namespace disturbance_aware_planner;

FlightCommander::FlightCommander(ros::NodeHandle* nh): nh_(*nh){
    local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("crazyflie/pose_and_att", 10, &FlightCommander::subPosCb, this);
    local_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("crazyflie/vel_and_angrate", 10, &FlightCommander::subVelCb, this);
    flight_mode_sub = nh_.subscribe<std_msgs::Int16>("crazyflie/ctrl_mode", 1, &FlightCommander::subModeCb, this);
    local_pcl_sub = nh_.subscribe<sensor_msgs::PointCloud2>("crazyflie/local_point_cloud", 5, &FlightCommander::subPclCb, this);

    target_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("crazyflie/pose_and_att_cmd", 10);
    target_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("crazyflie/vel_and_angrate_cmd", 10);
    target_acc_pub = nh_.advertise<geometry_msgs::AccelStamped>("crazyflie/acc_cmd", 10);
    target_mode_pub = nh_.advertise<std_msgs::Int16>("crazyflie/mode_cmd", 1);

    double cmd_freq;   // frequency for sending commands
    double replan_freq;   // frequency for conducting replanning
    nh_.param("Task/start_pos_x", start_pos(0), 0.0);
    nh_.param("Task/start_pos_y", start_pos(1), 0.0);
    nh_.param("Task/hover_height", start_pos(2), 1.0);
    nh_.param("Task/goal_pos_x", goal_pos(0), 1.0);
    nh_.param("Task/goal_pos_y", goal_pos(1), 1.0);
    nh_.param("Task/goal_pos_z", goal_pos(2), 1.0);
    nh_.param("Task/land_after_complete", land_after_complete, false);
    nh_.param("Commander/cmd_frequency", cmd_freq, 10.0);
    nh_.param("Commander/replan_frequency", replan_freq, 1.0);
    int pred_N;
    double pred_dt;
    nh_.param("Optimization/predict_num", pred_N, 100);
    nh_.param("Optimization/predict_dt", pred_dt, 0.02);
    replan_t_hori = pred_N * pred_dt;
    timer1 = nh_.createTimer(ros::Rate(cmd_freq), &FlightCommander::timer1Cb, this);
    timer2 = nh_.createTimer(ros::Rate(replan_freq), &FlightCommander::timer2Cb, this);

    global_ptr.reset(new GlobalMapProcessor(nh_));

    opter_ptr.reset(new PolyTrajOptimizer);
    opter_ptr->initParameters(nh_);
}

void FlightCommander::subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    current_att = Eigen::Vector4d(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void FlightCommander::subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    current_angrate = Eigen::Vector3d(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
}

void FlightCommander::subPclCb(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);
    int cloud_size = cloud_in.points.size();
    if(cloud_size <= 0){
        local_pcl.resize(3,0);
        return;
    }
    local_pcl.resize(3, cloud_size);
    for(size_t i=0; i<cloud_size; i++){
        pcl::PointXYZ pt = cloud_in.points[i];
        local_pcl(0,i) = pt.x;
        local_pcl(1,i) = pt.y;
        local_pcl(2,i) = pt.z;
    }
}

void FlightCommander::subModeCb(const std_msgs::Int16::ConstPtr& msg){
    current_ctrl_mode = msg->data;
}

void FlightCommander::timer1Cb(const ros::TimerEvent&){     // call at each command iteration
    if(target_mode_pub.getNumSubscribers() <= 0 || flight_mode_sub.getNumPublishers() <= 0){
        ROS_INFO_ONCE("Flight commander waiting for quadrotor driver connection...");
        return;
    }
    else{
        ROS_INFO_ONCE("Flight commander found quadrotor driver connected!");
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
        if(!goal_reached){
            if(!traj_buffer.initialized){
                pos_cmd.pose.position.x = start_pos(0);
                pos_cmd.pose.position.y = start_pos(1);
                pos_cmd.pose.position.z = start_pos(2);
                target_pos_pub.publish(pos_cmd);
            }
            else{
                Eigen::VectorXd ref = traj_buffer.getRefCmd_Full();   // [p, v, a, yaw, omega]
                pos_cmd.pose.position.x = ref(0);
                pos_cmd.pose.position.y = ref(1);
                pos_cmd.pose.position.z = ref(2);
                vel_cmd.twist.linear.x = ref(3);
                vel_cmd.twist.linear.y = ref(4);
                vel_cmd.twist.linear.z = ref(5);
                acc_cmd.accel.linear.x = ref(6);
                acc_cmd.accel.linear.y = ref(7);
                acc_cmd.accel.linear.z = ref(8);
                tf::Quaternion quat;
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
            }
        }
        else{
            pos_cmd.pose.position.x = goal_pos(0);
            pos_cmd.pose.position.y = goal_pos(1);
            pos_cmd.pose.position.z = goal_pos(2);
            vel_cmd.twist.linear.x = 0;
            vel_cmd.twist.linear.y = 0;
            vel_cmd.twist.linear.z = 0;
            acc_cmd.accel.linear.x = 0;
            acc_cmd.accel.linear.y = 0;
            acc_cmd.accel.linear.z = 0;
            target_pos_pub.publish(pos_cmd);
            target_vel_pub.publish(vel_cmd);
            target_acc_pub.publish(acc_cmd);
        }
    }

    if((current_pos - goal_pos).norm() <= 0.05){
        if(at_pos_count >= 50){
            goal_reached = true;
            at_pos_count = 100;
        }
        at_pos_count++;
    }
    else{
        at_pos_count = 0;
        goal_reached = false;
    }
}

void FlightCommander::timer2Cb(const ros::TimerEvent&){     // call at each replanning iteration
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
    Eigen::Vector3d tmp_target_p, tmp_target_vdir;
    global_ptr->getReplanInfo(current_pos, tmp_times, tmp_sfcs, tmp_target_p, tmp_target_vdir);
    opter_ptr->setStates(current_pos, current_vel, current_acc, tmp_target_p, tmp_target_vdir);
    opter_ptr->setCollisionConstraints(tmp_sfcs, tmp_times);
    if(opter_ptr->optimize()){
        ros::Time t_finish = ros::Time::now();
        replan_dur = t_finish - t_start;
        ROS_INFO("Replan iteration takes %f seconds.", replan_dur.toSec());
        Eigen::Matrix<double, 3, -1> poly_coefs = opter_ptr->getTrajectoryCoefficients();
        traj_buffer.setNewTraj(poly_coefs, replan_t_hori, replan_dur.toSec());
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
    FlightCommander cmder(&nh);

    ros::spin();
    return 0;
}