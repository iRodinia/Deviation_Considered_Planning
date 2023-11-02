#include "disturbance_aware_planner/commander_node.h"

using namespace disturbance_aware_planner;

FlightCommander::FlightCommander(ros::NodeHandle* nh): nh_(*nh){
    local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("crazyflie/pose_and_att", 10, &FlightCommander::subPosCb, this);
    local_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("crazyflie/vel_and_angrate", 10, &FlightCommander::subVelCb, this);
    flight_mode_sub = nh_.subscribe<std_msgs::Int16>("crazyflie/ctrl_mode", 1, &FlightCommander::subModeCb, this);

    target_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("crazyflie/pose_and_att_cmd", 10);
    target_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("crazyflie/vel_and_angrate_cmd", 10);
    target_acc_pub = nh_.advertise<geometry_msgs::AccelStamped>("crazyflie/acc_cmd", 10);
    target_mode_pub = nh_.advertise<std_msgs::Int16>("crazyflie/mode_cmd", 1);

    double cmd_freq;   // frequency for sending commands
    double replan_freq;   // frequency for conducting replanning
    nh_.param("Commander/cmd_frequency", cmd_freq, 10.0);
    nh_.param("Commander/replan_frequency", replan_freq, 1.0);
    timer1 = nh_.createTimer(ros::Rate(cmd_freq), &FlightCommander::timer1Cb, this);
    timer2 = nh_.createTimer(ros::Rate(replan_freq), &FlightCommander::timer2Cb, this);

}

void FlightCommander::subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    current_att = Eigen::Vector4d(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void FlightCommander::subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    current_angrate = Eigen::Vector3d(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
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
    geometry_msgs::PoseStamped pos_cmd;
    geometry_msgs::TwistStamped vel_cmd;
    geometry_msgs::AccelStamped acc_cmd;
    std_msgs::Int16 mode_cmd;
    if(current_ctrl_mode == 0){

    }
}

void FlightCommander::timer2Cb(const ros::TimerEvent&){     // call at each replanning iteration

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh;
    FlightCommander cmder(&nh);

    ros::spin();
    return 0;
}