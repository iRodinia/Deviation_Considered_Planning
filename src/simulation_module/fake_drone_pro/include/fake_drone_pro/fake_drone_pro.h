#ifndef FAKE_DRONE_PRO
#define FAKE_DRONE_PRO

#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include "flight_logger/logger.h"

class FakeDronePro{
public:
    FakeDronePro(ros::NodeHandle* node);
    ~FakeDronePro() {};
    void setLogger(FlightLogger* logger_ptr);

    int current_uav_mode = 0;
    Eigen::Vector3d ref_pos, current_pos;
    Eigen::Vector3d current_vel;
    Eigen::Vector3d current_acc;
    Eigen::Quaterniond ref_att, current_att;
    Eigen::Vector3d current_angrate;

private:
    ros::NodeHandle nh_;
    double sim_freq, sim_dt, T_c, damp_factor;
    bool enable_log;
    ros::Time start_log_ts;
    
    ros::Publisher uav_pos_pub;   // include pos and att
    ros::Publisher uav_vel_pub;   // include vel and angrate
    ros::Publisher flight_mode_pub;   // 0: land, 1: takeoff, 2: offb_ctrl
    ros::Subscriber target_pos_sub;   // include pos and att
    ros::Subscriber target_vel_sub;   // include vel and angrate
    ros::Subscriber target_acc_sub;
    ros::Subscriber target_mode_sub;
    ros::Timer timer1;

    tf2_ros::TransformBroadcaster tf_br;

    void subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void subAccCb(const geometry_msgs::AccelStamped::ConstPtr& msg);
    void subModeCb(const std_msgs::Int16::ConstPtr& msg);
    void timer1Cb(const ros::TimerEvent&);

    FlightLogger* logger_ptr;
};



#endif