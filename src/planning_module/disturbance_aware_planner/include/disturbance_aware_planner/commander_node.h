#ifndef DAP_COMMANDER
#define DAP_COMMANDER

#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "disturbance_aware_planner/reference_manager.h"

namespace disturbance_aware_planner{

class FlightCommander{
public:
    FlightCommander(ros::NodeHandle* nodehandle);
    ~FlightCommander() {};




    ros::Timer timer1, timer2;
    Eigen::Vector3d current_pos;
    Eigen::Vector3d current_vel;
    Eigen::Vector3d current_acc;
    Eigen::Vector4d current_att;
    Eigen::Vector3d current_angrate;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



private:
    ros::NodeHandle nh_;

    ros::Subscriber local_pos_sub;   // include pos and att
    ros::Subscriber local_vel_sub;   // include vel and angrate
    ros::Subscriber flight_mode_sub;   // 0: land, 1: takeoff, 2: offb_ctrl
    ros::Publisher target_pos_pub;   // include pos and att
    ros::Publisher target_vel_pub;   // include vel and angrate
    ros::Publisher target_acc_pub;
    ros::Publisher target_mode_pub;

    ReferenceManager traj_buffer;

    void subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void subModeCb(const std_msgs::Int16::ConstPtr& msg);
    void timer1Cb(const ros::TimerEvent&);
    void timer2Cb(const ros::TimerEvent&);

    double cmd_freq = 10;   // frequency for sending commands
    double replan_freq = 1;   // frequency for conducting replanning
    Eigen::Matrix3Xd local_pcl;
    Eigen::Vector3d start_pos, goal_pos;

    int at_pos_count = 0;



};

}


#endif