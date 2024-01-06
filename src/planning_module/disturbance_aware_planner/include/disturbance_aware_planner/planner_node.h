#ifndef DAP_PLANNER
#define DAP_PLANNER

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include "reference_governor/polyTraj.h"
#include "disturbance_aware_planner/global_map_process.h"
#include "disturbance_aware_planner/polytraj_optimizer.h"
#include "disturbance_loader/disturb_loader.h"

#include "flight_logger/logger.h"

namespace disturbance_aware_planner{

class DisturbanceAwarePlanner{
public:
    DisturbanceAwarePlanner(ros::NodeHandle* nodehandle);
    ~DisturbanceAwarePlanner();
    void dumpParams();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    ros::NodeHandle nh_;

    ros::Subscriber local_pos_sub;   // include pos and att
    ros::Subscriber local_vel_sub;   // include vel and angrate
    ros::Subscriber flight_mode_sub;   // 0: land, 1: takeoff, 2: offb_ctrl
    ros::Subscriber disturb_map_sub;
    ros::Publisher ref_traj_pub;
    ros::Timer timer1;

    int current_ctrl_mode = 0;
    Eigen::Vector3d current_pos;
    Eigen::Vector3d current_vel;
    Eigen::Vector3d current_acc;
    Eigen::Vector4d current_att;
    Eigen::Vector3d current_angrate;

    GlobalMapProcessor::Ptr global_ptr;
    PolyTrajOptimizer::Ptr opter_ptr;
    DisturbMap::Ptr disturb_ptr;

    void subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void subModeCb(const std_msgs::Int16::ConstPtr& msg);
    void subDisturbCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void timer1Cb(const ros::TimerEvent&);

    Eigen::Vector3d start_pos, goal_pos;
    double replan_t_hori;
    ros::Duration replan_dur = ros::Duration(0.0);

    bool enable_log;
    ros::Time start_log_ts;
    std::shared_ptr<FlightLogger> logger_ptr;
};

}


#endif