#ifndef REF_GOVERNOR
#define REF_GOVERNOR

#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include "reference_governor/polynomial3d.h"
#include "reference_governor/polyTraj.h"

namespace disturbance_aware_planner{

class ReferenceGovernor{
    typedef Eigen::Matrix<double, 13, 1> RefState;    // [pos, vel, acc, yaw, omega]

public:
    ReferenceGovernor(ros::NodeHandle* node);
    ~ReferenceGovernor() {};
    void setNewTraj(Eigen::Matrix<double, 3, -1> coefs, double t_horizon, ros::Time start_time);
    RefState getRefCmd_Full();

private:
    ros::NodeHandle nh;
    ros::Subscriber traj_sub;
    ros::Subscriber local_pos_sub;   // include pos and att
    ros::Subscriber local_vel_sub;   // include vel and angrate
    ros::Subscriber flight_mode_sub;   // 0: land, 1: takeoff, 2: offb_ctrl
    ros::Publisher target_pos_pub;   // include pos and att
    ros::Publisher target_vel_pub;   // include vel and angrate
    ros::Publisher target_acc_pub;
    ros::Publisher target_mode_pub;
    ros::Timer timer1;

    int current_ctrl_mode = 0;
    Eigen::Vector3d current_pos;
    Eigen::Vector3d current_vel;
    Eigen::Vector3d current_acc;
    Eigen::Vector4d current_att;   // quaternion [w, x, y ,z]
    Eigen::Vector3d current_angrate;

    Polynomial3D tmp_traj;
    Polynomial3D tmp_traj_d1;
    Polynomial3D tmp_traj_d2;
    double traj_time_len;
    ros::Time traj_start_time;
    RefState last_cmd;
    Eigen::Vector3d init_ref_pos, goal_pos;
    bool traj_set;
    bool cmd_takeoff;
    bool cmd_offb;
    bool goal_reached;
    bool land_after_complete;
    int mode_change_count;
    int at_goal_pos_count;

    void trajSubCb(const reference_governor::polyTraj::ConstPtr& msg);
    void subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void subModeCb(const std_msgs::Int16::ConstPtr& msg);
    void timer1Cb(const ros::TimerEvent&);   // call at each command iteration
};


}

#endif