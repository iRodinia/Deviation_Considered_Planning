#ifndef REF_GOVERNOR
#define REF_GOVERNOR

#include <vector>
#include <cmath>
#include <string>
#include <memory>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "flight_logger/logger.h"
#include "reference_governor/polynomial3d.h"
#include "reference_governor/polyTraj.h"

namespace disturbance_aware_planner{

class ReferenceGovernor{
    typedef Eigen::Matrix<double, 13, 1> RefState;    // [pos, vel, acc, yaw, omega]

public:
    ReferenceGovernor(ros::NodeHandle* node);
    ~ReferenceGovernor();
    void setNewTraj(Eigen::Matrix<double, 3, -1> coefs, double t_horizon, ros::Time start_time);
    RefState getRefCmd_Full();

    void dumpParams();

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
    ros::Publisher local_ref_traj_pub;
    ros::Timer timer1, timer2;

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
    double cmd_freq, traj_plot_freq;
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
    std::string world_frame_id;
    double goal_pos_delta;
    bool enable_log;
    ros::Time start_log_ts;

    void trajSubCb(const reference_governor::polyTraj::ConstPtr& msg);
    void subPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void subModeCb(const std_msgs::Int16::ConstPtr& msg);
    void timer1Cb(const ros::TimerEvent&);   // call at each command iteration
    void timer2Cb(const ros::TimerEvent&);   // call at each traj plot iteration

    std::shared_ptr<FlightLogger> logger_ptr;
};


}

#endif