#ifndef PLANNER_NODE
#define PLANNER_NODE

#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "disturbance_aware_planner/polytraj_optimizer.h"
#include "disturbance_aware_planner/reference_manager.h"

namespace disturbance_aware_planner{

class DisturbAwareManager {
public:
    DisturbAwareManager(ros::NodeHandle* nodehandle);
    ~DisturbAwareManager() {};
    void initSubscribers();
    void initPublishers();
    void initTimers();

    void initManager(ros::NodeHandle &nh);



    ros::Timer timer1;
    geometry_msgs::PoseStamped current_pos;
    geometry_msgs::TwistStamped current_vel;
    std::shared_ptr<DisturbAwareManager> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::Subscriber local_pos_sub;
    ros::Publisher local_vel_pub;
    ros::Subscriber local_vel_sub;
    ros::Subscriber target_pos_sub;
    ros::Subscriber target_vel_sub;

    ReferenceManager traj_buffer;

};



}

#endif