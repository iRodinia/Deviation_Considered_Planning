#include "disturbance_aware_planner/reference_manager.h"

using namespace disturbance_aware_planner;


ReferenceManager::ReferenceManager(){
    last_cmd = Eigen::VectorXd::Zero(13);
}

void ReferenceManager::setNewTraj(Eigen::Matrix<double, 3, -1> coefs, double t_horizon, double start_t_bias){
    tmp_traj = Polynomial3D(coefs);
    tmp_traj_d1 = tmp_traj.derivative();
    tmp_traj_d2 = tmp_traj_d1.derivative();
    traj_time_len = std::max(t_horizon, 0.0);
    traj_start_time = ros::Time::now() - ros::Duration(start_t_bias);
    initialized = true;
}

ReferenceManager::RefState ReferenceManager::getRefCmd_Full(){
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