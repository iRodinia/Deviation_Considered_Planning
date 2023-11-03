#ifndef REF_MANAGER
#define REF_MANAGER

#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <ros/ros.h>

namespace disturbance_aware_planner{

class Polynomial3D{
    typedef Eigen::Matrix<double, 3, 1> Coef;
    typedef Eigen::Matrix<double, 3, -1> Coefs;
    typedef Eigen::Matrix<double, 3, 1> Value;
public:
    Polynomial3D() {order_ = 0;}
    Polynomial3D(const Coefs& coefficients){
        if(coefficients.cols() <= 0){
            order_ = 0;
            coefficients_ = Coef(0, 0, 0);
        }
        else{
            order_ = coefficients.cols() - 1;
            coefficients_ = coefficients;
        }
    }

    Coefs getCoefficients(){
        return coefficients_;
    }

    Value evaluate(double x) const {
        Value result = Value::Zero();
        for (size_t i = 0; i <= order_; ++i) {
            result += coefficients_.block<3,1>(0,i) * std::pow(x, i);
        }
        return result;
    }

    Polynomial3D derivative() const {
        Coefs derivative_coefficients(order_);
        for (size_t i = 1; i <= order_; ++i) {
            derivative_coefficients.block<3,1>(0,i-1) = i * coefficients_.block<3,1>(0,i);
        }
        return Polynomial3D(derivative_coefficients);
    }

private:
    int order_;  // order = n
    Coefs coefficients_;   // [c0, c1, c2, ..., cn]
};


class ReferenceManager{
    typedef Eigen::Matrix<double, 13, 1> RefState;    // [pos, vel, acc, yaw, omega]

public:
    ReferenceManager();
    ~ReferenceManager() {};
    void setNewTraj(Eigen::Matrix<double, 3, -1> coefs, double t_horizon, double start_t_bias);
    RefState getRefCmd_Full();
    bool initialized = false;

private:
    Polynomial3D tmp_traj;
    Polynomial3D tmp_traj_d1;
    Polynomial3D tmp_traj_d2;
    double traj_time_len;
    ros::Time traj_start_time;
    RefState last_cmd;
};


}

#endif