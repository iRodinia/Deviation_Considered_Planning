#include <vector>
#include <string>
#include <cmath>

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <ros/ros.h>
#include <nlopt.hpp>

#include "disturbance_aware_planner/quadrotor_rk4.h"

namespace disturbance_aware_planner{

template<int Dim>
class Polynomial{
    typedef Eigen::Matrix<double, Dim, 1> Coef;
    typedef Eigen::Matrix<double, Dim, -1> Coefs;
    typedef Eigen::Matrix<double, Dim, 1> Value;
public:
    Polynomial(){}
    Polynomial(const std::vector<Coef>& coefficients){
        if(coefficients.size() <= 0){
            order_ = 0;
            coefficients_ = Coef(0, 0, 0);
        }
        else{
            order_ = coefficients.size() - 1;
            coefficients_ = Coefs(order_+1)::Zero();
            for(int i=0; i<=order_; i++){
                coefficients_.block<3,1>(0,i) = coefficients[i];
            }
        }
    }

    void setCoefficients(const Coefs& coefficients){
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

    Polynomial derivative() const {
        std::vector<Coef> derivative_coefficients;
        for (size_t i = 1; i <= order_; ++i) {
            derivative_coefficients.push_back(i * coefficients_.block<3,1>(0,i));
        }
        return Polynomial(derivative_coefficients);
    }

private:
    int order_;  // order = n
    Coefs coefficients_;   // [c0, c1, c2, ..., cn]
};


class PolyTrajOptimizer{
    typedef Eigen::Matrix<double, 3, 1> Coef;
    typedef Eigen::Matrix<double, 3, -1> Coefs;
    typedef Eigen::Matrix<double, 3, 1> Point;
    typedef Eigen::Matrix<double, 1, 3> sfcCoef;
    typedef double sfcBound;
    typedef Eigen::Matrix<double, 13, 1> quadState;
    typedef Eigen::Matrix<double, 4, 1> quadInput;
    typedef Eigen::Matrix<double, 13, 13> quadLinA;
    typedef Eigen::Matrix<double, 13, 4> quadLinB;

public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}
    void setParameters(ros::NodeHandle &nh);
    void setStates(const Point init_p, const Point init_v, const Point init_a, const Point goal_p, const Point goal_vdir);
    void setCollisionConstraints(const std::vector<sfcCoef>& constraintsA, const std::vector<sfcBound>& constraintsB);

    bool optimize();
    Coefs getTrajectoryCoefficients();
    Polynomial<3> poly_traj;

protected:
    static double wrapTotalCost(const std::vector<double>& x, std::vector<double>& grad, void* data);
    double calcSmoothnessCost(std::vector<double>& grad);
    double calcFRSCost(std::vector<double>& grad);
    double calcTerminalCost(std::vector<double>& grad);
    static void wrapTotalConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

private:
    int pred_N;
    double pred_dt, pred_T;
    int poly_order;
    int smoothness_cost_order;   // 3: minimize jerk, 4: minimize snap
    int max_cons_num, cons_point_num, convex_layer_num;
    double cons_dt;
    double w_smooth, w_frs, w_terminal;

    Point init_p_, init_v_, init_a_;
    Point goal_p_, goal_vdir_;
    Coef coef_c0, coef_c1, coef_c2;   // determined by the initial conditions in setStates(*).
    Coefs rest_coefs;
    std::vector<sfcCoef> constraintsA_;
    std::vector<sfcBound> constraintsB_;
    Eigen::MatrixXd smooth_cost_Q;
    QuadRotor quad;
    nlopt::opt opter;
    bool ready_for_optim;

    void getFlatStatesInputes(std::vector<quadState>& return_states, std::vector<quadInput>& return_inputes);
    Eigen::Vector4d getMotorNoise(Point pos);

public:
    std::shared_ptr<PolyTrajOptimizer> Ptr;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

};