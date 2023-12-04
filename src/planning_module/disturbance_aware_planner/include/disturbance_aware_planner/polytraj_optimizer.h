#ifndef DAP_OPTIMIZER
#define DAP_OPTIMIZER

#include <vector>
#include <string>
#include <cmath>
#include <numeric>

#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <ros/ros.h>
#include <nlopt.hpp>

#include "disturbance_aware_planner/math_utils/quadrotor_rk4.h"

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
            coefficients_ = Coefs::Zero(3, order_+1);
            for(int i=0; i<=order_; i++){
                coefficients_.col(i) = coefficients[i];
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
            result += coefficients_.col(i) * std::pow(x, i);
        }
        return result;
    }

    Polynomial derivative() const {
        std::vector<Coef> derivative_coefficients;
        for (size_t i = 1; i <= order_; ++i) {
            derivative_coefficients.push_back(i * coefficients_.col(i));
        }
        return Polynomial(derivative_coefficients);
    }

private:
    int order_;  // order = n
    Coefs coefficients_;   // [c0, c1, c2, ..., cn]
};

template<int Dim>
class ConvexHull{
    // Ax <= b    Ax - b <= 0
    typedef Eigen::Matrix<double, 1, Dim> rowA;
    typedef double rowB;
    typedef Eigen::Matrix<double, Dim, 1> Point;
public:
    ConvexHull() {}
    ConvexHull(const std::vector<rowA> Ain, const std::vector<rowB> Bin){
        if(Ain.size() != Bin.size()){
            ROS_WARN("Mismatch convex hull dimensions! Get %d in A and %d in B.", Ain.size(), Bin.size());
            constraint_num = std::min(Ain.size(), Bin.size());
        }
        else{
            constraint_num = Ain.size();
        }
        A.assign(Ain.begin(), Ain.begin()+constraint_num);
        B.assign(Bin.begin(), Bin.begin()+constraint_num);
    }

    ConvexHull(const Eigen::MatrixX4d ABin){
        constraint_num = ABin.rows();
        A.resize(constraint_num);
        B.resize(constraint_num);
        for(int i=0; i<constraint_num; i++){
            A[i] = ABin.block<1,3>(i,0);
            B[i] = -ABin(i,3);
        }
    }

    std::vector<double> evaluatePoint(Point x){
        std::vector<double> result(constraint_num, 0.0);
        for(int i=0; i<constraint_num; i++){
            result[i] = A[i]*x - B[i];
        }
        return result;
    }

    void getConstraints(std::vector<rowA>& Aout, std::vector<rowB>& Bout){
        Aout.resize(constraint_num);
        Bout.resize(constraint_num);
        Aout.assign(A.begin(), A.end());
        Bout.assign(B.begin(), B.end());
    }

private:
    int constraint_num = 0;
    std::vector<rowA> A;
    std::vector<rowB> B;
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
    void initParameters(ros::NodeHandle &nh);
    void setStates(const Point init_p, const Point init_v, const Point init_a, const Point goal_p, const Point goal_vdir);
    void setCollisionConstraints(const std::vector<Eigen::MatrixX4d>& constraints, const std::vector<double>& time_allocs);

    bool optimize();
    double getPredTimeHorizon();
    Coefs getTrajectoryCoefficients();
    Polynomial<3> poly_traj;
    std::vector<ConvexHull<3>> constraints_;

    static double wrapTotalCost(const std::vector<double>& x, std::vector<double>& grad, void* data);
    double calcSmoothnessCost(std::vector<double>& grad);
    double calcFRSCost(std::vector<double>& grad);
    double calcTerminalCost(std::vector<double>& grad);
    static void wrapTotalConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

    int pred_N;
    double pred_dt, pred_T;
    int poly_order;
    int smoothness_cost_order;   // 3: minimize jerk, 4: minimize snap
    int max_faces_num, constrained_point_num, total_cons_num;
    double cons_dt;
    double w_smooth, w_frs, w_terminal;

    Point init_p_, init_v_, init_a_;
    Point goal_p_, goal_vdir_;
    Coef coef_c0, coef_c1, coef_c2;   // determined by the initial conditions in setStates(*).
    Coefs rest_coefs;
    Eigen::MatrixXd smooth_cost_Q;
    QuadRotor quad;
    nlopt::opt opter;
    double uav_vel;
    bool ready_for_optim;

    void getFlatStatesInputes(std::vector<quadState>& return_states, std::vector<quadInput>& return_inputes);
    Eigen::Vector4d getMotorNoise(Point pos);

public:
    typedef std::shared_ptr<PolyTrajOptimizer> Ptr;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}

#endif