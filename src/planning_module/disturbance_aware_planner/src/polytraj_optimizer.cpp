#include "disturbance_aware_planner/polytraj_optimizer.h"

using namespace disturbance_aware_planner;

void PolyTrajOptimizer::setParameters(ros::NodeHandle& nh){
    nh.param("Optimization/poly_order", poly_order, 4);
    nh.param("Optimization/predict_num", pred_N, 100);
    nh.param("Optimization/predict_dt", pred_dt, 0.02);
    nh.param("Optimization/smoothness_cost_weight", w_smooth, 1.0);
    nh.param("Optimization/frs_cost_weight", w_frs, 1.0);
    nh.param("Optimization/terminal_cost_weight", w_terminal, 1.0);
    pred_T = pred_dt * pred_N;
    nh.param("Optimization/max_constraints_num", max_cons_num, 20);
    convex_layer_num = 0;
    cons_point_num = 0;
    cons_dt = 0;
    nh.param("Optimization/smoothness_cost_order", smoothness_cost_order, 4);
    smooth_cost_Q = Eigen::MatrixXd(poly_order+1, poly_order+1).setZero();
    Eigen::RowVectorXd tmp_vec = Eigen::RowVectorXd::LinSpaced(smoothness_cost_order, 0.0, smoothness_cost_order-1.0);
    for(int i=0; i<poly_order+1; i++){
        for(int j=0; j<poly_order+1; j++){
            if(i >= smoothness_cost_order && j >= smoothness_cost_order){
                int pow_ = i + j - 2*smoothness_cost_order + 1;
                Eigen::RowVectorXd coef_mat = (i*Eigen::RowVectorXd::Ones(tmp_vec.size()) - tmp_vec) * 
                    (j*Eigen::RowVectorXd::Ones(tmp_vec.size()) - tmp_vec).adjoint();
                smooth_cost_Q(i,j) = coef_mat.prod() * std::pow(pred_T,pow_) / pow_;
            }
        }
    }

    double mass, arm_len, kf, km;
    Eigen::Matrix<double, 3, 3> Inertia = Eigen::Matrix<double, 3, 3>::Zero();
    nh.param("Model/mass", mass, 0.027);
    nh.param("Model/inertia_xx", Inertia(0,0), 1.4e-5);
    nh.param("Model/inertia_yy", Inertia(1,1), 1.4e-5);
    nh.param("Model/inertia_zz", Inertia(2,2), 2.17e-5);
    nh.param("Model/arm_length", arm_len, 0.0397);
    nh.param("Model/k_force", kf, 3.16e-10);
    nh.param("Model/k_moment", km, 7.94e-12);
    quad.setParams(9.8066, mass, Inertia, arm_len, kf, km);

    coef_c0 = Coef(0, 0, 0);
    coef_c1 = Coef(0, 0, 0);
    coef_c2 = Coef(0, 0, 0);
    init_rest_coefs = Coefs(poly_order - 2);
    opter = nlopt::opt(nlopt::LN_COBYLA, 100);
    ready_for_optim = false;
}

void PolyTrajOptimizer::setStates(const Point init_p, const Point init_v, const Point init_a, 
                                    const Point goal_p, const Point goal_vdir){
    init_p_ = init_p;
    init_v_ = init_v;
    init_a_ = init_a;
    goal_p_ = goal_p;
    goal_vdir_ = goal_vdir;

    coef_c0 = init_p;   // fixed
    coef_c1 = init_v;   // fixed
    coef_c2 = 0.5 * init_a;   // fixed
    init_rest_coefs.block<3,1>(0,0) = 1 / (pred_T*pred_T*pred_T) * (goal_p - 0.5*pred_T*pred_T*init_a - pred_T*init_v - init_p);   // float

    ready_for_optim = true;
}


void PolyTrajOptimizer::setCollisionConstraints(const std::vector<sfcCoef>& constraintsA, const std::vector<sfcBound>& constraintsB){
    convex_layer_num = 0;
    if(constraintsA.size() < constraintsB.size()){
        convex_layer_num = constraintsA.size();
    }
    else{
        convex_layer_num = constraintsB.size();
    }
    constraintsA_.resize(convex_layer_num);
    constraintsB_.resize(convex_layer_num);
    constraintsA_ = std::vector<sfcCoef>(constraintsA.begin(), constraintsA.begin()+convex_layer_num);
    constraintsB_ = std::vector<sfcBound>(constraintsB.begin(), constraintsB.begin()+convex_layer_num);
    cons_point_num = std::max(int(max_cons_num/convex_layer_num), 1);
    cons_dt = pred_T / (cons_point_num+1);
}

bool PolyTrajOptimizer::optimize(){

}

double PolyTrajOptimizer::wrapTotalCost(const std::vector<double>& x, std::vector<double>& grad, void* data){
    PolyTrajOptimizer* optimizer = reinterpret_cast<PolyTrajOptimizer*>(data);
    int coef_num = optimizer->poly_order + 1;
    Coefs new_coefs(coef_num);
    new_coefs.block<3,1>(0,0) = optimizer->coef_c0;
    new_coefs.block<3,1>(0,1) = optimizer->coef_c1;
    new_coefs.block<3,1>(0,2) = optimizer->coef_c2;
    for(int i=3; i<coef_num; i++){
        new_coefs(0,i) = x[i-3];
        new_coefs(1,i) = x[coef_num+i-6];
        new_coefs(2,i) = x[2*coef_num+i-9];
    }
    optimizer->poly_traj.setCoefficients(new_coefs);
    double smoothness_cost = optimizer->calcSmoothnessCost(grad);
    double frs_cost = optimizer->calcFRSCost(grad);
    double terminal_cost = optimizer->calcTerminalCost(grad);
    return smoothness_cost + frs_cost + terminal_cost;
}

void PolyTrajOptimizer::wrapTotalConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
    PolyTrajOptimizer* optimizer = reinterpret_cast<PolyTrajOptimizer*>(f_data);
    int mid_point_num = optimizer->cons_point_num;
    int convex_num = optimizer->convex_layer_num;
    if(m != mid_point_num*convex_num){
        ROS_WARN("In polynomial optimization, expect %d linear constraints (%d points and %d inequalities for each point), \
                    but get %d in constraints c(x)<=0.", mid_point_num*convex_num, mid_point_num, convex_num, m);
        ros::shutdown();
        exit(-1);
    }
    int coef_num = optimizer->poly_order + 1;
    double mid_dt = optimizer->cons_dt;
    if(n != 3*(coef_num-3)){
        ROS_WARN("In polynomial optimization, expect %d optimization variables (polynomial order %d with c0, c1, c2 predifined), \
                    but get %d in optimizer.", 3*(coef_num-3), coef_num-1, n);
        ros::shutdown();
        exit(-1);
    }
    Coefs new_coefs(coef_num);
    new_coefs.block<3,1>(0,0) = optimizer->coef_c0;
    new_coefs.block<3,1>(0,1) = optimizer->coef_c1;
    new_coefs.block<3,1>(0,2) = optimizer->coef_c2;
    for(int i=3; i<coef_num; i++){
        new_coefs(0,i) = x[i-3];
        new_coefs(1,i) = x[coef_num+i-6];
        new_coefs(2,i) = x[2*coef_num+i-9];
    }
    optimizer->poly_traj.setCoefficients(new_coefs);
    for(int i=1; i<=mid_point_num; i++){
        Point eval_point = optimizer->poly_traj.evaluate(i*mid_dt);
        for(int k=0; k<convex_num; k++){
            sfcCoef sfc_coef = optimizer->constraintsA_[k];
            sfcBound bound = optimizer->constraintsB_[k];
            result[convex_num*(i-1)+k] = sfc_coef*eval_point - bound;
        }
    }
}

double PolyTrajOptimizer::calcSmoothnessCost(std::vector<double>& grad){
    Coefs p = poly_traj.getCoefficients();
    Eigen::Matrix<double, 3, 3> costs = p * smooth_cost_Q * p.transpose();
    return costs(0,0) + costs(1,1) + 1.2*costs(2,2);
}

double PolyTrajOptimizer::calcFRSCost(std::vector<double>& grad){
    std::vector<quadState> flat_states(pred_N, quadState::Zero());
    std::vector<quadInput> flat_inputs(pred_N, quadInput::Zero());
    this->getFlatStatesInputes(flat_states, flat_inputs);
    
}

double PolyTrajOptimizer::calcTerminalCost(std::vector<double>& grad){
    Point end_p = poly_traj.evaluate(pred_T);
    Polynomial<3> deriv1 = poly_traj.derivative();
    double pos_cost = (end_p - goal_p_).norm();
    Point end_v = deriv1.evaluate(pred_T);
    double ang_cost = 1 - end_v.dot(goal_vdir_) / (end_v.norm()*goal_vdir_.norm());
    return pos_cost + 1.5*ang_cost;
}

void PolyTrajOptimizer::getFlatStatesInputes(std::vector<quadState>& return_states, std::vector<quadInput>& return_inputes){
    Polynomial<3> deriv1 = poly_traj.derivative();
    Polynomial<3> deriv2 = deriv1.derivative();
    Polynomial<3> deriv3 = deriv2.derivative();
    std::fill(return_states.begin(), return_states.end(), quadState::Zero());
    std::fill(return_inputes.begin(), return_inputes.end(), quadInput::Zero());
    for(int i=0; i<pred_N; i++){
        double t_ = i*pred_dt;
        quadState tmp_state;
        quadInput tmp_input;
        quad.differential_flatness(poly_traj.evaluate(t_), deriv1.evaluate(t_), deriv2.evaluate(t_), 
                                    deriv3.evaluate(t_), tmp_state, tmp_input);
        return_states[i] = tmp_state;
        return_inputes[i] = tmp_input;
    }
}