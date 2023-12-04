#include "disturbance_aware_planner/polytraj_optimizer.h"

using namespace disturbance_aware_planner;

void PolyTrajOptimizer::initParameters(ros::NodeHandle& nh){
    nh.param("Optimization/poly_order", poly_order, 6);
    nh.param("Optimization/predict_num", pred_N, 100);
    nh.param("Optimization/predict_dt", pred_dt, 0.02);
    nh.param("Optimization/smoothness_cost_weight", w_smooth, 1.0);
    nh.param("Optimization/frs_cost_weight", w_frs, 1e-6);
    nh.param("Optimization/terminal_cost_weight", w_terminal, 1.0);
    pred_T = pred_dt * pred_N;
    nh.param("Optimization/max_single_convex_hull_faces", max_faces_num, 20);
    nh.param("Optimization/total_constrained_points_in_SFC", constrained_point_num, 20);
    cons_dt = pred_T / constrained_point_num;
    constraints_.resize(constrained_point_num);
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
    nh.param("Model/nominal_vel", uav_vel, 1.2);
    quad.setParams(9.8066, mass, Inertia, arm_len, kf, km);

    coef_c0 = Coef(0, 0, 0);
    coef_c1 = Coef(0, 0, 0);
    coef_c2 = Coef(0, 0, 0);
    rest_coefs = Coefs(3, poly_order - 2);
    opter = nlopt::opt(nlopt::LN_COBYLA, rest_coefs.size());
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
    rest_coefs.setZero();
    rest_coefs.block<3,1>(0,0) = 1 / (pred_T*pred_T*pred_T) * (goal_p - 0.5*pred_T*pred_T*init_a - pred_T*init_v - init_p);   // float

    ready_for_optim = true;
}

void PolyTrajOptimizer::setCollisionConstraints(const std::vector<Eigen::MatrixX4d>& constraints, const std::vector<double>& time_allocs){
    if(constraints.size() != time_allocs.size()){
        ROS_WARN("Incompatible constraints number. Get %ld convex hulls and %ld time allocations.", constraints.size(), time_allocs.size());
        ros::shutdown();
    }
    int constraints_num = constraints.size();
    std::vector<double> time_splits;
    std::partial_sum(time_allocs.begin(), time_allocs.end(), std::back_inserter(time_splits));
    if(time_splits[constraints_num-1] <= pred_T){
        time_splits[constraints_num-1] = pred_T + pred_dt;
    }

    total_cons_num = 0;
    int seg_idx = 0;
    for(int i=1; i<=constrained_point_num; i++){
        double temp_t = i*cons_dt;
        while(temp_t > time_splits[seg_idx]){
            seg_idx++;
        }
        if(constraints[seg_idx].rows() > max_faces_num){
            constraints_[i-1] = ConvexHull<3>(constraints[seg_idx].topRows(max_faces_num));
            total_cons_num += max_faces_num;
        }
        else{
            constraints_[i-1] = ConvexHull<3>(constraints[seg_idx]);
            total_cons_num += constraints[seg_idx].rows();
        }
    }
}

bool PolyTrajOptimizer::optimize(){
    opter.remove_inequality_constraints();

    opter.set_min_objective(PolyTrajOptimizer::wrapTotalCost, this);
    std::vector<double> cons_tolerance(total_cons_num, 0.05);
    opter.add_inequality_mconstraint(PolyTrajOptimizer::wrapTotalConstraints, this, cons_tolerance);

    opter.set_xtol_rel(1e-1);
    // opter.set_maxeval(1e3);
    // opter.set_maxtime(0.25);

    std::vector<double> optim_x(rest_coefs.size(), 0.0);
    for(int i=0; i<poly_order-2; i++){
        optim_x[i] = rest_coefs(0,i);
        optim_x[poly_order-2+i] = rest_coefs(1,i);
        optim_x[2*poly_order-4+i] = rest_coefs(2,i);
    }
    double min_f;
    nlopt::result res = opter.optimize(optim_x, min_f);
    if(res < 0){
        ROS_WARN("Unable to solve the optimization problem. Get result code %d.", int(res));
        return false;
    }
    else{
        ROS_INFO("optimized cost: %f", min_f);
    }

    for(int i=0; i<poly_order-2; i++){
        rest_coefs(0,i) = optim_x[i];
        rest_coefs(1,i) = optim_x[poly_order-2+i];
        rest_coefs(2,i) = optim_x[2*poly_order-4+i];
    }
    return true;
}

double PolyTrajOptimizer::getPredTimeHorizon(){
    return pred_T;
}

PolyTrajOptimizer::Coefs PolyTrajOptimizer::getTrajectoryCoefficients(){
    int coef_num = poly_order + 1;
    Coefs new_coefs(3, coef_num);
    new_coefs.block<3,1>(0,0) = coef_c0;
    new_coefs.block<3,1>(0,1) = coef_c1;
    new_coefs.block<3,1>(0,2) = coef_c2;
    for(int i=3; i<coef_num; i++){
        new_coefs.block<3,1>(0,i) = rest_coefs.block<3,1>(0,i-3);
    }
    return new_coefs;
}

double PolyTrajOptimizer::wrapTotalCost(const std::vector<double>& x, std::vector<double>& grad, void* data){
    PolyTrajOptimizer* optimizer = reinterpret_cast<PolyTrajOptimizer*>(data);
    int coef_num = optimizer->poly_order + 1;
    Coefs new_coefs(3, coef_num);
    new_coefs.block<3,1>(0,0) = optimizer->coef_c0;
    new_coefs.block<3,1>(0,1) = optimizer->coef_c1;
    new_coefs.block<3,1>(0,2) = optimizer->coef_c2;
    for(int i=3; i<coef_num; i++){
        new_coefs(0,i) = x[i-3];
        new_coefs(1,i) = x[coef_num+i-6];
        new_coefs(2,i) = x[2*coef_num+i-9];
    }

    optimizer->poly_traj.setCoefficients(new_coefs);
    optimizer->smooth_cost_save = optimizer->calcSmoothnessCost(grad);
    double smoothness_cost = optimizer->w_smooth * optimizer->smooth_cost_save;
    optimizer->frs_cost_save = optimizer->calcFRSCost(grad);
    double frs_cost = optimizer->w_frs * optimizer->frs_cost_save;
    optimizer->terminal_cost_save = optimizer->calcTerminalCost(grad);
    double terminal_cost = optimizer->w_terminal * optimizer->terminal_cost_save;

    return smoothness_cost + frs_cost + terminal_cost;
}

void PolyTrajOptimizer::wrapTotalConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
    PolyTrajOptimizer* optimizer = reinterpret_cast<PolyTrajOptimizer*>(f_data);
    int mid_point_num = optimizer->constrained_point_num;
    int cons_num = optimizer->total_cons_num;
    if(m != cons_num){
        ROS_WARN("In polynomial optimization, expect %d linear constraints, but get %d in constraints c(x)<=0.", cons_num, m);
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
    Coefs new_coefs(3, coef_num);
    new_coefs.block<3,1>(0,0) = optimizer->coef_c0;
    new_coefs.block<3,1>(0,1) = optimizer->coef_c1;
    new_coefs.block<3,1>(0,2) = optimizer->coef_c2;
    for(int i=3; i<coef_num; i++){
        new_coefs(0,i) = x[i-3];
        new_coefs(1,i) = x[coef_num+i-6];
        new_coefs(2,i) = x[2*coef_num+i-9];
    }
    optimizer->poly_traj.setCoefficients(new_coefs);
    int cons_idx = 0;
    for(int i=1; i<=mid_point_num; i++){
        Point eval_point = optimizer->poly_traj.evaluate(i*mid_dt);
        std::vector<double> eval_cons = optimizer->constraints_[i-1].evaluatePoint(eval_point);
        for(int j=0; j<eval_cons.size(); j++){
            result[cons_idx] = eval_cons[j];
            cons_idx++;
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

    Eigen::Matrix<double, 17, 17> M = Eigen::Matrix<double, 17, 17>::Zero();
    Eigen::Matrix<double, 17, 17> F = Eigen::Matrix<double, 17, 17>::Zero();
    F.block<4,4>(13,13) = Eigen::Matrix<double, 4, 4>::Identity();
    for(int i=0; i<pred_N; i++){
        Eigen::Vector4d disturb = getMotorNoise(flat_states[i].segment<3>(0));
        for(int j=0; j<4; j++){
            disturb(j) = disturb(j) * disturb(j);
        }
        M.block<4,4>(13,13) = disturb.asDiagonal();
        F.block<13,13>(0,0) = Eigen::Matrix<double, 13, 13>::Identity() + pred_dt*quad.dfdx(flat_states[i], flat_inputs[i]);
        F.block<13,4>(0,13) = pred_dt*quad.dfdu(flat_states[i], flat_inputs[i]);
        M = F * M * F.transpose();
    }
    Eigen::Matrix<double, 3, 3> E_pos = M.block<3,3>(0,0);
    double determ = std::abs(E_pos.determinant());
    // return 4/3 * M_PI * determ;
    return determ;

    // Plotting the ellipsoid
	// Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> adjoint_eigen_solver((E_pos + E_pos.transpose()) / 2.);
	// Eigen::MatrixXd mata = Eigen::MatrixXd::Zero(3, 1);
	// Eigen::Matrix<double, 3, 3> E_sqrt_pos = adjoint_eigen_solver.eigenvectors() * adjoint_eigen_solver.eigenvalues().cwiseMax(mata).cwiseSqrt().asDiagonal() * adjoint_eigen_solver.eigenvectors().transpose();
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

Eigen::Vector4d PolyTrajOptimizer::getMotorNoise(Point pos){
    return Eigen::Vector4d(0.02, 0.02, 0.02, 0.02) * pred_dt;
}