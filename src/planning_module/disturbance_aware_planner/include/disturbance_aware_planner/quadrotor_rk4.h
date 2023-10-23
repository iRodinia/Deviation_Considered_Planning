#ifndef QUAD_RK4_HPP
#define QUAD_RK4_HPP

#include <vector>
#include <Eigen/Eigen>

#include "disturbance_aware_planner/attitude_transformation.h"

template <class T, class state, class input>
state RK4_step(T *sys, const state& x, const input& u, double dt){
    state k1 = sys->f(x, u)*dt;
    state k2 = sys->f(x+k1/2.0, u)*dt; 
    state k3 = sys->f(x+k2/2.0, u)*dt;
    state k4 = sys->f(x+k3, u)*dt;
    return x + (k1/6.0) + (k2/3.0) + (k3/3.0) + (k4/6.0);
};

class QuadRotor{
    typedef Eigen::Matrix<double, 13, 1> state;
    typedef Eigen::Matrix<double, 4, 1> input;
    typedef Eigen::Matrix<double, 13, 13> matA;
    typedef Eigen::Matrix<double, 13, 4> matB;
    // x = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, angx, angy, angz]
    // u = [f1, f2, f3, f4]
    //   f1  f4
    //     \/
    //     /\
    //   f2  f3    ↑: Head

public:
    QuadRotor() {}
    ~QuadRotor() {}
    inline void setParams(double grav, double mass, Eigen::Matrix<double, 3, 3> inertia, double armlen, double kf, double km);
    inline state f(const state& x, const input& u);
    inline matA dfdx(const state& x, const input& u);
    inline matB dfdu(const state& x, const input& u);
    inline state step(const state& x, const input& u, double dt);
    inline void getDiscreteModel(const state& x, const input& u, const double dt, matA& A, matB& B);
    inline void differential_flatness(Eigen::Vector3d ref_p, Eigen::Vector3d ref_v, Eigen::Vector3d ref_a, 
                                        Eigen::Vector3d ref_j, state& tmp_state, input& tmp_input);

private:
    double kf=0.6, km=0.15, arml=0.2, m=0.6, g=9.81;
    double gamma;
	Eigen::Matrix<double, 3, 3> J, Jinv; 
	Eigen::Vector3d e3;
};

inline void QuadRotor::setParams(double grav, double mass, Eigen::Matrix<double, 3, 3> inertia, double armlen, double kf, double km){
	J = inertia;
	Jinv = J.inverse();
    kf=kf;
    km=km;
    gamma = km/kf;
    arml=armlen;
    g=grav;
    m=mass;
    e3 = Eigen::Vector3d(0, 0, 1);
}

inline QuadRotor::state QuadRotor::f(const state& x, const input& u){
    double F = u(0) + u(1) + u(2) + u(3);
    Eigen::Vector3d M;
    M(0) = 0.7071 * (u(0)+u(1)-u(2)-u(3));
    M(1) = 0.7071 * (-u(0)+u(1)+u(2)-u(3));
    M(2) = gamma * (u(0)-u(1)+u(2)-u(3));
	state x_dot = state::Zero();
    x_dot.segment<3>(0) = x.segment<3>(3);
    Eigen::Vector3d body_z(2*x(7)*x(9) + 2*x(6)*x(8), 2*x(8)*x(9) - 2*x(6)*x(7), 1 - 2*x(7)*x(7) - 2*x(8)*x(8));
    x_dot.segment<3>(3) = F/m * body_z - g * e3;
    x_dot(6) = -0.5 * x.segment<3>(7).dot(x.segment<3>(10));
    Eigen::Matrix<double, 3, 3> q_cross = Eigen::Matrix<double, 3, 3>::Zero();
    q_cross(0,1) = -x(9); q_cross(1,0) = x(9);
    q_cross(0,2) = x(8); q_cross(2,0) = -x(8);
    q_cross(1,2) = -x(6); q_cross(2,1) = x(6);
    x_dot.segment<3>(7) = 0.5 * (x(6)*Eigen::Matrix<double, 3, 3>::Identity() + q_cross) * x.segment<3>(10);
    x_dot.segment<3>(10) = Jinv * (-x.segment<3>(10).cross(J*x.segment<3>(10)) + M);
    return x_dot;
}

inline QuadRotor::state QuadRotor::step(const state& x, const input& u, double dt){
	return RK4_step<QuadRotor, state, input>(this, x, u, dt);
}

inline QuadRotor::matA QuadRotor::dfdx(const state& x, const input& u){
    double F = u(0) + u(1) + u(2) + u(3);

    matA A = matA::Zero();
    A.block<3,3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    double F_div_m = F / m;
    A(3,6) = 2 * F_div_m * x(8);  A(3,7) = 2 * F_div_m * x(9);  A(3,8) = 2 * F_div_m * x(6);  A(3,9) = 2 * F_div_m * x(7);
    A(4,6) = -2 * F_div_m * x(7); A(4,7) = -2 * F_div_m * x(6); A(4,8) = 2 * F_div_m * x(9);  A(4,9) = 2 * F_div_m * x(8);
    A(5,6) = 0;                   A(5,7) = -4 * F_div_m * x(7); A(5,8) = -4 * F_div_m * x(8); A(5,9) = 0;

    A(6,6) = 0;           A(6,7) = -0.5 * x(10); A(6,8) = -0.5 * x(11); A(6,9) = -0.5 * x(12); A(6,10) = -0.5 * x(7); A(6,11) = -0.5 * x(8); A(6,12) = -0.5 * x(9);
    A(7,6) = 0.5 * x(10); A(7,7) = 0;            A(7,8) = 0.5 * x(12);  A(7,9) = -0.5 * x(11); A(7,10) = 0.5 * x(6);  A(7,11) = -0.5 * x(9); A(7,12) = 0.5 * x(8);
    A(8,6) = 0.5 * x(11); A(8,7) = -0.5 * x(12); A(8,8) = 0;            A(8,9) = 0.5 * x(10);  A(8,10) = 0.5 * x(9);  A(8,11) = 0.5 * x(6);  A(8,12) = -0.5 * x(7);
    A(9,6) = 0.5 * x(12); A(9,7) = 0.5 * x(11);  A(9,8) = -0.5 * x(10); A(9,9) = 0;            A(9,10) = -0.5 * x(8); A(9,11) = 0.5 * x(7);  A(9,12) = 0.5 * x(6);

    Eigen::Matrix<double, 3, 3> partial_cross_term;
    partial_cross_term(0,0) = J(2,0)*x(10) - J(1,0)*x(12);                            partial_cross_term(0,1) = J(2,0)*x(10) + 2*J(2,1)*x(11) + (J(2,2)-J(1,1))*x(12); partial_cross_term(0,2) = -J(1,0)*x(10) + (J(2,2)-J(1,1))*x(11) - 2*J(1,2)*x(12);
    partial_cross_term(1,0) = -2*J(2,0)*x(10) - J(2,1)*x(11) + (J(0,0)-J(2,2))*x(12); partial_cross_term(1,1) = -J(2,1)*x(10) + J(0,1)*x(12);                          partial_cross_term(1,2) = (J(0,0)-J(2,2))*x(10) + J(0,1)*x(11) + 2*J(0,2)*x(12);
    partial_cross_term(2,0) = 2*J(1,0)*x(10) + (J(1,1)-J(0,0))*x(11) + J(1,2)*x(12);  partial_cross_term(2,1) = (J(1,1)-J(0,0))*x(10) - 2*J(0,1)*x(11) - J(0,2)*x(12); partial_cross_term(2,2) = J(1,2)*x(10) - J(0,2)*x(11);
    A.block<3,3>(10,10) = -Jinv * partial_cross_term;
    
    return A;
}

inline QuadRotor::matB QuadRotor::dfdu(const state& x, const input& u){
    matB B = matB::Zero();
    Eigen::Vector3d kine_mat(2*x(7)*x(9) + 2*x(6)*x(8), 2*x(8)*x(9) - 2*x(6)*x(7), 1 - 2*x(7)*x(7) - 2*x(8)*x(8));
    B.block<3,4>(3,0) = 1/m * kine_mat * Eigen::RowVector4d(1, 1, 1, 1);
    Eigen::Matrix<double, 3, 4> dMdu;
    dMdu << 0.7071, 0.7071, -0.7071, -0.7071,
            -0.7071, 0.7071, 0.7071, -0.7071,
            gamma, -gamma, gamma, -gamma;
    B.block<3,4>(10,0) =  Jinv * dMdu;

    return B;
}

inline void QuadRotor::getDiscreteModel(const state& x, const input& u, const double dt, matA& A, matB& B){
    A = dt * dfdx(x, u);
    B = dt * dfdu(x, u);
}

inline void QuadRotor::differential_flatness(Eigen::Vector3d ref_p, Eigen::Vector3d ref_v, Eigen::Vector3d ref_a, 
                                                            Eigen::Vector3d ref_j, state& tmp_state, input& tmp_input){
    tmp_state.segment<3>(0) = ref_p;
    tmp_state.segment<3>(3) = ref_v;
    double tmp_vx = ref_v(0);
    double tmp_vy = ref_v(1);
    double tmp_yaw;
    if(tmp_vx == 0 && tmp_vy == 0){
        tmp_yaw = 0;
    }
    else{
        tmp_yaw = std::atan2(-1.0 * tmp_vy, tmp_vx);
    }
    Eigen::Vector3d thrust_dir = ref_a + Eigen::Vector3d(0, 0, g);

    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d tmp_rotmat;
    proj_xb_des << std::cos(tmp_yaw), std::sin(tmp_yaw), 0.0;
    zb_des = thrust_dir / thrust_dir.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / ( yb_des.cross(zb_des) ).norm();
    tmp_rotmat << xb_des(0), yb_des(0), zb_des(0),
                xb_des(1), yb_des(1), zb_des(1),
                xb_des(2), yb_des(2), zb_des(2);
    tmp_state.segment<4>(6) = rot2Quaternion(tmp_rotmat);

    Eigen::Vector3d h_w = 1/thrust_dir.norm() * (ref_j - (zb_des.dot(ref_j)*zb_des));
    tmp_state(10) = -h_w.dot(yb_des);
    tmp_state(11) = h_w.dot(xb_des);
    tmp_state(12) = 0;

    double nominal_f = m*thrust_dir.norm() / 4;
    tmp_input = nominal_f * Eigen::Vector4d(1, 1, 1, 1);   // enough for system linerization.
}

#endif