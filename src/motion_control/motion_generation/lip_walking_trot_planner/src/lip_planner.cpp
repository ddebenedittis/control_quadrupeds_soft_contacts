#include "lip_walking_trot_planner/lip_planner.hpp"

#include "quadprog/quadprog.hpp"

#include <algorithm>



namespace lip_walking_trot_planner
{

MatrixXd kron_product(const MatrixXd& A, const MatrixXd& B)
{
    MatrixXd C(A.rows() * B.rows(), A.cols() * B.cols());

    for (int i = 0; i < A.rows(); i++) {
        for (int j = 0; j < A.cols(); j++) {
            C.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = 
                A(i, j) * B;
        }
    }

    return C;
}

VectorXd kron_product(const VectorXd& a, const VectorXd& b)
{
    VectorXd c(a.size() * b.size());

    for (int i = 0; i < a.size(); i++) {
        c.segment(i * b.size(), b.size()) = 
            a(i) * b;
    }

    return c;
}

MotionPlanner::MotionPlanner()
{
    update_initial_conditions();
}


/* ======================== Update_initial_conditions ======================= */

void MotionPlanner::update_initial_conditions(const Ref<Vector3d>& p_0, double yaw)
{
    _dtheta = yaw;

    _init_pos_swing_feet.resize(4);

    // Initialize the _init_pos_swing_feet with the positions of the feet relative to the base in base frame.
    // LF
    _init_pos_swing_feet[0] <<   _r * std::cos(_theta_0 + _dtheta),
                                 _r * std::sin(_theta_0 + _dtheta),
                                 0.0;

    // RF
    _init_pos_swing_feet[1] <<   _r * std::cos(_theta_0 - _dtheta),
                               - _r * std::sin(_theta_0 - _dtheta),
                                 0.0;

    // LH
    _init_pos_swing_feet[2] << - _r * std::cos(_theta_0 - _dtheta),
                                 _r * std::sin(_theta_0 - _dtheta),
                                 0.0;

    // RH
    _init_pos_swing_feet[3] << - _r * std::cos(_theta_0 + _dtheta),
                               - _r * std::sin(_theta_0 + _dtheta),
                                 0.0;

    // Shift the quantity by the initial position of the base.
    for (auto& foot_pos : _init_pos_swing_feet) {
        foot_pos += p_0;
    }    
}


/* =============================== Compute_A_t ============================== */

MatrixXd MotionPlanner::compute_A_t(double omega, double t)
{
    MatrixXd A(2,2);

    A << std::cosh(omega*t),         1/omega * std::sinh(omega*t),
         omega * std::sinh(omega*t), std::cosh(omega*t);

    return A;
}


/* =============================== Compute_b_t ============================== */

MatrixXd MotionPlanner::compute_b_t(double omega, double t)
{
    VectorXd b(2);

    b << 1 - std::cosh(omega*t),
         - omega * std::sinh(omega*t);

    return b;
}


/* ================================= Mpc_qp ================================= */

double MotionPlanner::mpc_qp(
    const Vector2d& Xcom_0, double p_0,
    double xcom_dot_des
) {
    /* ================================ State =============================== */
        
    // xi = [
    //     1                             extra state equal to one
    //     [xcom_i, x_com_dot_i].T       position and velocity coordinate (only one coordinate)
    //     px                            ZMP coordinate
    // ]

    double Ts = _interpolator.get_step_duration();


    /* =========================== QP Formulation =========================== */

    // min_xi   1/2 xi^T H xi + c^T xi
    // s.t.: A xi - b  = 0
    //       D xi - f <= 0


    /* =========================== Cost Definition ========================== */

    // cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]

    // Part of the cost associated with the com velocity error
    MatrixXd H11 = kron_product(_Q, (MatrixXd(2,2) << 0, 0, 0, 1).finished());
    // Part of the cost associated with the zmp displacement with respect to the previous step
    MatrixXd H22 = _R;
    H22 += (VectorXd(_R.rows()) << _R.diagonal().tail(_R.rows() - 1), 0).finished().asDiagonal();
    H22.diagonal( 1) -= _R.diagonal().tail(_R.rows() - 1);
    H22.diagonal(-1) -= _R.diagonal().tail(_R.rows() - 1);

    MatrixXd H = MatrixXd::Zero(1 + H11.rows() + H22.rows(), 1 + H11.cols() + H22.cols());
    H.block(1, 1, H11.rows(), H11.cols()) = H11;
    H.block(1 + H11.rows(), 1 + H11.rows(), H22.rows(), H22.rows()) = H22;


    VectorXd xcom_dot_des_vec = xcom_dot_des * VectorXd::Ones(_n);

    double c0 = xcom_dot_des_vec.transpose() * _Q * xcom_dot_des_vec + _R(0,0) * std::pow(p_0, 2);

    VectorXd c1 = kron_product(- 2 * _Q * xcom_dot_des_vec, (VectorXd(2) << 0, 1).finished());

    VectorXd c2 = VectorXd::Zero(_n);
    c2(0) = - 2 * _R(0, 0) * p_0;

    VectorXd c(1 + c1.size() + c2.size());
    c << c0, c1, c2;


    /* ======================== Equality Constraints ======================== */

    // Enforces dynamic consistency with the linearized inverted pendulum model

    double g = 9.81;
    double omega = std::pow((g / _height_com), 0.5);

    MatrixXd A_t = compute_A_t(omega, Ts);
    MatrixXd b_t = compute_b_t(omega, Ts);

    MatrixXd I_off_diag = MatrixXd::Zero(_n, _n);
    I_off_diag.diagonal(-1).setOnes();

    MatrixXd A = MatrixXd::Zero(1 + 2*_n, 1 + 3*_n);
    A(0, 0) = 1;
    A.block(1, 1, 2*_n, 2*_n) = - MatrixXd::Identity(2*_n, 2*_n)
                                + kron_product(I_off_diag, A_t);
    A.bottomRightCorner(2*_n, _n) = kron_product(MatrixXd::Identity(_n, _n), (MatrixXd(2,1) << b_t(0), b_t(1)).finished());

    double t0 = Ts * (1. - _phi);
    MatrixXd A_t0 = compute_A_t(omega, t0);
    MatrixXd b_t0 = compute_b_t(omega, t0);

    VectorXd b = VectorXd::Zero(1 + 2*_n);
    b(0) = 1;
    b.segment(1, 2) = - A_t * (A_t0 * Xcom_0 + b_t0 * p_0);


    /* ======================= Inequality Constraints ======================= */
    
    // Limit the zmp displacement between two consecutive steps

    MatrixXd D = MatrixXd::Zero(2 * _n, 1 + 3*_n);
    D.topRightCorner(_n, _n) = MatrixXd::Identity(_n, _n) - I_off_diag;
    D.bottomRightCorner(_n, _n) = - D.topRightCorner(_n, _n);

    VectorXd f(2 * _n);
    f(0) = p_0 + _step_reachability;
    f.segment(1, _n-1).setConstant(_step_reachability);
    f(_n) = - p_0 + _step_reachability;
    f.tail(_n - 1).setConstant(_step_reachability);


    /* ======================= Quadprog QP Formulation ====================== */

    // min  1/2 x^T G x - a^T x
    // s.t. C^T x >= b
    // where the first meq are equality constraints and the remaining ones are inequality constraints

    double reg = 1e-9;
    H.diagonal() += reg * VectorXd::Ones(H.rows());

    MatrixXd C(A.rows() + D.rows(), A.cols());
    C.topRows(A.rows()) = A;
    C.bottomRows(D.rows()) = D;

    VectorXd d(b.size() + f.size());
    d.head(b.size()) = b;
    d.tail(f.size()) = f;

    VectorXd sol(1+3*_n);

    // std::cout << H << "\n" << std::endl;
    // std::cout << c << "\n" << std::endl;
    // std::cout << C << "\n" << std::endl;
    // std::cout << d << "\n\n" << std::endl;

    solve_quadprog(
        std::move(H),
        - c,
        - (C).transpose(),
        - d,
        sol,
        2*_n + 1
    );

    double p_i_star = sol(1 + 2 * _n);

    return p_i_star;
}


std::vector<Vector2d> MotionPlanner::compute_desired_footholds(const Vector2d& p_star)
{
    std::vector<Vector2d> p_swing_feet = {};

    for (const std::string& foot_name : _swing_feet_names) {
        if (foot_name == "LF") {
            p_swing_feet.emplace_back(
                p_star + _r * (Eigen::Vector2d() <<   std::cos(_theta_0 + _dtheta),   std::sin(_theta_0 + _dtheta)).finished()
            );    
        } else if (foot_name == "RF") {
            p_swing_feet.emplace_back(
                p_star + _r * (Eigen::Vector2d() <<   std::cos(_theta_0 - _dtheta), - std::sin(_theta_0 - _dtheta)).finished()
            );    
        } else if (foot_name == "LH") {
            p_swing_feet.emplace_back(
                p_star + _r * (Eigen::Vector2d() << - std::cos(_theta_0 - _dtheta),   std::sin(_theta_0 - _dtheta)).finished()
            );    
        } else if (foot_name == "RH") {
            p_swing_feet.emplace_back(
                p_star + _r * (Eigen::Vector2d() << - std::cos(_theta_0 + _dtheta), - std::sin(_theta_0 + _dtheta)).finished()
            );    
        }
    }

    _final_pos_swing_feet = p_swing_feet;

    return p_swing_feet;
}

std::tuple<VectorXd, VectorXd, VectorXd> MotionPlanner::compute_foot_trajectory(
    const std::vector<Vector2d>& pos_swing_feet
) {
    long n_swing_feet = pos_swing_feet.size();
    VectorXd r_s_des = VectorXd::Zero(3 * n_swing_feet);
    VectorXd r_s_dot_des = VectorXd::Zero(3 * n_swing_feet);
    VectorXd r_s_ddot_des = VectorXd::Zero(3 * n_swing_feet);

    for (int i = 0; i < 4; i++) {
        const auto it = std::find(_swing_feet_names.begin(), _swing_feet_names.end(), _all_feet_names[i]);

        if (it != _swing_feet_names.end()) {
            int j = it - _swing_feet_names.begin();

            auto init_pos = _init_pos_swing_feet[i];
            // std::cout << init_pos << "\n" << std::endl;
            Vector3d end_pos = {pos_swing_feet[j][0], pos_swing_feet[j][1], 0.0};
            // std::cout << end_pos << "\n\n" << std::endl;
            
            std::forward_as_tuple(
                r_s_des.segment(3*j, 3),
                r_s_dot_des.segment(3*j, 3),
                r_s_ddot_des.segment(3*j, 3)
            ) = _interpolator.interpolate(init_pos, end_pos, _phi);
        }
    }

    return {r_s_ddot_des, r_s_dot_des, r_s_des};
}


void MotionPlanner::switch_swing_feet(const std::vector<Vector2d>& p_swing_feet)
{
    auto old_swing_feet_names = _swing_feet_names;
    _swing_feet_names = {};
    _swing_feet_names.reserve(2);

    int count = 0;

    for (int i = 0; i < 4; i++) {        
        auto it = std::find(old_swing_feet_names.begin(), old_swing_feet_names.end(), _all_feet_names[i]);

        if (it == old_swing_feet_names.end()) {
            _swing_feet_names.push_back(_all_feet_names[i]);
        } else {
            _init_pos_swing_feet[i] << p_swing_feet[count][0],
                                       p_swing_feet[count][1],
                                       0.;

            count++;
        }
    }
}


std::tuple<Vector3d, Vector3d, Vector3d> MotionPlanner::get_des_base_pose(
    const Vector2d& Xcom_0, const Vector2d& Ycom_0,
    double px_0, double py_0
) {
    double g = 9.81;
    double omega = std::pow((g / _height_com), 0.5);
    
    Vector2d Xcom = compute_A_t(omega, _dt) * Xcom_0 + compute_b_t(omega, _dt) * px_0;
    Vector2d Ycom = compute_A_t(omega, _dt) * Ycom_0 + compute_b_t(omega, _dt) * py_0;

    Vector3d r_b_des = {Xcom[0], Ycom[0], _height_com};
    Vector3d r_b_dot_des = {Xcom[1], Ycom[1], 0};
    Vector3d r_b_ddot_des = {Xcom_0[0] - px_0, Ycom_0[0] - py_0, 0};
    r_b_ddot_des *= g / _height_com;

    return {r_b_ddot_des, r_b_dot_des, r_b_des};
}


/* =================================== Mpc ================================== */

generalized_pose::GeneralizedPoseStruct MotionPlanner::mpc(
    const Vector3d& p_com, const Vector3d& v_com, const Vector3d& a_com,
    const Vector2d& vel_cmd, double yaw_rate_cmd
) {
    // Initial CoM position
    Vector2d Xcom_0 = {p_com[0], v_com[0]};
    Vector2d Ycom_0 = {p_com[1], v_com[1]};

    // Initial ZMP position (from the dynamics of a linear inverted pendulum).
    double g = 9.81;
    double px_0 = p_com[0] - a_com[0] * _height_com / g;
    double py_0 = p_com[1] - a_com[1] * _height_com / g;
    
    double ph_0 =   px_0 * std::cos(_dtheta) + py_0 * std::sin(_dtheta);
    double pl_0 = - px_0 * std::sin(_dtheta) + py_0 * std::cos(_dtheta);
    
    Vector2d Hcom_0 =   Xcom_0 * std::cos(_dtheta) + Ycom_0 * std::sin(_dtheta);
    Vector2d Lcom_0 = - Xcom_0 * std::sin(_dtheta) + Ycom_0 * std::cos(_dtheta);

    // Compute the x and y coordinates of the ZMP.
    double xcom_dot_des = vel_cmd[0];
    double ycom_dot_des = vel_cmd[1];
    double p_h_star = mpc_qp(Hcom_0, ph_0, xcom_dot_des);
    double p_l_star = mpc_qp(Lcom_0, pl_0, ycom_dot_des);
    
    double p_x_star = p_h_star * std::cos(_dtheta) - p_l_star * std::sin(_dtheta);
    double p_y_star = p_h_star * std::sin(_dtheta) + p_l_star * std::cos(_dtheta);

    // Optimal ZMP position.
    Vector2d p_star = {p_x_star, p_y_star};

    // std::cout << p_star << "\n" << std::endl;

    // Compute the desired footholds of the feet currently in swing phase.
    _dtheta += yaw_rate_cmd * _dt;
    std::vector<Vector2d> p_swing_feet = compute_desired_footholds(p_star);

    // Compute the feet trajectories.
    auto [r_s_ddot_des, r_s_dot_des, r_s_des] = compute_foot_trajectory(p_swing_feet);

    // Compute the base linear trajectory.
    auto [r_b_ddot_des, r_b_dot_des, r_b_des] = get_des_base_pose(Xcom_0, Ycom_0, px_0, py_0);

    // Compute the base angular trajectory.
    Vector3d omega_des = {0.0, 0.0, yaw_rate_cmd};
    generalized_pose::Quaternion q_des(0.0, 0.0, std::sin(_dtheta/2), std::cos(_dtheta/2));

    // Compute the list of feet in contact phase.
    std::vector<std::string> contact_feet_names = {};
    for (const auto& foot_name : _all_feet_names) {
        if (std::find(_swing_feet_names.begin(), _swing_feet_names.end(), foot_name) == _swing_feet_names.end()) {
            contact_feet_names.push_back(foot_name);
        }
    }

    // Update the normalized phase. When it becomes >=1, switch the contact feet and reset it to zero.
    _phi += _dt / _interpolator.get_step_duration();
    if (_phi >= 1) {
        _phi = 0;
        switch_swing_feet(p_swing_feet);
    }
        
    generalized_pose::GeneralizedPoseStruct des_gen_pose(
        r_b_ddot_des,
        r_b_dot_des,
        r_b_des,
        omega_des,
        q_des,
        r_s_ddot_des,
        r_s_dot_des,
        r_s_des,
        contact_feet_names
    );
    
    return des_gen_pose;
}


/* =============================== Check_stop =============================== */

bool MotionPlanner::check_stop(
    const Vector2d& vel_cmd, double yaw_rate_cmd
) {
    double threshold = 0.05;

    if (_phi == 0.
        && _fixed_steps >= _max_fixed_steps
        && vel_cmd.norm() < threshold
        && std::abs(yaw_rate_cmd) < threshold) {
        // Stop the robot movements.

        return true;
    } else if (_phi == 0.
               && _fixed_steps < _max_fixed_steps
               && vel_cmd.norm() < threshold
               && std::abs(yaw_rate_cmd) < threshold) {
        // Do not stop the robot, but increase the number of steps during which the commanded twist was zero.

        _fixed_steps += 1;
    } else if (vel_cmd.norm() > threshold || std::abs(yaw_rate_cmd) > threshold) {
        _fixed_steps = 0.;
    }

    return false;
}


/* ================================= Update ================================= */

generalized_pose::GeneralizedPoseStruct MotionPlanner::update(
    const Ref<Vector3d>& p_com, const Ref<Vector3d>& v_com, const Ref<Vector3d>& a_com,
    const Ref<Vector2d>& vel_cmd, double yaw_rate_cmd
) {
    bool stop_flag = check_stop(vel_cmd, yaw_rate_cmd);

    if (stop_flag) {
        double pos_x = 0;
        double pos_y = 0;

        for (const auto& elem : _init_pos_swing_feet) {
            pos_x += elem[0] / _init_pos_swing_feet.size();
            pos_y += elem[1] / _init_pos_swing_feet.size();
        }

        return  generalized_pose::GeneralizedPoseStruct(
            generalized_pose::Vector3(pos_x, pos_y, _height_com),
            generalized_pose::Quaternion(0.0, 0.0, std::sin(_dtheta / 2), std::cos(_dtheta / 2))
        );
    } else {        
        return mpc(
            p_com, v_com, a_com,
            vel_cmd, yaw_rate_cmd
        );
    }
}


/* ==================== Compute_trajectory_sample_points ==================== */

std::vector<std::vector<Vector3d>> MotionPlanner::compute_trajectory_sample_points()
{
    std::vector<std::vector<Vector3d>> swing_feet_sampled_trajectories;
    swing_feet_sampled_trajectories.resize(2);
    for (std::vector<Vector3d>& trajectory : swing_feet_sampled_trajectories) {
        trajectory.resize(_n_sample_points);
    }
    
    // Indices of the swing feet
    std::vector<int> feet_ids = {};
    for (int i = 0; i < 4; i++) {
        if (std::find(_swing_feet_names.begin(), _swing_feet_names.end(), _all_feet_names[i]) != _swing_feet_names.end()) {
            feet_ids.push_back(i);
        }
    }

    for (int i = 0; i < _n_sample_points; i++) {
        double phi = static_cast<double>(i) / (_n_sample_points - 1);

        for (int j : feet_ids) {
            auto init_pos = _init_pos_swing_feet[j];
            auto end_pos_xy = _final_pos_swing_feet[
                std::find(_swing_feet_names.begin(), _swing_feet_names.end(), _all_feet_names[j]) - _swing_feet_names.begin()
            ];

            Vector3d end_pos = {end_pos_xy[0], end_pos_xy[1], 0.};

            int jj = std::find(feet_ids.begin(), feet_ids.end(), j) - feet_ids.begin();
            swing_feet_sampled_trajectories[jj][i] = std::get<0>(
                _interpolator.interpolate(
                    init_pos, end_pos, phi
            ));
        }
    }

    return swing_feet_sampled_trajectories;
}

}
