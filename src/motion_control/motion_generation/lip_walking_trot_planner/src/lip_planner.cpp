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
    dtheta_ = yaw;

    init_pos_swing_feet_.resize(4);

    // Initialize the _init_pos_swing_feet with the positions of the feet relative to the base in base frame.
    // LF
    init_pos_swing_feet_[0] <<   r_ * std::cos(theta_0_ + dtheta_),
                                 r_ * std::sin(theta_0_ + dtheta_),
                                 0.0;

    // RF
    init_pos_swing_feet_[1] <<   r_ * std::cos(theta_0_ - dtheta_),
                               - r_ * std::sin(theta_0_ - dtheta_),
                                 0.0;

    // LH
    init_pos_swing_feet_[2] << - r_ * std::cos(theta_0_ - dtheta_),
                                 r_ * std::sin(theta_0_ - dtheta_),
                                 0.0;

    // RH
    init_pos_swing_feet_[3] << - r_ * std::cos(theta_0_ + dtheta_),
                               - r_ * std::sin(theta_0_ + dtheta_),
                                 0.0;

    // Shift the quantity by the initial position of the base.
    for (auto& foot_pos : init_pos_swing_feet_) {
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

    double Ts = interpolator_.get_step_duration();


    /* =========================== QP Formulation =========================== */

    // min_xi   1/2 xi^T H xi + c^T xi
    // s.t.: A xi - b  = 0
    //       D xi - f <= 0


    /* =========================== Cost Definition ========================== */

    // cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]

    // Part of the cost associated with the com velocity error
    MatrixXd H11 = kron_product(Q_, (MatrixXd(2,2) << 0, 0, 0, 1).finished());
    // Part of the cost associated with the zmp displacement with respect to the previous step
    MatrixXd H22 = R_;
    H22 += (VectorXd(R_.rows()) << R_.diagonal().tail(R_.rows() - 1), 0).finished().asDiagonal();
    H22.diagonal( 1) -= R_.diagonal().tail(R_.rows() - 1);
    H22.diagonal(-1) -= R_.diagonal().tail(R_.rows() - 1);

    MatrixXd H = MatrixXd::Zero(1 + H11.rows() + H22.rows(), 1 + H11.cols() + H22.cols());
    H.block(1, 1, H11.rows(), H11.cols()) = H11;
    H.block(1 + H11.rows(), 1 + H11.rows(), H22.rows(), H22.rows()) = H22;


    VectorXd xcom_dot_des_vec = xcom_dot_des * VectorXd::Ones(n_);

    double c0 = xcom_dot_des_vec.transpose() * Q_ * xcom_dot_des_vec + R_(0,0) * std::pow(p_0, 2);

    VectorXd c1 = kron_product(- 2 * Q_ * xcom_dot_des_vec, (VectorXd(2) << 0, 1).finished());

    VectorXd c2 = VectorXd::Zero(n_);
    c2(0) = - 2 * R_(0, 0) * p_0;

    VectorXd c(1 + c1.size() + c2.size());
    c << c0, c1, c2;


    /* ======================== Equality Constraints ======================== */

    // Enforces dynamic consistency with the linearized inverted pendulum model

    double g = 9.81;
    double omega = std::pow((g / height_com_), 0.5);

    MatrixXd A_t = compute_A_t(omega, Ts);
    MatrixXd b_t = compute_b_t(omega, Ts);

    MatrixXd I_off_diag = MatrixXd::Zero(n_, n_);
    I_off_diag.diagonal(-1).setOnes();

    MatrixXd A = MatrixXd::Zero(1 + 2*n_, 1 + 3*n_);
    A(0, 0) = 1;
    A.block(1, 1, 2*n_, 2*n_) = - MatrixXd::Identity(2*n_, 2*n_)
                                + kron_product(I_off_diag, A_t);
    A.bottomRightCorner(2*n_, n_) = kron_product(MatrixXd::Identity(n_, n_), (MatrixXd(2,1) << b_t(0), b_t(1)).finished());

    double t0 = Ts * (1. - phi_);
    MatrixXd A_t0 = compute_A_t(omega, t0);
    MatrixXd b_t0 = compute_b_t(omega, t0);

    VectorXd b = VectorXd::Zero(1 + 2*n_);
    b(0) = 1;
    b.segment(1, 2) = - A_t * (A_t0 * Xcom_0 + b_t0 * p_0);


    /* ======================= Inequality Constraints ======================= */
    
    // Limit the zmp displacement between two consecutive steps

    MatrixXd D = MatrixXd::Zero(2 * n_, 1 + 3*n_);
    D.topRightCorner(n_, n_) = MatrixXd::Identity(n_, n_) - I_off_diag;
    D.bottomRightCorner(n_, n_) = - D.topRightCorner(n_, n_);

    VectorXd f(2 * n_);
    f(0) = p_0 + step_reachability_;
    f.segment(1, n_-1).setConstant(step_reachability_);
    f(n_) = - p_0 + step_reachability_;
    f.tail(n_ - 1).setConstant(step_reachability_);


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

    VectorXd sol(1+3*n_);

    solve_quadprog(
        std::move(H),
        - c,
        - (C).transpose(),
        - d,
        sol,
        2*n_ + 1
    );

    double p_i_star = sol(1 + 2 * n_);

    return p_i_star;
}


void MotionPlanner::compute_desired_footholds(const Vector2d& p_star)
{
    final_pos_swing_feet_ = {};

    for (const std::string& foot_name : swing_feet_names_) {
        if (foot_name == "LF") {
            final_pos_swing_feet_.emplace_back(
                p_star + r_ * (Eigen::Vector2d() <<   std::cos(theta_0_ + dtheta_),   std::sin(theta_0_ + dtheta_)).finished()
            );    
        } else if (foot_name == "RF") {
            final_pos_swing_feet_.emplace_back(
                p_star + r_ * (Eigen::Vector2d() <<   std::cos(theta_0_ - dtheta_), - std::sin(theta_0_ - dtheta_)).finished()
            );    
        } else if (foot_name == "LH") {
            final_pos_swing_feet_.emplace_back(
                p_star + r_ * (Eigen::Vector2d() << - std::cos(theta_0_ - dtheta_),   std::sin(theta_0_ - dtheta_)).finished()
            );    
        } else if (foot_name == "RH") {
            final_pos_swing_feet_.emplace_back(
                p_star + r_ * (Eigen::Vector2d() << - std::cos(theta_0_ + dtheta_), - std::sin(theta_0_ + dtheta_)).finished()
            );    
        }
    }
}

std::tuple<VectorXd, VectorXd, VectorXd> MotionPlanner::compute_swing_feet_trajectories(
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
)
{
    bool good_feet_positions = false;
    for (const auto& foot_position : feet_positions) {
        if (foot_position.norm() > 1e-6) {
            good_feet_positions = true;
            break;
        }
    }

    int n_swing_feet = final_pos_swing_feet_.size();
    VectorXd r_s_des = VectorXd::Zero(3 * n_swing_feet);
    VectorXd r_s_dot_des = VectorXd::Zero(3 * n_swing_feet);
    VectorXd r_s_ddot_des = VectorXd::Zero(3 * n_swing_feet);

    for (int i = 0; i < 4; i++) {
        const auto it = std::find(swing_feet_names_.begin(), swing_feet_names_.end(), all_feet_names_[i]);

        if (it != swing_feet_names_.end()) {
            int j = it - swing_feet_names_.begin();

            Vector3d end_pos = {final_pos_swing_feet_[j][0], final_pos_swing_feet_[j][1], 0.0};
            
            if (good_feet_positions) {
                std::forward_as_tuple(
                    r_s_des.segment(3*j, 3),
                    r_s_dot_des.segment(3*j, 3),
                    r_s_ddot_des.segment(3*j, 3)
                ) = interpolator_.interpolate(
                    feet_positions[i], feet_velocities[i],
                    end_pos, {0,0,0},
                    phi_, dt_);
            } else {
                auto init_pos = init_pos_swing_feet_[i];

                std::forward_as_tuple(
                    r_s_des.segment(3*j, 3),
                    r_s_dot_des.segment(3*j, 3),
                    r_s_ddot_des.segment(3*j, 3)
                ) = interpolator_.interpolate(init_pos, end_pos, phi_);
            }
        }
    }

    return {r_s_ddot_des, r_s_dot_des, r_s_des};
}


void MotionPlanner::switch_swing_feet()
{
    auto old_swing_feet_names = swing_feet_names_;
    swing_feet_names_ = {};
    swing_feet_names_.reserve(2);

    int count = 0;

    for (int i = 0; i < 4; i++) {        
        auto it = std::find(old_swing_feet_names.begin(), old_swing_feet_names.end(), all_feet_names_[i]);

        if (it == old_swing_feet_names.end()) {
            swing_feet_names_.push_back(all_feet_names_[i]);
        } else {
            init_pos_swing_feet_[i] << final_pos_swing_feet_[count][0],
                                       final_pos_swing_feet_[count][1],
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
    double omega = std::pow((g / height_com_), 0.5);
    
    Vector2d Xcom = compute_A_t(omega, dt_) * Xcom_0 + compute_b_t(omega, dt_) * px_0;
    Vector2d Ycom = compute_A_t(omega, dt_) * Ycom_0 + compute_b_t(omega, dt_) * py_0;

    Vector3d r_b_des = {Xcom[0], Ycom[0], height_com_};
    Vector3d r_b_dot_des = {Xcom[1], Ycom[1], 0};
    Vector3d r_b_ddot_des = {Xcom_0[0] - px_0, Ycom_0[0] - py_0, 0};
    r_b_ddot_des *= g / height_com_;

    return {r_b_ddot_des, r_b_dot_des, r_b_des};
}


/* =================================== Mpc ================================== */

generalized_pose::GeneralizedPoseStruct MotionPlanner::mpc(
    const Vector3d& p_com, const Vector3d& v_com, const Vector3d& a_com,
    const Vector2d& vel_cmd, double yaw_rate_cmd,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
) {
    // Initial CoM position
    Vector2d Xcom_0 = {p_com[0], v_com[0]};
    Vector2d Ycom_0 = {p_com[1], v_com[1]};

    // Initial ZMP position (from the dynamics of a linear inverted pendulum).
    double g = 9.81;
    double px_0 = p_com[0] - a_com[0] * height_com_ / g;
    double py_0 = p_com[1] - a_com[1] * height_com_ / g;
    
    double ph_0 =   px_0 * std::cos(dtheta_) + py_0 * std::sin(dtheta_);
    double pl_0 = - px_0 * std::sin(dtheta_) + py_0 * std::cos(dtheta_);
    
    Vector2d Hcom_0 =   Xcom_0 * std::cos(dtheta_) + Ycom_0 * std::sin(dtheta_);
    Vector2d Lcom_0 = - Xcom_0 * std::sin(dtheta_) + Ycom_0 * std::cos(dtheta_);

    // Compute the x and y coordinates of the ZMP.
    double xcom_dot_des = vel_cmd[0];
    double ycom_dot_des = vel_cmd[1];
    double p_h_star = mpc_qp(Hcom_0, ph_0, xcom_dot_des);
    double p_l_star = mpc_qp(Lcom_0, pl_0, ycom_dot_des);
    
    double p_x_star = p_h_star * std::cos(dtheta_) - p_l_star * std::sin(dtheta_);
    double p_y_star = p_h_star * std::sin(dtheta_) + p_l_star * std::cos(dtheta_);

    // Optimal ZMP position.
    Vector2d p_star = {p_x_star, p_y_star};

    // Compute the desired footholds of the feet currently in swing phase.
    dtheta_ += yaw_rate_cmd * dt_;
    compute_desired_footholds(p_star);

    // Compute the feet trajectories.
    auto [r_s_ddot_des, r_s_dot_des, r_s_des] = compute_swing_feet_trajectories(
        feet_positions, feet_velocities
    );

    // Compute the base linear trajectory.
    auto [r_b_ddot_des, r_b_dot_des, r_b_des] = get_des_base_pose(Xcom_0, Ycom_0, px_0, py_0);

    // Compute the base angular trajectory.
    Vector3d omega_des = {0.0, 0.0, yaw_rate_cmd};
    generalized_pose::Quaternion q_des(0.0, 0.0, std::sin(dtheta_/2), std::cos(dtheta_/2));

    // Compute the list of feet in contact phase.
    std::vector<std::string> contact_feet_names = {};
    for (const auto& foot_name : all_feet_names_) {
        if (std::find(swing_feet_names_.begin(), swing_feet_names_.end(), foot_name) == swing_feet_names_.end()) {
            contact_feet_names.push_back(foot_name);
        }
    }

    // Update the normalized phase. When it becomes >=1, switch the contact feet and reset it to zero.
    phi_ += dt_ / interpolator_.get_step_duration();
    if (phi_ >= 1) {
        phi_ = 0;
        switch_swing_feet();
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

    if (phi_ == 0.
        && fixed_steps_ >= max_fixed_steps_
        && vel_cmd.norm() < threshold
        && std::abs(yaw_rate_cmd) < threshold) {
        // Stop the robot movements.

        return true;
    } else if (phi_ == 0.
               && fixed_steps_ < max_fixed_steps_
               && vel_cmd.norm() < threshold
               && std::abs(yaw_rate_cmd) < threshold) {
        // Do not stop the robot, but increase the number of steps during which the commanded twist was zero.

        fixed_steps_ += 1;
    } else if (vel_cmd.norm() > threshold || std::abs(yaw_rate_cmd) > threshold) {
        fixed_steps_ = 0.;
    }

    return false;
}


/* ================================= Update ================================= */

generalized_pose::GeneralizedPoseStruct MotionPlanner::update(
    const Ref<Vector3d>& p_com, const Ref<Vector3d>& v_com, const Ref<Vector3d>& a_com,
    const Ref<Vector2d>& vel_cmd, double yaw_rate_cmd,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
) {
    stop_flag_ = check_stop(vel_cmd, yaw_rate_cmd);

    if (stop_flag_) {
        double pos_x = 0;
        double pos_y = 0;

        for (const auto& elem : init_pos_swing_feet_) {
            pos_x += elem[0] / init_pos_swing_feet_.size();
            pos_y += elem[1] / init_pos_swing_feet_.size();
        }

        return  generalized_pose::GeneralizedPoseStruct(
            generalized_pose::Vector3(pos_x, pos_y, height_com_),
            generalized_pose::Quaternion(0.0, 0.0, std::sin(dtheta_ / 2), std::cos(dtheta_ / 2))
        );
    } else {        
        return mpc(
            p_com, v_com, a_com,
            vel_cmd, yaw_rate_cmd,
            feet_positions, feet_velocities
        );
    }
}


/* ==================== Compute_trajectory_sample_points ==================== */

std::vector<std::vector<Vector3d>> MotionPlanner::compute_trajectory_sample_points() const
{
    if (stop_flag_) {
        return {};
    }

    std::vector<std::vector<Vector3d>> swing_feet_sampled_trajectories;
    swing_feet_sampled_trajectories.resize(2);
    for (std::vector<Vector3d>& trajectory : swing_feet_sampled_trajectories) {
        trajectory.resize(n_sample_points_);
    }
    
    // Indices of the swing feet
    std::vector<int> feet_ids = {};
    for (int i = 0; i < 4; i++) {
        if (std::find(swing_feet_names_.begin(), swing_feet_names_.end(), all_feet_names_[i]) != swing_feet_names_.end()) {
            feet_ids.push_back(i);
        }
    }

    for (int i = 0; i < n_sample_points_; i++) {
        double phi = static_cast<double>(i) / (n_sample_points_ - 1);

        for (int j : feet_ids) {
            auto init_pos = init_pos_swing_feet_[j];
            auto end_pos_xy = final_pos_swing_feet_[
                std::find(swing_feet_names_.begin(), swing_feet_names_.end(), all_feet_names_[j]) - swing_feet_names_.begin()
            ];

            Vector3d end_pos = {end_pos_xy[0], end_pos_xy[1], 0.};

            int jj = std::find(feet_ids.begin(), feet_ids.end(), j) - feet_ids.begin();
            swing_feet_sampled_trajectories[jj][i] = std::get<0>(
                interpolator_.interpolate(
                    init_pos, end_pos, phi
            ));
        }
    }

    return swing_feet_sampled_trajectories;
}

}
