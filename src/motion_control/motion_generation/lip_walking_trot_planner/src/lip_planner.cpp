#include "lip_walking_trot_planner/lip_planner.hpp"

#include "lip_walking_trot_planner/quaternion_math.hpp"

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


/* =============================== Constructor ============================== */

MotionPlanner::MotionPlanner()
{
    update_initial_conditions();
}


/* ======================== Update_initial_conditions ======================= */

void MotionPlanner::update_initial_conditions(
    const Ref<Vector3d>& init_pos,
    double init_yaw,
    const std::vector<Vector3d>& feet_positions)
{
    yaw_ = init_yaw;

    init_pos_swing_feet_.resize(4);

    if (feet_positions.size() != 4) {
        // The feet_positions has the wrong dimension, do not use it to initialize the swing feet positions.

        // Initialize the _init_pos_swing_feet with the positions of the feet relative to the base in base frame.
        // LF
        init_pos_swing_feet_[0] << r_ * std::cos(theta_0_ + init_yaw),
                                   r_ * std::sin(theta_0_ + init_yaw),
                                   0.0;

        // RF
        init_pos_swing_feet_[1] <<   r_ * std::cos(theta_0_ - init_yaw),
                                   - r_ * std::sin(theta_0_ - init_yaw),
                                     0.0;

        // LH
        init_pos_swing_feet_[2] << - r_ * std::cos(theta_0_ - init_yaw),
                                     r_ * std::sin(theta_0_ - init_yaw),
                                     0.0;

        // RH
        init_pos_swing_feet_[3] << - r_ * std::cos(theta_0_ + init_yaw),
                                   - r_ * std::sin(theta_0_ + init_yaw),
                                     0.0;

        // Shift the quantity by the initial position of the base.
        for (auto& foot_pos : init_pos_swing_feet_) {
            foot_pos.head(2) += init_pos.head(2);
        }
    } else {
        // Initialize the swing feet positions directly with the measured position of the feet.
        for (int i = 0; i < 4; i++) {
            init_pos_swing_feet_[i] << feet_positions[i][0],
                                       feet_positions[i][1],
                                       feet_positions[i][2];
        }
    }
}


/* ================================= Update ================================= */

std::vector<generalized_pose::GeneralizedPoseStruct> MotionPlanner::update(
    const Ref<Vector3d>& pos_com, const Ref<Vector3d>& vel_com, const Ref<Vector3d>& acc_com,
    const Ref<Vector2d>& vel_cmd, double yaw_rate_cmd,
    const Vector3d& plane_coeffs,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities,
    double time
) {
    stop_flag_ = check_stop(vel_cmd, yaw_rate_cmd);

    if (stop_flag_) {
        return stand_still(plane_coeffs);
    }

    return mpc(
        pos_com, vel_com, acc_com,
        vel_cmd, yaw_rate_cmd,
        plane_coeffs,
        feet_positions, feet_velocities,
        time
    );
}


/* =============================== Stand_still ============================== */

std::vector<generalized_pose::GeneralizedPoseStruct> MotionPlanner::stand_still(
    const Vector3d& plane_coeffs
) const {
    // The commanded position along the x and y coordinates is the mean of the positions of the contact feet.
    double pos_x = 0;
    double pos_y = 0;

    for (const auto& elem : init_pos_swing_feet_) {
        pos_x += elem[0] / static_cast<double>(init_pos_swing_feet_.size());
        pos_y += elem[1] / static_cast<double>(init_pos_swing_feet_.size());
    }

    // Align the base with the estimated terrain plane.
    double roll = std::atan(plane_coeffs[1]);
    double pitch = - std::atan(plane_coeffs[0]);

    Quaterniond quat = compute_quaternion_from_euler_angles(
        roll,
        pitch,
        yaw_
    );

    return {{
        generalized_pose::Vector3(
            pos_x + height_com_ * std::sin(pitch),
            pos_y - height_com_ * std::sin(roll),
            height_com_ * std::cos(roll) * std::cos(pitch)
        ),
        generalized_pose::Quaternion(
            quat.x(), quat.y(), quat.z(), quat.w()
        )
    }};
}


/* =================================== Mpc ================================== */

std::vector<generalized_pose::GeneralizedPoseStruct> MotionPlanner::mpc(
    const Vector3d& pos_com, const Vector3d& vel_com, const Vector3d& acc_com,
    const Vector2d& vel_cmd, double yaw_rate_cmd,
    const Vector3d& plane_coeffs,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities,
    double time
) {
    // Initial CoM state (position and velocity in the two directions).
    Vector2d X_com_0 = {pos_com[0], vel_com[0]};
    Vector2d Y_com_0 = {pos_com[1], vel_com[1]};

    // Initial ZMP position (from the dynamics of a linear inverted pendulum).
    double g = 9.81;
    double x_zmp_0 = pos_com[0] - acc_com[0] * height_com_ / g;
    double y_zmp_0 = pos_com[1] - acc_com[1] * height_com_ / g;
    
    auto pos_zmp_star = downsampled_solve_qps(
        X_com_0, Y_com_0,
        x_zmp_0, y_zmp_0,
        vel_cmd, yaw_rate_cmd
    );

    auto des_gen_poses = compute_gen_poses(
        X_com_0, Y_com_0,
        x_zmp_0, y_zmp_0,
        pos_zmp_star,
        yaw_rate_cmd,
        plane_coeffs,
        feet_positions, feet_velocities
    );

    /* ===================== Update Some Internal States ==================== */

    yaw_ += yaw_rate_cmd * dt_;

    // Update the normalized phase. When it becomes >=1, switch the contact feet and reset phi to zero.
    if (time > last_time) {
        phi_ += (time - last_time) / interpolator_.get_step_duration();
        last_time = time;
    } else {
        phi_ += dt_ / interpolator_.get_step_duration();
    }
    if (phi_ >= 1) {
        phi_ = 0;
        switch_swing_feet(feet_positions);
    }
    
    return des_gen_poses;
}


/* ========================== Downsampled_solve_qps ========================= */

std::vector<Vector2d> MotionPlanner::downsampled_solve_qps(
    const Vector2d& X_com_0, const Vector2d& Y_com_0,
    double x_zmp_0, double y_zmp_0,
    const Vector2d& vel_cmd, double yaw_rate_cmd
) {
    if (phi_ == 0) {
        counter_ = 0;
    }
    
    if (counter_ == 0) {
        pos_zmp_star_ = solve_qps(
            X_com_0, Y_com_0,
            x_zmp_0, y_zmp_0,
            vel_cmd, yaw_rate_cmd
        );
    }

    counter_++;
    counter_ = counter_ % this->downsample_factor_;

    // We want to copy this, since it will be modified by compute_gen_poses but we may need to reuse it at the next iteration.
    return pos_zmp_star_;
}


/* ================================ Solve_qps =============================== */

std::vector<Vector2d> MotionPlanner::solve_qps(
    const Vector2d& X_com_0, const Vector2d& Y_com_0,
    double x_zmp_0, double y_zmp_0,
    const Vector2d& vel_cmd, double yaw_rate_cmd
) const {
    // Compute the heading and lateral state of the com.
    Vector2d H_com_0 =   X_com_0 * std::cos(yaw_) + Y_com_0 * std::sin(yaw_);
    Vector2d L_com_0 = - X_com_0 * std::sin(yaw_) + Y_com_0 * std::cos(yaw_);
    
    // Compute the heading and lateral coordinates of the zmp.
    double h_zmp_0 =   x_zmp_0 * std::cos(yaw_) + y_zmp_0 * std::sin(yaw_);
    double l_zmp_0 = - x_zmp_0 * std::sin(yaw_) + y_zmp_0 * std::cos(yaw_);

    // Compute the heading and lateral coordinates of the ZMP.
    double h_com_dot_des = vel_cmd[0];
    double l_com_dot_des = vel_cmd[1];
    std::vector<double> h_zmp_star = mpc_qp(H_com_0, h_zmp_0, h_com_dot_des);
    std::vector<double> l_zmp_star = mpc_qp(L_com_0, l_zmp_0, l_com_dot_des);
    
    // Optimal zmp position computed by the mpc for the next n_steps_prediction_horizon steps.
    std::vector<Vector2d> pos_zmp_star(n_steps_prediction_horizon);

    for (int i = 0; i < n_steps_prediction_horizon; i++) {
        double yaw = yaw_ + i * interpolator_.get_step_duration() * yaw_rate_cmd;

        // Optimal ZMP position.
        pos_zmp_star[i] = {
            h_zmp_star[i] * std::cos(yaw) - l_zmp_star[i] * std::sin(yaw),  // x
            h_zmp_star[i] * std::sin(yaw) + l_zmp_star[i] * std::cos(yaw)   // y
        };
    }

    return pos_zmp_star;
}


/* ================================= Mpc_qp ================================= */

std::vector<double> MotionPlanner::mpc_qp(
    const Vector2d& X_com_0, double x_zmp_0,
    double x_com_dot_des
) const {
    /* ================================ State =============================== */
        
    // xi = [
    //     1                             extra state equal to one
    //     [xcom_i, x_com_dot_i].T       position and velocity coordinate (only one coordinate)
    //     px                            ZMP coordinate
    // ]

    const double Ts = interpolator_.get_step_duration();
    const int n = n_steps_prediction_horizon;


    /* =========================== QP Formulation =========================== */

    // min_xi   1/2 xi^T H xi + c^T xi
    // s.t.: A xi - b  = 0
    //       D xi - f <= 0


    /* =========================== Cost Definition ========================== */

    // cost = sum 1/2 [ (xcom_dot - xcom_dot_des)^T Q (xcom_dot - xcom_dot_des) + (px - px_m)^T R (px - px_m) ]

    MatrixXd H = MatrixXd::Zero(1 + 2*n + n, 1 + 2*n + n);
    // Part of the cost associated with the com velocity error.
    H.block(1, 1, 2*n, 2*n) = kron_product(Q_, (MatrixXd(2,2) << 0, 0, 0, 1).finished());;
    // Part of the cost associated with the zmp displacement with respect to the previous step.
    H.bottomRightCorner(n, n) = R_;
    H.bottomRightCorner(n, n) += (VectorXd(R_.rows()) << R_.diagonal().tail(R_.rows() - 1), 0).finished().asDiagonal();
    H.bottomRightCorner(n, n).diagonal( 1) -= R_.diagonal().tail(R_.rows() - 1);
    H.bottomRightCorner(n, n).diagonal(-1) -= R_.diagonal().tail(R_.rows() - 1);


    VectorXd x_com_dot_des_vec = x_com_dot_des * VectorXd::Ones(n);

    VectorXd c(1 + 2*n + n);
    c(0) = x_com_dot_des_vec.transpose() * Q_ * x_com_dot_des_vec + R_(0,0) * std::pow(x_zmp_0, 2);
    c.segment(1, 2*n) = kron_product(- 2 * Q_ * x_com_dot_des_vec, (VectorXd(2) << 0, 1).finished());
    c(1 + 2*n) = - 2 * R_(0, 0) * x_zmp_0;
    c.tail(n-1).setZero();


    /* ======================== Equality Constraints ======================== */

    // Enforces dynamic consistency with the linearized inverted pendulum model

    double g = 9.81;
    double omega = std::pow((g / height_com_), 0.5);

    Matrix2d A_t = compute_A_t(omega, Ts);
    Vector2d b_t = compute_b_t(omega, Ts);

    MatrixXd I_off_diag = MatrixXd::Zero(n, n);
    I_off_diag.diagonal(-1).setOnes();

    MatrixXd A = MatrixXd::Zero(1 + 2*n, 1 + 3*n);
    A(0, 0) = 1;
    A.block(1, 1, 2*n, 2*n) = - MatrixXd::Identity(2*n, 2*n)
                              + kron_product(I_off_diag, A_t);
    A.bottomRightCorner(2*n, n) = kron_product(MatrixXd::Identity(n, n), (MatrixXd(2,1) << b_t(0), b_t(1)).finished());

    double t_0 = Ts * (1. - phi_);
    Matrix2d A_t0 = compute_A_t(omega, t_0);
    Vector2d b_t0 = compute_b_t(omega, t_0);

    VectorXd b = VectorXd::Zero(1 + 2*n);
    b(0) = 1;
    b.segment(1, 2) = - A_t * (A_t0 * X_com_0 + b_t0 * x_zmp_0);


    /* ======================= Inequality Constraints ======================= */
    
    // Limit the zmp displacement between two consecutive steps

    MatrixXd D = MatrixXd::Zero(2 * n, 1 + 3*n);
    D.topRightCorner(n, n) = MatrixXd::Identity(n, n) - I_off_diag;
    D.bottomRightCorner(n, n) = - D.topRightCorner(n, n);

    VectorXd f(2 * n);
    f(0) = x_zmp_0 + step_reachability_;
    f.segment(1, n-1).setConstant(step_reachability_);
    f(n) = - x_zmp_0 + step_reachability_;
    f.tail(n - 1).setConstant(step_reachability_);


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

    VectorXd sol(1+3*n);

    solve_quadprog(
        std::move(H),
        - c,
        - (C).transpose(),
        - d,
        sol,
        2*n + 1
    );

    std::vector<double> x_zmp_star(
        sol.segment(1 + 2 * n, n).data(),
        sol.segment(1 + 2 * n, n).data() + n
    );

    return x_zmp_star;
}


/* ============================ Compute_gen_poses =========================== */

std::vector<generalized_pose::GeneralizedPoseStruct> MotionPlanner::compute_gen_poses(
    const Vector2d& X_com_0, const Vector2d& Y_com_0,
    double x_zmp_0, double y_zmp_0,
    std::vector<Vector2d>& pos_zmp_star,
    double yaw_rate_cmd,
    const Vector3d& plane_coeffs,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
) {
    // Compute the desired footholds of the feet currently in swing phase.
    compute_desired_footholds(pos_zmp_star);

    // Compute the feet trajectories.
    auto [r_s_ddot_des, r_s_dot_des, r_s_des] = compute_swing_feet_trajectories(
        plane_coeffs,
        feet_positions, feet_velocities
    );

    // Compute the base linear trajectory.
    pos_zmp_star.insert(pos_zmp_star.begin(), {x_zmp_0, y_zmp_0});
    auto [r_b_ddot_des, r_b_dot_des, r_b_des] = get_des_base_poses(X_com_0, Y_com_0, pos_zmp_star);

    std::vector<generalized_pose::GeneralizedPoseStruct> des_gen_poses(n_gen_poses_);

    std::vector<std::string> contact_feet_names = get_other_feet(swing_feet_names_);

    double phi = phi_ - dt_gen_poses_ / interpolator_.get_step_duration();

    for (int i = 0; i < n_gen_poses_; i++) {
        phi += dt_gen_poses_ / interpolator_.get_step_duration();
        double yaw = yaw_ + i * dt_gen_poses_;

        if (phi >= 1) {
            phi--;
            contact_feet_names = get_other_feet(contact_feet_names);
        }

        // Compute the base angular trajectory.
        Vector3d omega_des = {0.0, 0.0, yaw_rate_cmd};
        generalized_pose::Quaternion q_des(0.0, 0.0, std::sin(yaw/2), std::cos(yaw/2));

        des_gen_poses[i] = generalized_pose::GeneralizedPoseStruct(
            r_b_ddot_des[i],
            r_b_dot_des[i],
            r_b_des[i],
            omega_des,
            q_des,
            r_s_ddot_des[i],
            r_s_dot_des[i],
            r_s_des[i],
            contact_feet_names
        );

        correct_base_pose_with_terrain_plane(
            plane_coeffs,
            yaw,
            des_gen_poses[i]
        );
    }

    return des_gen_poses;
}


/* ==================== Compute_trajectory_sample_points ==================== */

std::vector<std::vector<Vector3d>> MotionPlanner::compute_trajectory_sample_points() const
{
    // When the robot is stopped, no trajectories are computed.
    if (stop_flag_) {
        return {};
    }

    std::vector<std::vector<Vector3d>> swing_feet_sampled_trajectories;
    swing_feet_sampled_trajectories.resize(2);
    for (std::vector<Vector3d>& trajectory : swing_feet_sampled_trajectories) {
        trajectory.resize(n_sample_points_);
    }
    
    // Indices of the swing feet.
    std::vector<int> feet_ids = {};
    for (int i = 0; i < 4; i++) {
        if (std::find(swing_feet_names_.begin(), swing_feet_names_.end(), all_feet_names_[i]) != swing_feet_names_.end()) {
            feet_ids.push_back(i);
        }
    }

    // Populate the trajectories of the feet.
    for (int i = 0; i < n_sample_points_; i++) {
        double phi = static_cast<double>(i) / (n_sample_points_ - 1);

        for (int j : feet_ids) {
            auto init_pos = init_pos_swing_feet_[j];
            auto end_pos_xy = final_pos_swing_feet_[0][
                std::find(swing_feet_names_.begin(), swing_feet_names_.end(), all_feet_names_[j]) - swing_feet_names_.begin()
            ];

            Vector3d end_pos = {end_pos_xy[0], end_pos_xy[1], init_pos[2]};

            int jj = static_cast<int>(std::find(feet_ids.begin(), feet_ids.end(), j) - feet_ids.begin());
            swing_feet_sampled_trajectories[jj][i] = std::get<0>(
                interpolator_.interpolate(
                    init_pos, end_pos, phi
            ));
        }
    }

    return swing_feet_sampled_trajectories;
}


/* =============================== Compute_A_t ============================== */

Matrix2d MotionPlanner::compute_A_t(double omega, double time)
{
    Matrix2d A_t;
    

    A_t << std::cosh(omega*time),         1/omega * std::sinh(omega*time),
           omega * std::sinh(omega*time), std::cosh(omega*time);

    return A_t;
}


/* =============================== Compute_b_t ============================== */

Vector2d MotionPlanner::compute_b_t(double omega, double time)
{
    Vector2d b_t;

    b_t << 1 - std::cosh(omega*time),
           - omega * std::sinh(omega*time);

    return b_t;
}


/* ======================== Compute_desired_footholds ======================= */

void MotionPlanner::compute_desired_footholds(const std::vector<Vector2d>& pos_zmp)
{
    final_pos_swing_feet_.resize(n_steps_prediction_horizon);

    auto swing_feet_names = swing_feet_names_;

    double yaw = yaw_;

    for (int i = 0; i < n_steps_prediction_horizon; i++) {

        final_pos_swing_feet_[i] = {};

        for (const std::string& foot_name : swing_feet_names) {
            if (foot_name == "LF") {
                final_pos_swing_feet_[i].emplace_back(
                    pos_zmp[i] + r_ * (Eigen::Vector2d() <<   std::cos(theta_0_ + yaw),   std::sin(theta_0_ + yaw)).finished()
                );    
            } else if (foot_name == "RF") {
                final_pos_swing_feet_[i].emplace_back(
                    pos_zmp[i] + r_ * (Eigen::Vector2d() <<   std::cos(theta_0_ - yaw), - std::sin(theta_0_ - yaw)).finished()
                );    
            } else if (foot_name == "LH") {
                final_pos_swing_feet_[i].emplace_back(
                    pos_zmp[i] + r_ * (Eigen::Vector2d() << - std::cos(theta_0_ - yaw),   std::sin(theta_0_ - yaw)).finished()
                );    
            } else if (foot_name == "RH") {
                final_pos_swing_feet_[i].emplace_back(
                    pos_zmp[i] + r_ * (Eigen::Vector2d() << - std::cos(theta_0_ + yaw), - std::sin(theta_0_ + yaw)).finished()
                );    
            }
        }

        swing_feet_names = get_other_feet(swing_feet_names);
    }
}


/* ===================== Compute_swing_feet_trajectories ==================== */

std::tuple<std::vector<VectorXd>, std::vector<VectorXd>, std::vector<VectorXd>> MotionPlanner::compute_swing_feet_trajectories(
    const Vector3d& plane_coeffs,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities)
{
    bool use_feet_positions = false;

    if (interpolate_swing_feet_from_current_position_) {
        for (const auto& foot_position : feet_positions) {
            if (foot_position.norm() > 1e-6) {
                use_feet_positions = true;
                break;
            }
        }
    }

    std::vector<VectorXd> desired_feet_pos(n_gen_poses_);
    std::vector<VectorXd> desired_feet_vel(n_gen_poses_);
    std::vector<VectorXd> desired_feet_acc(n_gen_poses_);

    int n_future_steps = 0;
    double Ts = interpolator_.get_step_duration();
    double phi = phi_;

    auto swing_feet_names = swing_feet_names_;

    for (int i = 0; i < n_gen_poses_; i++) {
        int n_swing_feet = static_cast<int>(final_pos_swing_feet_[n_future_steps].size());
        VectorXd r_s_des = VectorXd::Zero(3 * n_swing_feet);
        VectorXd r_s_dot_des = VectorXd::Zero(3 * n_swing_feet);
        VectorXd r_s_ddot_des = VectorXd::Zero(3 * n_swing_feet);

        for (int j = 0; j < 4; j++) {
            const auto it = std::find(swing_feet_names.begin(), swing_feet_names.end(), all_feet_names_[j]);

            if (it != swing_feet_names.end()) {
                int k = it - swing_feet_names.begin();

                Vector3d end_pos = {
                    final_pos_swing_feet_[n_future_steps][k][0],
                    final_pos_swing_feet_[n_future_steps][k][1],
                    plane_coeffs[0] * final_pos_swing_feet_[n_future_steps][k][0] + plane_coeffs[1] * final_pos_swing_feet_[n_future_steps][k][1] + plane_coeffs[2]
                };

                Vector3d init_pos;
                if (use_feet_positions && n_future_steps < 2) {
                    init_pos = feet_positions[j];
                } else if (!use_feet_positions && n_future_steps < 2) {
                    init_pos = init_pos_swing_feet_[j];
                    init_pos[2] = plane_coeffs[0] * init_pos[0] + plane_coeffs[1] * init_pos[1] + plane_coeffs[2];
                } else {
                    init_pos = {
                        final_pos_swing_feet_[n_future_steps - 2][k][0],
                        final_pos_swing_feet_[n_future_steps - 2][k][1],
                        plane_coeffs[0] * final_pos_swing_feet_[n_future_steps - 2][k][0] + plane_coeffs[1] * final_pos_swing_feet_[n_future_steps - 2][k][1] + plane_coeffs[2]
                    };
                }
                
                if (use_feet_positions) {
                    std::forward_as_tuple(
                        r_s_des.segment(3*k, 3),
                        r_s_dot_des.segment(3*k, 3),
                        r_s_ddot_des.segment(3*k, 3)
                    ) = interpolator_.interpolate(
                        init_pos, feet_velocities[j],
                        end_pos, {0,0,0},
                        phi, dt_);
                } else {
                    std::forward_as_tuple(
                        r_s_des.segment(3*k, 3),
                        r_s_dot_des.segment(3*k, 3),
                        r_s_ddot_des.segment(3*k, 3)
                    ) = interpolator_.interpolate(init_pos, end_pos, phi);
                }
            }
        }

        phi += dt_gen_poses_ / Ts;
        if (phi >= 1) {
            phi--;
            n_future_steps++;
            swing_feet_names = get_other_feet(swing_feet_names);
        }

        desired_feet_pos[i] = r_s_des;
        desired_feet_vel[i] = r_s_dot_des;
        desired_feet_acc[i] = r_s_ddot_des;
    }

    return {desired_feet_acc, desired_feet_vel, desired_feet_pos};
}


/* ============================ Switch_swing_feet =========================== */

void MotionPlanner::switch_swing_feet(const std::vector<Vector3d>& feet_positions)
{
    auto old_swing_feet_names = swing_feet_names_;
    swing_feet_names_ = {};
    swing_feet_names_.reserve(2);

    int count = 0;

    for (int i = 0; i < 4; i++) {        
        auto it = std::find(old_swing_feet_names.begin(), old_swing_feet_names.end(), all_feet_names_[i]);

        if (it == old_swing_feet_names.end()) {
            swing_feet_names_.push_back(all_feet_names_[i]);

            if (feet_positions.size() == 4) {
                init_pos_swing_feet_[i] << feet_positions[i][0],
                                           feet_positions[i][1],
                                           feet_positions[i][2];

            }
        } else {
            init_pos_swing_feet_[i] << final_pos_swing_feet_[0][count][0],
                                       final_pos_swing_feet_[0][count][1],
                                       0.;

            count++;
        }
    }
}


/* ============================ get_des_base_pose =========================== */

std::tuple<std::vector<Vector3d>, std::vector<Vector3d>, std::vector<Vector3d>> MotionPlanner::get_des_base_poses(
    const Vector2d& X_com_0, const Vector2d& Y_com_0,
    const std::vector<Vector2d>& pos_zmp
) const {
    std::vector<Vector3d> base_pos(n_gen_poses_);
    std::vector<Vector3d> base_vel(n_gen_poses_);
    std::vector<Vector3d> base_acc(n_gen_poses_);

    double g = 9.81;
    double omega = std::pow((g / height_com_), 0.5);

    auto X_com_i = X_com_0;
    auto Y_com_i = Y_com_0;

    int n_future_steps = 0;
    double phi = phi_;
    double Ts = interpolator_.get_step_duration();

    double dt = dt_;

    for (int i = 0; i < n_gen_poses_; i++) {
        Vector2d X_com = compute_A_t(omega, dt) * X_com_i + compute_b_t(omega, dt) * pos_zmp[n_future_steps][0];
        Vector2d Y_com = compute_A_t(omega, dt) * Y_com_i + compute_b_t(omega, dt) * pos_zmp[n_future_steps][1];

        base_pos[i] = {X_com[0], Y_com[0], height_com_};
        base_vel[i] = {X_com[1], Y_com[1], 0};
        base_acc[i] = {X_com_i[0] - pos_zmp[n_future_steps][0], Y_com_i[0] - pos_zmp[n_future_steps][1], 0};
        base_acc[i] *= g / height_com_;

        phi += dt_gen_poses_ / Ts;
        dt += dt_gen_poses_;

        if (phi >= 1) {
            X_com_i = compute_A_t(omega, Ts) * X_com_i + compute_b_t(omega, Ts) * pos_zmp[n_future_steps][0];
            Y_com_i = compute_A_t(omega, Ts) * Y_com_i + compute_b_t(omega, Ts) * pos_zmp[n_future_steps][1];

            phi--;
            n_future_steps++;
            dt = phi * Ts;
        }
    }
    
    return {base_acc, base_vel, base_pos};
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


/* ======================= Correct_with_terrain_plane ======================= */

void MotionPlanner::correct_base_pose_with_terrain_plane(
    const Vector3d& plane_coeffs,
    double yaw,
    generalized_pose::GeneralizedPoseStruct& gen_pose
) {
    // Local terrain height
    double delta_h =   plane_coeffs[0] * gen_pose.base_pos.x
                     + plane_coeffs[1] * gen_pose.base_pos.y
                     + plane_coeffs[2];

    // Shift the desired base pose and the desired feet positions.
    gen_pose.base_pos.z += delta_h;

    // Align the desired base pose and pitch angles to the local terrain plane.
    Quaterniond quat = compute_quaternion_from_euler_angles(
            std::atan(plane_coeffs[1]),
        - std::atan(plane_coeffs[0]),
        yaw
    );

    gen_pose.base_quat = generalized_pose::Quaternion(
        quat.x(), quat.y(), quat.z(), quat.w()
    );
}

} // lip_walking_trot_planner
