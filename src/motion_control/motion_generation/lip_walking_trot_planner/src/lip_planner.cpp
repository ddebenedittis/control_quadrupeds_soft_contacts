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
    const Ref<Vector3d>& pos_0, double yaw,
    const std::vector<Vector3d>& feet_positions)
{
    yaw_ = yaw;

    init_pos_swing_feet_.resize(4);

    if (feet_positions.size() != 4) {
        // The feet_positions has the wrong dimension, do not use it to initialize the swing feet positions.

        // Initialize the _init_pos_swing_feet with the positions of the feet relative to the base in base frame.
        // LF
        init_pos_swing_feet_[0] << r_ * std::cos(theta_0_ + yaw),
                                   r_ * std::sin(theta_0_ + yaw),
                                   0.0;

        // RF
        init_pos_swing_feet_[1] <<   r_ * std::cos(theta_0_ - yaw),
                                   - r_ * std::sin(theta_0_ - yaw),
                                     0.0;

        // LH
        init_pos_swing_feet_[2] << - r_ * std::cos(theta_0_ - yaw),
                                     r_ * std::sin(theta_0_ - yaw),
                                     0.0;

        // RH
        init_pos_swing_feet_[3] << - r_ * std::cos(theta_0_ + yaw),
                                   - r_ * std::sin(theta_0_ + yaw),
                                     0.0;

        // Shift the quantity by the initial position of the base.
        for (auto& foot_pos : init_pos_swing_feet_) {
            foot_pos.head(2) += pos_0.head(2);
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

generalized_pose::GeneralizedPoseStruct MotionPlanner::update(
    const Ref<Vector3d>& pos_com, const Ref<Vector3d>& vel_com, const Ref<Vector3d>& acc_com,
    const Ref<Vector2d>& vel_cmd, double yaw_rate_cmd,
    const Vector3d& plane_coeffs,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
) {
    stop_flag_ = check_stop(vel_cmd, yaw_rate_cmd);

    if (stop_flag_) {
        return  stand_still(plane_coeffs);
    } else {        
        return mpc(
            pos_com, vel_com, acc_com,
            vel_cmd, yaw_rate_cmd,
            plane_coeffs,
            feet_positions, feet_velocities
        );
    }
}


/* =============================== Stand_still ============================== */

generalized_pose::GeneralizedPoseStruct MotionPlanner::stand_still(
    const Vector3d& plane_coeffs
) const {
    // The commanded position along the x and y coordinates is the mean of the positions of the contact feet.
    double pos_x = 0;
    double pos_y = 0;

    for (const auto& elem : init_pos_swing_feet_) {
        pos_x += elem[0] / static_cast<double>(init_pos_swing_feet_.size());
        pos_y += elem[1] / static_cast<double>(init_pos_swing_feet_.size());
    }

    double roll = std::atan(plane_coeffs[1]);
    double pitch = - std::atan(plane_coeffs[0]);

    Quaterniond quat = compute_quaternion_from_euler_angles(
        roll,
        pitch,
        yaw_
    );

    return  {
        generalized_pose::Vector3(
            pos_x + height_com_ * std::sin(pitch),
            pos_y - height_com_ * std::sin(roll),
            height_com_ * std::cos(roll) * std::cos(pitch)
        ),
        generalized_pose::Quaternion(
            quat.x(), quat.y(), quat.z(), quat.w()
        )
    };
}


/* =================================== Mpc ================================== */

generalized_pose::GeneralizedPoseStruct MotionPlanner::mpc(
    const Vector3d& pos_com, const Vector3d& vel_com, const Vector3d& acc_com,
    const Vector2d& vel_cmd, double yaw_rate_cmd,
    const Vector3d& plane_coeffs,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities
) {
    // Initial CoM position
    Vector2d X_com_0 = {pos_com[0], vel_com[0]};
    Vector2d Y_com_0 = {pos_com[1], vel_com[1]};

    // Initial ZMP position (from the dynamics of a linear inverted pendulum).
    double g = 9.81;
    double x_zmp_0 = pos_com[0] - acc_com[0] * height_com_ / g;
    double y_zmp_0 = pos_com[1] - acc_com[1] * height_com_ / g;
    
    // Compute the heading and lateral coordinates of the zmp
    double h_zmp_0 =   x_zmp_0 * std::cos(yaw_) + y_zmp_0 * std::sin(yaw_);
    double l_zmp_0 = - x_zmp_0 * std::sin(yaw_) + y_zmp_0 * std::cos(yaw_);
    
    Vector2d H_com_0 =   X_com_0 * std::cos(yaw_) + Y_com_0 * std::sin(yaw_);
    Vector2d L_com_0 = - X_com_0 * std::sin(yaw_) + Y_com_0 * std::cos(yaw_);

    // Compute the x and y coordinates of the ZMP.
    double h_com_dot_des = vel_cmd[0];
    double l_com_dot_des = vel_cmd[1];
    double h_zmp_star = mpc_qp(H_com_0, h_zmp_0, h_com_dot_des);
    double l_zmp_star = mpc_qp(L_com_0, l_zmp_0, l_com_dot_des);
    
    double x_zmp_star = h_zmp_star * std::cos(yaw_) - l_zmp_star * std::sin(yaw_);
    double y_zmp_star = h_zmp_star * std::sin(yaw_) + l_zmp_star * std::cos(yaw_);

    // Optimal ZMP position.
    Vector2d pos_zmp_star = {x_zmp_star, y_zmp_star};

    // Compute the desired footholds of the feet currently in swing phase.
    yaw_ += yaw_rate_cmd * dt_;
    compute_desired_footholds(pos_zmp_star);

    // Compute the feet trajectories.
    auto [r_s_ddot_des, r_s_dot_des, r_s_des] = compute_swing_feet_trajectories(
        plane_coeffs,
        feet_positions, feet_velocities
    );

    // Compute the base linear trajectory.
    auto [r_b_ddot_des, r_b_dot_des, r_b_des] = get_des_base_pose(X_com_0, Y_com_0, x_zmp_0, y_zmp_0);

    // Compute the base angular trajectory.
    Vector3d omega_des = {0.0, 0.0, yaw_rate_cmd};
    generalized_pose::Quaternion q_des(0.0, 0.0, std::sin(yaw_/2), std::cos(yaw_/2));

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
        switch_swing_feet(feet_positions);
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

    correct_with_terrain_plane(
        plane_coeffs,
        des_gen_pose
    );
    
    return des_gen_pose;
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
            auto end_pos_xy = final_pos_swing_feet_[
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


/* ================================= Mpc_qp ================================= */

double MotionPlanner::mpc_qp(
    const Vector2d& X_com_0, double x_zmp_0,
    double x_com_dot_des
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


    VectorXd x_com_dot_des_vec = x_com_dot_des * VectorXd::Ones(n_);

    double c0 = x_com_dot_des_vec.transpose() * Q_ * x_com_dot_des_vec + R_(0,0) * std::pow(x_zmp_0, 2);

    VectorXd c1 = kron_product(- 2 * Q_ * x_com_dot_des_vec, (VectorXd(2) << 0, 1).finished());

    VectorXd c2 = VectorXd::Zero(n_);
    c2(0) = - 2 * R_(0, 0) * x_zmp_0;

    VectorXd c(1 + c1.size() + c2.size());
    c << c0, c1, c2;


    /* ======================== Equality Constraints ======================== */

    // Enforces dynamic consistency with the linearized inverted pendulum model

    double g = 9.81;
    double omega = std::pow((g / height_com_), 0.5);

    Matrix2d A_t = compute_A_t(omega, Ts);
    Vector2d b_t = compute_b_t(omega, Ts);

    MatrixXd I_off_diag = MatrixXd::Zero(n_, n_);
    I_off_diag.diagonal(-1).setOnes();

    MatrixXd A = MatrixXd::Zero(1 + 2*n_, 1 + 3*n_);
    A(0, 0) = 1;
    A.block(1, 1, 2*n_, 2*n_) = - MatrixXd::Identity(2*n_, 2*n_)
                                + kron_product(I_off_diag, A_t);
    A.bottomRightCorner(2*n_, n_) = kron_product(MatrixXd::Identity(n_, n_), (MatrixXd(2,1) << b_t(0), b_t(1)).finished());

    double t_0 = Ts * (1. - phi_);
    Matrix2d A_t0 = compute_A_t(omega, t_0);
    Vector2d b_t0 = compute_b_t(omega, t_0);

    VectorXd b = VectorXd::Zero(1 + 2*n_);
    b(0) = 1;
    b.segment(1, 2) = - A_t * (A_t0 * X_com_0 + b_t0 * x_zmp_0);


    /* ======================= Inequality Constraints ======================= */
    
    // Limit the zmp displacement between two consecutive steps

    MatrixXd D = MatrixXd::Zero(2 * n_, 1 + 3*n_);
    D.topRightCorner(n_, n_) = MatrixXd::Identity(n_, n_) - I_off_diag;
    D.bottomRightCorner(n_, n_) = - D.topRightCorner(n_, n_);

    VectorXd f(2 * n_);
    f(0) = x_zmp_0 + step_reachability_;
    f.segment(1, n_-1).setConstant(step_reachability_);
    f(n_) = - x_zmp_0 + step_reachability_;
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

    double x_zmp_star = sol(1 + 2 * n_);

    return x_zmp_star;
}


/* ======================== Compute_desired_footholds ======================= */

void MotionPlanner::compute_desired_footholds(const Vector2d& pos_zmp)
{
    final_pos_swing_feet_ = {};

    for (const std::string& foot_name : swing_feet_names_) {
        if (foot_name == "LF") {
            final_pos_swing_feet_.emplace_back(
                pos_zmp + r_ * (Eigen::Vector2d() <<   std::cos(theta_0_ + yaw_),   std::sin(theta_0_ + yaw_)).finished()
            );    
        } else if (foot_name == "RF") {
            final_pos_swing_feet_.emplace_back(
                pos_zmp + r_ * (Eigen::Vector2d() <<   std::cos(theta_0_ - yaw_), - std::sin(theta_0_ - yaw_)).finished()
            );    
        } else if (foot_name == "LH") {
            final_pos_swing_feet_.emplace_back(
                pos_zmp + r_ * (Eigen::Vector2d() << - std::cos(theta_0_ - yaw_),   std::sin(theta_0_ - yaw_)).finished()
            );    
        } else if (foot_name == "RH") {
            final_pos_swing_feet_.emplace_back(
                pos_zmp + r_ * (Eigen::Vector2d() << - std::cos(theta_0_ + yaw_), - std::sin(theta_0_ + yaw_)).finished()
            );    
        }
    }
}


/* ===================== Compute_swing_feet_trajectories ==================== */

std::tuple<VectorXd, VectorXd, VectorXd> MotionPlanner::compute_swing_feet_trajectories(
    const Vector3d& plane_coeffs,
    const std::vector<Vector3d>& feet_positions, const std::vector<Vector3d>& feet_velocities)
{
    bool good_feet_positions = false;

    if (interpolate_swing_feet_from_current_position_) {
        for (const auto& foot_position : feet_positions) {
            if (foot_position.norm() > 1e-6) {
                good_feet_positions = true;
                break;
            }
        }
    }

    int n_swing_feet = static_cast<int>(final_pos_swing_feet_.size());
    VectorXd r_s_des = VectorXd::Zero(3 * n_swing_feet);
    VectorXd r_s_dot_des = VectorXd::Zero(3 * n_swing_feet);
    VectorXd r_s_ddot_des = VectorXd::Zero(3 * n_swing_feet);

    for (int i = 0; i < 4; i++) {
        const auto it = std::find(swing_feet_names_.begin(), swing_feet_names_.end(), all_feet_names_[i]);

        if (it != swing_feet_names_.end()) {
            int j = it - swing_feet_names_.begin();

            Vector3d end_pos = {
                final_pos_swing_feet_[j][0],
                final_pos_swing_feet_[j][1],
                plane_coeffs[0] * final_pos_swing_feet_[j][0] + plane_coeffs[1] * final_pos_swing_feet_[j][1] + plane_coeffs[2]
            };
            
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
                init_pos[2] = plane_coeffs[0] * init_pos[0] + plane_coeffs[1] * init_pos[1] + plane_coeffs[2];

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
            init_pos_swing_feet_[i] << final_pos_swing_feet_[count][0],
                                       final_pos_swing_feet_[count][1],
                                       0.;

            count++;
        }
    }
}


/* ============================ Get_des_base_pose =========================== */

std::tuple<Vector3d, Vector3d, Vector3d> MotionPlanner::get_des_base_pose(
    const Vector2d& X_com_0, const Vector2d& Y_com_0,
    double x_zmp_0, double y_zmp_0
) const {
    double g = 9.81;
    double omega = std::pow((g / height_com_), 0.5);
    
    Vector2d Xcom = compute_A_t(omega, dt_) * X_com_0 + compute_b_t(omega, dt_) * x_zmp_0;
    Vector2d Ycom = compute_A_t(omega, dt_) * Y_com_0 + compute_b_t(omega, dt_) * y_zmp_0;

    Vector3d r_b_des = {Xcom[0], Ycom[0], height_com_};
    Vector3d r_b_dot_des = {Xcom[1], Ycom[1], 0};
    Vector3d r_b_ddot_des = {X_com_0[0] - x_zmp_0, Y_com_0[0] - y_zmp_0, 0};
    r_b_ddot_des *= g / height_com_;

    return {r_b_ddot_des, r_b_dot_des, r_b_des};
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

void MotionPlanner::correct_with_terrain_plane(
    const Vector3d& plane_coeffs,
    generalized_pose::GeneralizedPoseStruct& gen_pose
) const {
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
          yaw_
    );

    gen_pose.base_quat = generalized_pose::Quaternion(
        quat.x(), quat.y(), quat.z(), quat.w()
    );
}

} // lip_walking_trot_planner
