#include "whole_body_controller/mpc_control_tasks.hpp"



namespace wbc {

using namespace Eigen;

/* ================================ Compute_T =============================== */

void MPCControlTasks::compute_T()
{
    T_.resize(nv+1, nv);
    T_.setZero();

    T_.topLeftCorner(3, 3).diagonal().setOnes();
    const auto qw = q(6);
    const auto qx = q(3);
    const auto qy = q(4);
    const auto qz = q(5);
    T_.block(3, 3, 4, 3) <<
         qw/2, -qz/2,  qy/2,
         qz/2,  qw/2, -qx/2,
        -qy/2,  qx/2,  qw/2,
        -qx/2, -qy/2, -qz/2;
    T_.bottomRightCorner(nv-6, nv-6).diagonal().setOnes();


    T_pinv_.resize(nv, nv+1);
    T_pinv_.setZero();

    T_.topLeftCorner(3, 3).diagonal().setOnes();
    T_.block(3, 3, 3, 4) <<
         2*qw,  2*qz, -2*qy, -2*qx,
        -2*qz,  2*qw,  2*qx, -2*qy,
         2*qy, -2*qx,  2*qw, -2*qz;
    T_.bottomRightCorner(nv-6, nv-6).diagonal().setOnes();
}

/* ================================== Tile ================================== */

/// @internal @brief Create a new vector by repeating the input vector: out = [v, v, ...]
/// @param[in] v
/// @param[in] repeat
/// @return Eigen::VectorXd 
namespace {
Eigen::VectorXd tile(const Eigen::VectorXd& v, int repeat)
{
    Eigen::VectorXd temp(v.size() * repeat);

    for (int i = 0; i < repeat; i++) {
        temp.segment(i*v.size(), v.size()) = v;
    }

    return temp;
}
}


/* ================================== Reset ================================= */

void MPCControlTasks::reset(const std::vector<std::string>& contact_feet_names)
{
    robot_model.set_feet_names(contact_feet_names);

    nc = static_cast<int>(contact_feet_names.size());
    nF = 3 * nc;

    Jc = Eigen::MatrixXd::Zero(3*nc, nv);
    robot_model.get_Jc(Jc);
    Js = Eigen::MatrixXd::Zero(12 - 3*nc, nv);
    robot_model.get_Js(Js);
}


/* ========================= Task_first_mpc_dynamics ======================== */

void MPCControlTasks::task_first_mpc_dynamics(
    Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b
) {
    compute_T();

    // q_1 = q_0 + T v_0 dt_mpc + 1/2 T v_dot_0 dt_mpc^2
    A.topLeftCorner(nv+1, nv+1).diagonal().setOnes();
    A.block(0, 2*nv+1, nv+1, nv) = - 1./2. * dt_mpc_ * dt_mpc_ * T_;
    b.head(nv+1) = q + T_ * v * dt_mpc_;

    // v_1 = v_0 + v_dot_0 dt_mpc
    A.block(nv+1, nv+1, nv, nv).diagonal().setOnes();
    A.block(nv+1, 2*nv+1, nv, nv).setConstant(- dt_mpc_);
    b.tail(nv) = v;
}


/* ========================= Task_later_mpc_dynamics ======================== */

void MPCControlTasks::task_later_mpc_dynamics(
    Eigen::Ref<Eigen::MatrixXd> A
) const {
    const int nx = 3*nv+1 + nF + nd;

    // q_k+1 = q_k + T v_k dt_mpc + 1/2 T v_dot_k dt_mpc^2
    A.block(0, nx, nv+1, nv+1).diagonal().setOnes();                    // q_k+1
    A.block(0, nx+2*nv+1, nv+1, nv) = - 0.5 * dt_mpc_ * dt_mpc_ * T_;   // v_dot_k
    A.topRightCorner(nv+1, nv+1) = - MatrixXd::Identity(nv+1, nv+1);    // q_k
    A.block(0, nv+1, nv+1, nv) = - T_ * dt_mpc_;                        // v_k

    // v_k+1 = v_k + v_dot_k * dt_mpc_
    A.block(nv+1, nx + nv+1, nv, nv).diagonal().setOnes();                  // v_k+1
    A.block(nv+1, nv+1, nv, nv).diagonal().setConstant(- 1);                // v_k
    A.block(nv+1, nx + 2*nv+1, nv, nv).diagonal().setConstant(- dt_mpc_);   // v_dot_k
}


/* ======================= Task_linear_motion_tracking ====================== */

void MPCControlTasks::task_linear_motion_tracking(
    Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
    const Eigen::Vector3d& r_b_ddot_des, const Eigen::Vector3d& r_b_dot_des, const Eigen::Vector3d& r_b_des,
    int i
) {
    // Compute the second order kinematics
    robot_model.compute_second_order_FK(q, v);
    robot_model.get_Jb(Jb);
    robot_model.get_Jb_dot_times_v(Jb_dot_times_v);

    if (i == 0) {
        // A = [ Jb_pos, 0, 0 ]   ∈ 3 x (nv+nF+nd)

        A.leftCols(nv) = Jb.topRows(3);

        b =   r_b_ddot_des 
            + kd_b_pos.asDiagonal() * (r_b_dot_des - v.head(3))
            + kp_b_pos.asDiagonal() * (r_b_des - q.head(3))
            - Jb_dot_times_v.topRows(3);
    } else {
        //             q_k-1,       v_k-1, ., ., q, v,  v_dot, . ]
        // A = [ Kp Jb_pos T, Kd Jb_pos T, 0, 0, 0, 0, Jb_pos, 0 ]   ∈ 3 x 2 (nv+1+nv+nv+nF+nd)
        // b = [r_b_ddot_des + Kd r_b_dot_des + Kp r_b_des]

        A.leftCols(3) = kp_b_pos.asDiagonal() * Jb.topRows(3);
        A.middleCols(nv+1, 3) = kd_b_pos.asDiagonal() * Jb.topRows(3); // TODO: add Jb_dot
        const int nx = 3*nv+1 + nF + nd;
        A.middleCols(nx + 2*nv+1, 3) = Jb.topRows(3);

        b = r_b_ddot_des
            + kd_b_pos.asDiagonal() * r_b_dot_des
            + kp_b_pos.asDiagonal() * r_b_des;
    }

    // std::cout << "MPC A:\n" << A << std::endl;
    // std::cout << "MPC b:\n" << b << std::endl;
}


/* ====================== Task_angular_motion_tracking ====================== */

void MPCControlTasks::task_angular_motion_tracking(
    Ref<MatrixXd> A, Ref<VectorXd> b,
    const Vector3d& omega_des, const Vector4d& q_des,
    int i
) const {
    // TODO check that this is correct. It depends on how I define q_des.
    // Quaternion<double>(w, x, y, z);
    Quaterniond quat_des = Quaternion<double>(q_des(3), q_des(0), q_des(1), q_des(2));
    Quaterniond quat = Quaternion<double>(q(6), q(3), q(4), q(5));

    // pinocchio::FrameIndex base_id = 1;
    // MatrixXd oRb = robot_model.get_data().oMi[base_id].rotation();

    if (i == 0) {
        A.leftCols(nv) = Jb.bottomRows(3);

        b =   kd_b_ang.asDiagonal() * (omega_des - v.segment(3, 3))        // omega in in body frame with Pinocchio, I want it in inertial frame
            + kp_b_ang.asDiagonal() * (pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()))
            - Jb_dot_times_v.bottomRows(3);
    } else {
        A.middleCols(3, 4) = kp_b_ang * Jb.bottomRows(3) * T_pinv_.block(3, 3, 3, 4);
        A.middleCols(nv+1 + 3, 4) = kd_b_ang * Jb.bottomRows(3); // TODO: add Jb_dot
        const int nx = 3*nv+1 + nF + nd;
        A.middleCols(nx + 2*nv + 4, 3) = Jb.bottomRows(3);

        b =   kd_b_ang.asDiagonal() * omega_des
            + kp_b_ang.asDiagonal() * (pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()))
            + kp_b_ang.asDiagonal() * Jb.bottomRows(3) * T_pinv_.block(3, 3, 3, 4) * quat.coeffs(); // quat.coeffs() = {qx, qy, qz, qw}
    }
}


/* ======================== Task_swing_feet_tracking ======================== */

void MPCControlTasks::task_swing_feet_tracking(
    Ref<MatrixXd> A, Ref<VectorXd> b,
    const VectorXd& r_s_ddot_des, const VectorXd& r_s_dot_des, const VectorXd& r_s_des,
    int i
) {
    if (nc == 4) {
        return;
    }
    
    VectorXd Js_dot_times_v = VectorXd::Zero(4*3-nF);
    robot_model.get_Js_dot_times_v(Js_dot_times_v);

    VectorXd r_s = VectorXd::Zero(4*3-nF);
    robot_model.get_r_s(r_s);

    if (i == 0) {
        A.leftCols(nv) = Js;

        b =   r_s_ddot_des 
            + tile(kd_s_pos, 4-nc).asDiagonal() * (r_s_dot_des - Js * v)
            + tile(kp_s_pos, 4-nc).asDiagonal() * (r_s_des - r_s)
            - Js_dot_times_v;
    } else {
        A.leftCols(nv+1) = tile(kp_s_pos, 4-nc).asDiagonal() * Js * T_pinv_;
        A.middleCols(nv+1, nv) = tile(kp_s_pos, 4-nc).asDiagonal() * Js; // TODO: add Js_dot
        const int nx = 3*nv+1 + nF + nd;
        A.middleCols(nx + 2*nv+1, nv) = Js;

        b = r_s_ddot_des 
            + tile(kd_s_pos, 4-nc).asDiagonal() * r_s_dot_des
            + tile(kp_s_pos, 4-nc).asDiagonal() * ((r_s_des - r_s) + Js * T_pinv_ * q);
    }
}


/* ======================== Task_joint_singularities ======================== */

void MPCControlTasks::task_joint_singularities(
    Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> /*d*/
) {
    // Initialize knee_joint_sign if not already done. The desired knee joint angle sign is the same as the starting knee joint angle.
    if (knee_joint_sign[0] == 0) {
        for (int i = 0; i < 4; i++) {
            if (q[7 + 2+3*i] >= 0) {
                knee_joint_sign[i] = 1;
            } else {
                knee_joint_sign[i] = -1;
            }
        }
    }

    for (int i=0; i<4; i++) {
        C(i, 7 + 2+3*i) = - knee_joint_sign[i];
    }
}

} // namespace wbc