/* ========================================================================== */
/*                                 DESCRIPTION                                */
/* ========================================================================== */

/*

*/



/* ========================================================================== */
/*                              IMPORT LIBRARIES                              */
/* ========================================================================== */

#include "whole_body_controller/control_tasks.hpp"

#include "robot_model/robot_model.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>



/* ========================================================================== */
/*                                    CODE                                    */
/* ========================================================================== */

namespace wbc {


/* ======================== ControlTasks Constructor ======================== */

ControlTasks::ControlTasks(std::string robot_name) : robot_model(robot_name) {}


/* ================================== reset ================================= */

void ControlTasks::reset(Eigen::VectorXd& q, Eigen::VectorXd& v, std::vector<std::string> contact_feet_names) {
    robot_model.set_contact_feet_names(contact_feet_names);

    robot_model.compute_swing_feet_names();

    robot_model.compute_EOM(q, v);

    nc = static_cast<int>(contact_feet_names.size());
    nc3 = 3 * nc;

    M = Eigen::MatrixXd::Zero(nv, nv);
    h = Eigen::VectorXd::Zero(nv);
    Jc = Eigen::MatrixXd::Zero(nc3, nv);

    Jb = Eigen::MatrixXd::Zero(6, nv);

    Jb_dot_times_v = Eigen::VectorXd::Zero(6);
}


/* ========================= task_floating_base_eom ========================= */

using namespace Eigen;

void ControlTasks::task_floating_base_eom(Ref<MatrixXd>& A, Ref<VectorXd>& b) {
    // Compute the EOM quantities
    robot_model.compute_EOM(q, v);
    M = robot_model.get_data().M;
    h = robot_model.get_data().nle;

    A.leftCols(nv) = M.topRows(6);

    A.rightCols(nc3) = - Jc.leftCols(6).transpose();

    b = - h.topRows(6);
}


/* =========================== Task_torque_limits =========================== */

void ControlTasks::task_torque_limits(Ref<MatrixXd>& C, Ref<VectorXd>& d) {
    C.topLeftCorner(nv-6, nv) = M.bottomRows(nv-6);
    C.topRightCorner(nv-6, nc3) = - Jc.rightCols(nv-6).transpose();

    d.topRows(nv-6) = VectorXd::Ones(nv-6) * tau_max - h.bottomRows(nv-6);

    C.bottomRows(nv-6) = - C.topRows(nv-6);
    d.bottomRows(nv-6) = - d.topRows(nv-6);
}


/* ======================= Task_friction_Fc_modulation ====================== */

void ControlTasks::task_friction_Fc_modulation(Ref<MatrixXd>& C, Ref<VectorXd>& d) {
    MatrixXd he(nc, nc3);
    MatrixXd la(nc, nc3);
    MatrixXd  n(nc, nc3);

    for (int i = 0; i < nc; i++) {
        he.block(i, 3*i, 1, 3) << 1, 0, 0;
        la.block(i, 3*i, 1, 3) << 0, 1, 0;
         n.block(i, 3*i, 1, 3) << 0, 0, 1;
    }


    C.block(   0, nv, nc, nc3) =   he - mu * n;
    C.block(  nc, nv, nc, nc3) = - he - mu * n;
    C.block(2*nc, nv, nc, nc3) =   la - mu * n;
    C.block(3*nc, nv, nc, nc3) = - la - mu * n;

    C.block(4*nc, nv, nc, nc3) =   n;
    C.block(5*nc, nv, nc, nc3) = - n;


    d.segment(4*nc, nc) =   Fn_max * VectorXd::Ones(nv-6);
    d.segment(5*nc, nc) = - Fn_min * VectorXd::Ones(nv-6);
}


/* ======================= Task_linear_motion_tracking ====================== */

void ControlTasks::task_linear_motion_tracking(
    Ref<MatrixXd>& A, Ref<VectorXd>& b,
    VectorXd& r_b_ddot_des, VectorXd& r_b_dot_des, VectorXd& r_b_des
) {
    robot_model.compute_second_order_FK(q, v);
    robot_model.get_Jb(Jb);
    robot_model.get_Jb_dot_times_v(Jb_dot_times_v);

    A.leftCols(nv) = Jb.topRows(3);

    b =   r_b_ddot_des 
        + Kd_b_pos * (r_b_dot_des - Jb.topRows(3) * v)
        + Kp_b_pos * (r_b_des - q.head(3))
        - Jb_dot_times_v.topRows(3);
}


/* ====================== Task_angular_motion_tracking ====================== */

void ControlTasks::task_angular_motion_tracking(
    Ref<MatrixXd>& A, Ref<VectorXd>& b,
    Vector3d& omega_des, Vector4d& q_des
) {
    // TODO check that this is correct. It depends on how I define q_des.
    // Quaternion<double>(w, x, y, z);
    Quaterniond quat_des = Quaternion<double>(q_des(3), q_des(0), q_des(1), q_des(2));
    Quaterniond quat = Quaternion<double>(q(6), q(3), q(4), q(5));

    pinocchio::FrameIndex base_id = 1;
    MatrixXd oRb = robot_model.get_data().oMi[base_id].rotation();

    A.leftCols(nv) = Jb.bottomRows(3);

    b =   Kd_b_ang * (omega_des - oRb * v.segment(3, 3))
        + Kp_b_ang * (pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()))
        - Jb_dot_times_v.bottomRows(3);
}


/* ======================== Task_swing_feet_tracking ======================== */

void ControlTasks::task_swing_feet_tracking(
    Ref<MatrixXd>& A, Ref<VectorXd>& b,
    VectorXd& r_s_ddot_des, VectorXd& r_s_dot_des, VectorXd& r_s_des
) {
    MatrixXd Js(4*3-nc3, nv);
    robot_model.get_Js(Js);

    VectorXd Js_dot_times_v(4*3-nc3);
    robot_model.get_Js_dot_times_v(Js_dot_times_v);

    VectorXd r_s(4*3-nc3);
    robot_model.get_r_s(r_s);

    A.leftCols(nv) = Js;

    b =   r_s_ddot_des 
        + Kd_s_pos * (r_s_dot_des - Js * v)
        + Kp_s_pos * (r_s_des - r_s)
        - Jb_dot_times_v;
}


/* ======================== Task_contact_constraints ======================== */

void ControlTasks::task_contact_constraints() {

}


/* ===================== Task_energy_forces_optimization ==================== */

void ControlTasks::task_energy_forces_optimization(Ref<MatrixXd>& A, Ref<VectorXd>& b) {
    A.topLeftCorner(nv-6, nv) = M.bottomRows(nv-6);
    A.topRightCorner(nv-6, nc3) = - Jc.rightCols(nv-6).transpose();

    b.topRows(nv-6) = - h.bottomRows(nv-6);

    A.block(nv-6, nv, nc3, nc3) = MatrixXd::Identity(nc3, nc3);

    A.bottomRightCorner(nc3, nc3) = MatrixXd::Identity(nc3, nc3);
}


} // namespace wbc