#include "whole_body_controller/control_tasks.hpp"

#include "robot_model/robot_model.hpp"

#include <Eigen/Geometry>



namespace wbc {


/* ================================== Tile ================================== */

/// @internal @brief Create a new vector by repeating the input vector: out = [v, v, ...]
/// @param[in] v
/// @param[in] repeat
/// @return Eigen::VectorXd 
Eigen::VectorXd tile(const Eigen::VectorXd& v, int repeat)
{
    Eigen::VectorXd temp(v.size() * repeat);

    for (int i = 0; i < repeat; i++) {
        temp.segment(i*v.size(), v.size()) = v;
    }

    return temp;
}



/* ========================================================================== */
/*                            CONTROLTASKS METHODS                            */
/* ========================================================================== */


/* ======================== ControlTasks Constructor ======================== */

ControlTasks::ControlTasks(const std::string& robot_name, float dt)
: robot_model(robot_name),
  nv(robot_model.get_model().nv),
  dt(dt)
{}


/* ================================== reset ================================= */

void ControlTasks::reset(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const std::vector<std::string>& contact_feet_names, ContactConstraintType contact_constraint_type)
{
    // Update the joints position and velocity vector
    this->q = q;
    this->v = v;

    robot_model.set_feet_names(contact_feet_names);

    robot_model.compute_EOM(q, v);

    nc = static_cast<int>(contact_feet_names.size());
    nF = 3 * nc;
    
    if (contact_constraint_type == ContactConstraintType::soft_kv) {
        nd = nF;
    } else if (contact_constraint_type == ContactConstraintType::soft_sim) {
        nd = nc;
    } else if (contact_constraint_type == ContactConstraintType::rigid) {
        nd = 0;
    }
    

    // Initialize these matrices with all zeros (required by Pinocchio library)
    Jc = Eigen::MatrixXd::Zero(3*nc, nv);
    Jb = Eigen::MatrixXd::Zero(6, nv);
    Jb_dot_times_v = Eigen::VectorXd::Zero(6);
}

/* ========================================================================== */
/*                                    TASKS                                   */
/* ========================================================================== */

/* General p-th task:

   we (Ap x - bp)  = v
   wi (Cp x - dp) <= w

   Optimization vector:

   x = [ u_dot
         F_c
         d_des ]
*/

/* ========================= task_floating_base_eom ========================= */

using namespace Eigen;

void ControlTasks::task_floating_base_eom(Ref<MatrixXd> A, Ref<VectorXd> b)
{
    // Get the EOM quantities
    M = robot_model.get_data().M;
    h = robot_model.get_data().nle;

    robot_model.get_Jc(Jc);
    
    // A = [ M_u, - Jc_u.T, 0 ];   ∈ 6 x (nv+nF+nd) ]   ∈ 6 x (nv+nF+nd)
    // b = - h_u

    A.leftCols(nv) = M.topRows(6);
    A.middleCols(nv, nF) = - Jc.leftCols(6).transpose();

    b = - h.topRows(6);
}


/* =========================== Task_torque_limits =========================== */

void ControlTasks::task_torque_limits(Ref<MatrixXd> C, Ref<VectorXd> d)
{
    // C = [   M_a, - Jc_a.T, 0 ]
    //     [ - M_a,   Jc_a.T, 0 ]   ∈ 2(nv-6) x (nv+nF+nd)
    // d = [   tau_max - h_a ]
    //     [ - tau_min + h_a ]

    C.topLeftCorner(nv-6, nv) = M.bottomRows(nv-6);
    C.block(0, nv, nv-6, nF) = - Jc.rightCols(nv-6).transpose();

    d.head(nv-6) = VectorXd::Ones(nv-6) * tau_max - h.tail(nv-6);

    C.bottomRows(nv-6) = - C.topRows(nv-6);
    // d.bottomRows(nv-6) = - d.topRows(nv-6);
    d.tail(nv-6) = VectorXd::Ones(nv-6) * tau_max + h.tail(nv-6);
}


/* ======================= Task_friction_Fc_modulation ====================== */

void ControlTasks::task_friction_Fc_modulation(Ref<MatrixXd> C, Ref<VectorXd> d) const
{
    // For example, with nc = 3:
    //      [ 1 0 0 0 0 0 0 0 0 ]
    // he = [ 0 0 0 1 0 0 0 0 0 ]   ∈ nc x (3 nc)
    //      [ 0 0 0 0 0 0 1 0 0 ]

    MatrixXd he = MatrixXd::Zero(nc, nF);
    MatrixXd la = MatrixXd::Zero(nc, nF);
    MatrixXd  n = MatrixXd::Zero(nc, nF);

    for (int i = 0; i < nc; i++) {
        he.block(i, 3*i, 1, 3) << 1, 0, 0;
        la.block(i, 3*i, 1, 3) << 0, 1, 0;
         n.block(i, 3*i, 1, 3) << 0, 0, 1;
    }


    //     [ 0_(nc, nv), + he - mu*n, 0_(nc, nf) ]
    //     | 0,          - he - mu*n, 0          |
    // C = | 0,          + la - mu*n, 0          |   ∈ 6(nc) x (nv+nF+nd)
    //     | 0,          - la - mu*n, 0          |
    //     | 0,                  + n, 0          |
    //     [ 0,                  - n, 0          ]

    //     [   0      ]
    //     |   0      |
    // d = |   0      |   
    //     |   0      |
    //     |   Fn_max |
    //     [ - Fn_min ]

    C.block(   0, nv, nc, nF) =   he - mu * n;
    C.block(  nc, nv, nc, nF) = - he - mu * n;
    C.block(2*nc, nv, nc, nF) =   la - mu * n;
    C.block(3*nc, nv, nc, nF) = - la - mu * n;

    C.block(4*nc, nv, nc, nF) =   n;
    C.block(5*nc, nv, nc, nF) = - n;


    d.segment(4*nc, nc) =   Fn_max * VectorXd::Ones(nc);
    d.segment(5*nc, nc) = - Fn_min * VectorXd::Ones(nc);
}


/* ======================= Task_linear_motion_tracking ====================== */

void ControlTasks::task_linear_motion_tracking(
    Ref<MatrixXd> A, Ref<VectorXd> b,
    const Vector3d& r_b_ddot_des, const Vector3d& r_b_dot_des, const Vector3d& r_b_des
) {
    // Compute the second order kinematics
    robot_model.compute_second_order_FK(q, v);
    robot_model.get_Jb(Jb);
    robot_model.get_Jb_dot_times_v(Jb_dot_times_v);

    // A = [ Jb_pos, 0, 0 ]   ∈ 3 x (nv+nF+nd)

    A.leftCols(nv) = Jb.topRows(3);

    b =   r_b_ddot_des 
        + kd_b_pos.asDiagonal() * (r_b_dot_des - v.head(3))
        + kp_b_pos.asDiagonal() * (r_b_des - q.head(3))
        - Jb_dot_times_v.topRows(3);
}


/* ====================== Task_angular_motion_tracking ====================== */

void ControlTasks::task_angular_motion_tracking(
    Ref<MatrixXd> A, Ref<VectorXd> b,
    const Vector3d& omega_des, const Vector4d& q_des
) const {
    // TODO check that this is correct. It depends on how I define q_des.
    // Quaternion<double>(w, x, y, z);
    Quaterniond quat_des = Quaternion<double>(q_des(3), q_des(0), q_des(1), q_des(2));
    Quaterniond quat = Quaternion<double>(q(6), q(3), q(4), q(5));

    // pinocchio::FrameIndex base_id = 1;
    // MatrixXd oRb = robot_model.get_data().oMi[base_id].rotation();

    A.leftCols(nv) = Jb.bottomRows(3);

    b =   kd_b_ang.asDiagonal() * (omega_des - v.segment(3, 3))        // omega in in body frame with Pinocchio, I want it is inertial frame
        + kp_b_ang.asDiagonal() * (pinocchio::log3(quat_des.toRotationMatrix() * quat.toRotationMatrix().transpose()))
        - Jb_dot_times_v.bottomRows(3);
}


/* ======================== Task_swing_feet_tracking ======================== */

void ControlTasks::task_swing_feet_tracking(
    Ref<MatrixXd> A, Ref<VectorXd> b,
    const VectorXd& r_s_ddot_des, const VectorXd& r_s_dot_des, const VectorXd& r_s_des
) {
    MatrixXd Js = MatrixXd::Zero(4*3-nF, nv);
    robot_model.get_Js(Js);

    VectorXd Js_dot_times_v = VectorXd::Zero(4*3-nF);
    robot_model.get_Js_dot_times_v(Js_dot_times_v);

    VectorXd r_s = VectorXd::Zero(4*3-nF);
    robot_model.get_r_s(r_s);

    A.leftCols(nv) = Js;

    b =   r_s_ddot_des 
        + tile(kd_s_pos, 4-nc).asDiagonal() * (r_s_dot_des - Js * v)
        + tile(kp_s_pos, 4-nc).asDiagonal() * (r_s_des - r_s)
        - Js_dot_times_v;
}


/* ======================== Task_contact_constraints ======================== */

void ControlTasks::task_contact_constraints_soft_kv(
    Ref<MatrixXd> A, Ref<VectorXd> b,
    Ref<MatrixXd> C, const Ref<const VectorXd>& /*d*/,
    const VectorXd& d_k1, const VectorXd& d_k2
) {
    // Fc = Kp d + Kd d_dot

    // A = [  0, I, - Kp - Kd/dt ]                      soft contact constraint 
    //     [ Jc, 0, I/dt^2       ]                      - deformation_ddot = contact_point_acceleration

    // b = [ - Kd d_k1 / dt ]                           soft contact constraint 
    //     [ - Jc_dot * v + 2 d_k1/dt^2 - d_k2/dt^2 ]   - deformation_ddot = contact_point_acceleration

    MatrixXd Kp = tile(kp_terr, nc).asDiagonal();
    MatrixXd Kd = tile(kd_terr, nc).asDiagonal();
    MatrixXd Kc_v = tile(kc_v, nc).asDiagonal();

    A.block( 0,    nv, nF, nF) = MatrixXd::Identity(nF, nF);
    A.block( 0, nv+nF, nd, nd) = - Kp - Kd / dt;
    A.bottomLeftCorner(nF, nv) = Jc;
    A.block(nF, nv+nF, nd, nd) = MatrixXd::Identity(nd, nd) / (dt*dt);

    VectorXd Jc_dot_times_v = VectorXd::Zero(nF);
    robot_model.get_Jc_dot_times_v(Jc_dot_times_v);

    b.head(nF) = - Kd * d_k1 / dt;
    b.tail(nF) = - Jc_dot_times_v + 2 * d_k1 / (dt*dt) - d_k2 / (dt*dt) - Kc_v * Jc * v;


    // C = [ ... ]   ∈ 2*nc x (nv+nF+nd)
    // d = [ ... ]

    MatrixXd C_temp = MatrixXd::Zero(nc, nd);
    for (int i=0; i<nc; i++) {
        C_temp(i, 2+3*i) = 1;
    }

    C.block( 0, nv+nF, nc, nd) = - C_temp;
    C.block(nc,    nv, nc, nF) = - C_temp;
}


/* ======================== Task_joint_singularities ======================== */

void ControlTasks::task_joint_singularities(Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d)
{
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

    // C = [ - diag(knee_joint_sign) ]

    // d = [ 2/dt^2 * (q_knees + v_knees * dt) * diag(knee_joint_sign) ]

    C.setZero();
    for (int i=0; i<4; i++) {
        C(i, 6 + 2+3*i) = - knee_joint_sign[i];

        d(i) = 2/(dt*dt) * (q(7 + 2+3*i) + v(6 + 2+3*i) * dt) * knee_joint_sign[i];
    }

}


/* ===================== task_contact_constraints_rigid ===================== */

void ControlTasks::task_contact_constraints_rigid(Ref<MatrixXd> A, Ref<VectorXd> b)
{
    // A = [ Jc, 0 ]
    // b = [ - Jc_dot_times_v ]

    MatrixXd Kc_v = tile(kc_v, nc).asDiagonal();

    VectorXd Jc_dot_times_v = VectorXd::Zero(nF);
    robot_model.get_Jc_dot_times_v(Jc_dot_times_v);

    A.leftCols(nv) = Jc;

    b = - Jc_dot_times_v - Kc_v * Jc * v;
}


/* -------------------- task_contact_constraints_soft_hc -------------------- */

// Eigen::VectorXi sign(const Eigen::VectorXd& v)
// {
//     Eigen::VectorXi v_sign = Eigen::VectorXd::Ones(v.size());

//     for (i=0; i<v.size(), i++) {
//         if (v(i) < 0) {
//             v(i) = -1;
//         }
//     }

//     return v_sign;
// }

// void ControlTasks::task_contact_constraints_soft_hc(
//     Ref<MatrixXd> A, Ref<VectorXd> b,
//     Ref<MatrixXd> C, Ref<VectorXd> d,
//     const VectorXd& d_k1, const VectorXd& d_k2
// ) {
//     // Fc = Kp @ d^n + Kd @ (d^m * d_dot)

//     double n = 1.5;
//     double m = 0.5;

//     Eigen::VectorXd d_k1_unsg = d_k1 * sign(d_k1);

//     A.block( 0,    nv, nF, nF) = MatrixXd::Identity(nF, nF);
//     A.block( 0, nv+nF, nd, nd) = - Kp * d_k1_unsg.pow(n-1) - Kd * d_k1_unsg.pow(m) / dt;
//     A.bottomLeftCorner(nF, nv) = Jc;
//     A.block(nF, nv+nF, nd, nd) = MatrixXd::Identity(nd, nd) / (dt*dt);

//     VectorXd Jc_dot_times_v = VectorXd::Zero(nF);
//     robot_model.get_Jc_dot_times_v(Jc_dot_times_v);

//     b.head(nF) = - Kd * (d_k1) / dt;
//     b.tail(nF) = - Jc_dot_times_v + 2 * d_k1 / (dt*dt) - d_k2 / (dt*dt);


//     // c = [ ... ]   ∈ 2*nF x (nv+nF+nd)
//     // d = [ ... ]

//     VectorXd ma_che_fai_scemo(3);
//     ma_che_fai_scemo << 0, 0, 1;
//     MatrixXd C_temp = tile(ma_che_fai_scemo, nc).asDiagonal();

//     C.block( 0, nv+nF, nd, nd) = - C_temp;
//     C.block(nd, nv+nF, nd, nd) = (Kp - Kd/dt) * C_temp;

//     d.tail(nd) = C_temp * Kd * d_k1 / dt;
// }


/* -------------------- task_contact_constraints_soft_sim ------------------- */

void ControlTasks::task_contact_constraints_soft_sim(
    Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
    Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d,
    const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2)
{
    // Fc_z = Kp d + Kd d_dot

    // A = [  0, I, - Kp - Kd/dt ]                      soft contact constraint 
    //     [ Jc, 0, I/dt^2       ]                      - deformation_ddot = contact_point_acceleration

    // b = [ - Kd d_k1 / dt ]                           soft contact constraint 
    //     [ - Jc_dot * v + 2 d_k1/dt^2 - d_k2/dt^2 ]   - deformation_ddot = contact_point_acceleration

    MatrixXd C_temp = MatrixXd::Zero(nd, nF);
    for (int i=0; i<nc; i++) {
        C_temp(i, 2+3*i) = 1;
    }

    MatrixXd Kp = kp_terr(2) * MatrixXd::Identity(nc, nc);
    MatrixXd Kd = kd_terr(2) * MatrixXd::Identity(nc, nc);
    MatrixXd Kc_v = tile(kc_v, nc).asDiagonal();

    A.block( 0,    nv, nd, nF) = C_temp;
    A.block( 0, nv+nF, nd, nd) = - Kp - Kd / dt;
    A.bottomLeftCorner(nF, nv) = Jc;
    A.block(nd, nv+nF, nF, nd) = C_temp.transpose() / (dt*dt);

    VectorXd Jc_dot_times_v = VectorXd::Zero(nF);
    robot_model.get_Jc_dot_times_v(Jc_dot_times_v);

    b.head(nd) = - Kd * d_k1 / dt;
    b.tail(nF) = - Jc_dot_times_v + C_temp.transpose() * (2 * d_k1 / (dt*dt) - d_k2 / (dt*dt)) - Kc_v * Jc * v;


    // c = [ ... ]   ∈ 2*nc x (nv+nF+nd)
    // d = [ ... ]

    C.block( 0, nv+nF, nc, nd) = - MatrixXd::Identity(nc, nd);
    // C.block(nc, nv   , nc, nF) = - C_temp;

    // To avoid unused parameter warning.
    d.head(2*nc).setZero();
}


/* ===================== Task_energy_forces_optimization ==================== */

void ControlTasks::task_energy_forces_minimization(Ref<MatrixXd> A, Ref<VectorXd> b) const
{
    //     [ M_a, -Jc_a.T, 0 ]      joint torques minimization
    // A = [   0,       I, 0 ]      contact forces minimization
    //     [   0,       0, I ]      feet deformations minimization
    //
    //     [ - h_a ]
    // b = [   0   ]
    //     [   0   ]

    A.topLeftCorner(nv-6, nv) = M.bottomRows(nv-6);
    A.block(0, nv, nv-6, nF) = - Jc.rightCols(nv-6).transpose();
    A.block(nv-6, nv, nF, nF) = MatrixXd::Identity(nF, nF);
    A.bottomRightCorner(nd, nd) = MatrixXd::Identity(nd, nd);

    b.topRows(nv-6) = - h.bottomRows(nv-6);
}


} // namespace wbc