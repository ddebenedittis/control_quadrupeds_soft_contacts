#pragma once

#include "robot_model/robot_model.hpp"



namespace wbc {

/* ========================================================================== */
/*                             CONSTRAINTTYPE ENUM                            */
/* ========================================================================== */

enum class ContactConstraintType {soft_kv, soft_sim, rigid, invalid};



/// @class @brief Implements all the tasks that are used for the the hierarchical optimization problem.
/// @details The ControlTasks class provides methods to compute the matrices A, b, C, d that define the tasks used in the control problem. These tasks are specified in no specific order.
/// This class is intended to be used with prioritized_tasks. This second class organizes the various control tasks by merging them in a single task of a certain priority. The priority order of the control tasks can be specified by means of the attribute prioritized_tasks_list.
class ControlTasks {
public:
    ControlTasks(const std::string& robot_name, float dt);

    /// @brief Reset the class before restarting the optimization problem.
    /// @param[in] q
    /// @param[in] v
    /// @param[in] contact_feet_names
    void reset(
        const Eigen::VectorXd& q, const Eigen::VectorXd& v,
        const std::vector<std::string>& contact_feet_names, ContactConstraintType contact_constraint_type);

    /// @brief Compute the matrices A and b that enforce the dynamic consistency with the Equations of Motion of the floating base.
    /// @param[out] A
    /// @param[out] b
    void task_floating_base_eom(Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b);

    /// @brief Compute the matrices C and d that enfore the limits on the joint torques.
    /// @param[out] C
    /// @param[out] d
    void task_torque_limits(Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d);

    /// @brief Compute C and d that enforce the friction limits and the limits on the normal component of the contact forces.
    /// @param[out] C
    /// @param[out] d
    void task_friction_Fc_modulation(Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d) const;

    /// @brief Compute A and b that enforce the tracking of the reference base linear trajectory.
    /// @param[out] A
    /// @param[out] b
    /// @param[in]  r_b_ddot_des
    /// @param[in]  r_b_dot_des
    /// @param[in]  r_b_des
    void task_linear_motion_tracking(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        const Eigen::Vector3d& r_b_ddot_des, const Eigen::Vector3d& r_b_dot_des, const Eigen::Vector3d& r_b_des
    );

    /// @brief Compute A and b that enforce the tracking of the reference base angular motion.
    /// @param[out] A 
    /// @param[out] b 
    /// @param[in]  omega_des 
    /// @param[in]  q_des 
    void task_angular_motion_tracking(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        const Eigen::Vector3d& omega_des, const Eigen::Vector4d& q_des
    ) const;

    /// @brief Compute A and b that enforce the trajectory tracking of the reference swing feet motion.
    /// @param[out] A 
    /// @param[out] b 
    /// @param[in]  r_s_ddot_des 
    /// @param[in]  r_s_dot_des 
    /// @param[in]  r_s_des 
    void task_swing_feet_tracking(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        const Eigen::VectorXd& r_s_ddot_des, const Eigen::VectorXd& r_s_dot_des, const Eigen::VectorXd& r_s_des
    );

    /// @brief Compute A, b, C, d that enforce the soft contact constraints, assuming a Kelving-Voight soft contact model (linear springs and linear dampers).
    /// @param[out] A 
    /// @param[out] b 
    /// @param[out] C 
    /// @param[out] d 
    /// @param[in]  d_k1 Deformations of the feet in contact with the terrain at the previous time step.
    /// @param[in]  d_k2 Deformations of the feet in contact with the terrain at the second previous time step.
    void task_contact_constraints_soft_kv(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> /*d*/,
        const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2
    );

    /// @brief Compute A and b that enforce the rigid contact constraints.
    /// @param[out] A 
    /// @param[out] b 
    void task_contact_constraints_rigid(Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b);
    
    /// @brief Compute A, b, C, d that enforce the soft contact constraints, assuming a Hunt-Crossley soft contact model (nonlinear springs and nonlinear dampers).
    /// @param[out] A
    /// @param[out] b
    /// @param[out] C
    /// @param[out] d
    /// @param[in]  d_k1 Deformations of the feet in contact with the terrain at the previous time step.
    /// @param[in]  d_k2 Deformations of the feet in contact with the terrain at the second previous time step.
    // void task_contact_constraints_soft_hc(
    //     Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
    //     Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d,
    //     const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2
    // );

    /// @brief Compute A, b, C, d that enforce the soft contact constraints, assuming a Kelving-Voight soft contact model only in the normal (z) direction.
    /// @param[out] A 
    /// @param[out] b 
    /// @param[out] C 
    /// @param[out] d 
    /// @param[in]  d_k1 
    /// @param[in]  d_k2 
    void task_contact_constraints_soft_sim(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d,
        const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2
    );

    /// @brief Enforce the non-singularity condition of the legs, by imposing that the knee joint does not change sign.
    /// @param[out] C 
    /// @param[out] d 
    void task_joint_singularities(Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d);

    /// @brief Compute A and b that enforce the minimization of the energy (the square of the torques) and of the contact forces.
    /// @param[out] A 
    /// @param[out] b 
    void task_energy_forces_minimization(Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b) const;


    /* =============================== Getters ============================== */

    int get_nv() const {return nv;}
    int get_nc() const {return nc;}
    int get_nF() const {return nF;}
    int get_nd() const {return nd;}

    const pinocchio::Model& get_model() const { return robot_model.get_model(); }
    
    const pinocchio::Data& get_data() { return robot_model.get_data(); }

    double get_mass() const { return robot_model.get_mass(); }

    double get_friction_coefficient() const { return mu; }

    Eigen::Vector3d get_com_position() {return pinocchio::centerOfMass(get_model(), robot_model.get_data(), false);}

    const Eigen::MatrixXd& get_M()  const { return M; }
    const Eigen::VectorXd& get_h()  const { return h; }
    const Eigen::MatrixXd& get_Jc() const { return Jc; }

    Eigen::VectorXd get_feet_positions() const{ return robot_model.get_feet_positions(); }

    Eigen::VectorXd get_feet_velocities(const Eigen::VectorXd& v) { return robot_model.get_feet_velocities(v); }

    /// @brief Return the generic feet names. The generic feet names are the same for all quadrupedal robots: LF, RF, LH, and RH.
    const std::vector<std::string>& get_generic_feet_names() const {return robot_model.get_generic_feet_names();}

    /// @brief Return the feet names of the specific robot. These are the names of the links used for computing the contact point.
    const std::vector<std::string>& get_all_feet_names() const {return robot_model.get_all_feet_names();}

    const Eigen::Vector3d& get_kp_terr() const {return this->kp_terr;}


    /* =============================== Setters ============================== */

    void set_tau_max(const double tau_max) {this->tau_max = tau_max;}
    void set_mu(const double mu) {this->mu = mu;}
    void set_Fn_max(const double Fn_max) {this->Fn_max = Fn_max;}
    void set_Fn_min(const double Fn_min) {this->Fn_min = Fn_min;}

    void set_kp_b_pos(const Eigen::Ref<const Eigen::Vector3d>& kp_b_pos) {this->kp_b_pos = kp_b_pos;}
    void set_kd_b_pos(const Eigen::Ref<const Eigen::Vector3d>& kd_b_pos) {this->kd_b_pos = kd_b_pos;}

    void set_kp_b_ang(const Eigen::Ref<const Eigen::Vector3d>& kp_b_ang) {this->kp_b_ang = kp_b_ang;}
    void set_kd_b_ang(const Eigen::Ref<const Eigen::Vector3d>& kd_b_ang) {this->kd_b_ang = kd_b_ang;}

    void set_kp_s_pos(const Eigen::Ref<const Eigen::Vector3d>& kp_s_pos) {this->kp_s_pos = kp_s_pos;}
    void set_kd_s_pos(const Eigen::Ref<const Eigen::Vector3d>& kd_s_pos) {this->kd_s_pos = kd_s_pos;}

    void set_kp_terr(const Eigen::Ref<const Eigen::Vector3d>& kp_terr) {this->kp_terr = kp_terr;}
    void set_kd_terr(const Eigen::Ref<const Eigen::Vector3d>& kd_terr) {this->kd_terr = kd_terr;}

    void set_kc_v(const Eigen::Ref<const Eigen::Vector3d>& kc_v) {this->kc_v = kc_v;}

private:
    robot_wrapper::RobotModel robot_model;

    int nv = 18;    ///< @brief Dimension of the generalized velocity vector
    int nc =  0;    ///< @brief Number of feet in contact with the terrain
    int nF =  0;    ///< @brief Dimension of the stack of the contact forces with the terrain (= 3*nc)
    int nd =  0;    ///< @brief Dimension of the stack of the desired feet deformations (= 3*nc iff a soft contact model is used)

    Eigen::VectorXd q;      ///< @brief Generalized coordinates vector
    Eigen::VectorXd v;      ///< @brief Generalized velocities vector

    float dt;   ///< @brief Time step between two controllers cycles.

    double tau_max = 80;    ///< @brief Maximum joint torques
    double mu = 0.8;        ///< @brief friction coefficient
    double Fn_max = 350;    ///< @brief Maximum normal contact force (this may be a function of the swing phase)
    double Fn_min = 40;     ///< @brief Minimum normal contact force (this may be a function of the swing phase)

    Eigen::MatrixXd M;                  ///< @brief Mass matrix
    Eigen::VectorXd h;                  ///< @brief Nonlinear terms vector
    Eigen::MatrixXd Jc;                 ///< @brief Stack of the contact points jacobians

    Eigen::MatrixXd Jb;                 ///< @brief Base jacobian
    Eigen::VectorXd Jb_dot_times_v;     ///< @brief Vector representing Jb_dot * v

    Eigen::Vector3d kp_b_pos = {1, 1, 1};       ///< @brief Controller gains on the position error of the base
    Eigen::Vector3d kd_b_pos = {1, 1, 1};       ///< @brief Controller gains on the velocity error of the base

    Eigen::Vector3d kp_b_ang = {1, 1, 1};       ///< @brief Controller gains on the orientation error of the base
    Eigen::Vector3d kd_b_ang = {1, 1, 1};       ///< @brief Controller gains on the angular velocity error of the base

    Eigen::Vector3d kp_s_pos = {1, 1, 1};       ///< @brief Controller gains on the position error of the swing feet
    Eigen::Vector3d kd_s_pos = {1, 1, 1};       ///< @brief Controller gains on the velocity error of the swing feet

    Eigen::Vector3d kp_terr = {1, 1, 1};        ///< @brief Stiffness of the soft foot in the three directions
    Eigen::Vector3d kd_terr = {1, 1, 1};        ///< @brief Damping of the soft foot in the three directions

    Eigen::Vector3d kc_v = {0, 0, 0};           ///< @brief Contact task: Jc * u_dot + Jc_dot * u = d_ddot - Kc_v * Jc * u, where Kc_v is the diagonal matrix whose diagonal is kc_v repeated nc number of times.

    /// @brief 
    Eigen::Vector4d knee_joint_sign = {0, 0, 0, 0};
};

} // namespace wbc