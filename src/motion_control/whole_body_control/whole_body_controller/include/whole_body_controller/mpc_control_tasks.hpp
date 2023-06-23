#pragma once

#include "whole_body_controller/control_tasks.hpp"
#include <Eigen/src/Core/Matrix.h>



namespace wbc {

class MPCControlTasks: public ControlTasks {
public:
    MPCControlTasks(const std::string& robot_name, float dt, float dt_mpc = 0)
    : ControlTasks(robot_name, dt),
      dt_mpc_(dt_mpc)
    {}

    using ControlTasks::reset;
    void reset(const std::vector<std::string>& contact_feet_names);

    void task_mpc_dynamics(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        int i
    ) {
        if (i == 0) {
            task_first_mpc_dynamics(A, b);
        } else {
            task_later_mpc_dynamics(A);
        }
    }

    /// @brief Compute A and b that enforce the tracking of the reference base linear trajectory.
    /// @param[out] A
    /// @param[out] b
    /// @param[in]  r_b_ddot_des
    /// @param[in]  r_b_dot_des
    /// @param[in]  r_b_des
    void task_linear_motion_tracking(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        const Eigen::Vector3d& r_b_ddot_des, const Eigen::Vector3d& r_b_dot_des, const Eigen::Vector3d& r_b_des,
        int i = 0
    ) override;

    /// @brief Compute A and b that enforce the tracking of the reference base angular motion.
    /// @param[out] A 
    /// @param[out] b 
    /// @param[in]  omega_des 
    /// @param[in]  q_des 
    void task_angular_motion_tracking(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        const Eigen::Vector3d& omega_des, const Eigen::Vector4d& q_des,
        int i = 0
    ) const override;

    /// @brief Compute A and b that enforce the trajectory tracking of the reference swing feet motion.
    /// @param[out] A 
    /// @param[out] b 
    /// @param[in]  r_s_ddot_des 
    /// @param[in]  r_s_dot_des 
    /// @param[in]  r_s_des 
    void task_swing_feet_tracking(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        const Eigen::VectorXd& r_s_ddot_des, const Eigen::VectorXd& r_s_dot_des, const Eigen::VectorXd& r_s_des,
        int i = 0
    ) override;

    /// @brief Enforce the non-singularity condition of the legs, by imposing that the knee joint does not change sign.
    /// @param[out] C 
    /// @param[out] d 
    void task_joint_singularities(
        Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> /*d*/
    ) override;

private:
    void compute_T();

    /// @brief MPC dynamics at the first timestep. The previous joint coordinates and velocities are measured.
    void task_first_mpc_dynamics(
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b
    );
    
    /// @brief MPC dynamics at the timesteps after the first. The previous joint coordinates and velocities are not measured.
    void task_later_mpc_dynamics(
        Eigen::Ref<Eigen::MatrixXd> A
    ) const;

    /// @brief q_dot = T_ * v
    Eigen::MatrixXd T_;
    /// @brief v = T_^+ * q_dot
    Eigen::MatrixXd T_pinv_;

    Eigen::MatrixXd Js;

    float dt_mpc_;
};

} // namespace wbc