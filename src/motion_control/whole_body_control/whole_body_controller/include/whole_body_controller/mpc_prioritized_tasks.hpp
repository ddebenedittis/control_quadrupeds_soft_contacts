#pragma once

#include "hierarchical_optimization/hierarchical_qp.hpp"
#include "whole_body_controller/deformations_history_manager.hpp"
#include "whole_body_controller/mpc_control_tasks.hpp"
#include "whole_body_controller/prioritized_tasks.hpp"



namespace wbc {

/// @class @brief 
class MPCPrioritizedTasks: public PrioritizedTasks<MPCControlTasks> {
public:
    MPCPrioritizedTasks(const std::string& robot_name, float dt, int n_horizon = 1)
    : PrioritizedTasks(robot_name, dt),
      n_horizon_(n_horizon)
    {
        prioritized_tasks_list = {
            TasksNames::FloatingBaseEOM, TasksNames::MPCDynamics, TasksNames::ContactConstraints, TasksNames::SEPARATOR,
            TasksNames::JointSingularities, TasksNames::SEPARATOR,
            TasksNames::TorqueLimits, TasksNames::FrictionAndFcModulation, TasksNames::SEPARATOR,
            TasksNames::LinearBaseMotionTracking, TasksNames::AngularBaseMotionTracking, TasksNames::SwingFeetMotionTracking, TasksNames::SEPARATOR,
            TasksNames::EnergyAndForcesOptimization, TasksNames::SEPARATOR
        };

        compute_prioritized_tasks_vector();
    }

    using PrioritizedTasks<MPCControlTasks>::reset;
    void reset(const std::vector<std::string>& contact_feet_names);

    using PrioritizedTasks<MPCControlTasks>::compute_task_p;
    /// @brief Compute the matrices A, b, C, d that represents the task.
    /// @param[in] priority
    /// @param[out] A
    /// @param[out] b
    /// @param[out] C
    /// @param[out] d
    /// @param[in] gen_pose
    /// @param[in] d_k1
    /// @param[in] d_k2
    void compute_task_p(
        int priority,
        Eigen::MatrixXd& A, Eigen::VectorXd& b,
        Eigen::MatrixXd& C, Eigen::VectorXd& d,
        const std::vector<GeneralizedPose>& gen_poses,
        const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2
    ) override;

    void set_optimization_horizon(int n) {n_horizon_ = n;}

private:
    /// @brief Compute the matrices A, b, C, d that represents the task.
    /// @param[in] priority
    /// @param[out] A
    /// @param[out] b
    /// @param[out] C
    /// @param[out] d
    /// @param[in] gen_pose
    /// @param[in] d_k1
    /// @param[in] d_k2
    /// @param[in] step_mpc
    void compute_task_p_non_res(
        int priority,
        Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
        Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d,
        const GeneralizedPose& gen_pose,
        const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2,
        int step_mpc,
        const std::vector<int>& eq_rows, const std::vector<int>& ineq_rows, const std::vector<int>& cols
    );

    int n_horizon_ = 1;
};

} // namespace wbc