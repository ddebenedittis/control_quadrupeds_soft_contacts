#include "whole_body_controller/mpc_prioritized_tasks.hpp"
#include "whole_body_controller/prioritized_tasks.hpp"
#include <cmath>



namespace wbc {

void MPCPrioritizedTasks::reset(const std::vector<std::string>& contact_feet_names)
{
    control_tasks.reset(contact_feet_names);
}

void MPCPrioritizedTasks::compute_task_p(
    int priority,
    Eigen::MatrixXd& A, Eigen::VectorXd& b,
    Eigen::MatrixXd& C, Eigen::VectorXd& d,
    const std::vector<GeneralizedPose>& gen_poses,
    const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2
) {
    std::vector<int> eq_rows(n_horizon_ + 1);
    std::vector<int> ineq_rows(n_horizon_ + 1);
    std::vector<int> cols(n_horizon_ + 1);

    const int n_state =   control_tasks.get_nv() + 1     // q
                        + control_tasks.get_nv();        // v

    eq_rows[0] = 0;
    ineq_rows[0] = 0;
    cols[0] = 0;

    for (int i = 0; i < n_horizon_; i++) {
        reset(gen_poses[i].contact_feet_names);

        std::pair<int,int> tasks_rows = get_prioritized_task_dimension(priority);

        eq_rows[i+1] = eq_rows[i] + tasks_rows.first;
        ineq_rows[i+1] = ineq_rows[i] + tasks_rows.second;

        const int n_x = n_state + control_tasks.get_nv() + control_tasks.get_nF() + control_tasks.get_nd();

        cols[i+1] = cols[i] + n_x;
    }

    A.resize(eq_rows.back(), cols.back());
    A.setZero();
    b.resize(eq_rows.back());
    b.setZero();

    C.resize(ineq_rows.back(), cols.back());
    C.setZero();
    d.resize(ineq_rows.back());
    d.setZero();

    for (int i = 0; i < n_horizon_; i++) {
        compute_task_p_non_res(
            priority,
            A, b,
            C, d,
            gen_poses[i],
            d_k1, d_k2,
            i,
            eq_rows, ineq_rows, cols
        );
    }
}

void MPCPrioritizedTasks::compute_task_p_non_res(
    int priority,
    Eigen::Ref<Eigen::MatrixXd> A, Eigen::Ref<Eigen::VectorXd> b,
    Eigen::Ref<Eigen::MatrixXd> C, Eigen::Ref<Eigen::VectorXd> d,
    const GeneralizedPose& gen_pose,
    const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2,
    int step_mpc,
    const std::vector<int>& eq_rows, const std::vector<int>& ineq_rows, const std::vector<int>& cols
) {
    int ne = 0;
    int ni = 0;

    int ne_temp = 0;
    int ni_temp = 0;

    const int n_state =   control_tasks.get_nv() + 1     // q
                        + control_tasks.get_nv();        // v

    for (int i = 0; i < static_cast<int>(tasks_vector.size()); i++) {
        if (tasks_vector[i] == priority) {
            auto n_temp = get_task_dimension(static_cast<TasksNames>(i));
            ne_temp = n_temp.first;     // number of rows of the equality control task
            ni_temp = n_temp.second;    // number of rows of the inequality control task

            int ie_row = eq_rows[step_mpc];
            int ii_row = ineq_rows[step_mpc];

            int i_col = 0;
            int n_col = 0;
            if (static_cast<TasksNames>(i) != TasksNames::MPCDynamics
                && static_cast<TasksNames>(i) != TasksNames::LinearBaseMotionTracking
                && static_cast<TasksNames>(i) != TasksNames::AngularBaseMotionTracking
                && static_cast<TasksNames>(i) != TasksNames::SwingFeetMotionTracking
                && static_cast<TasksNames>(i) != TasksNames::JointSingularities
            ) {
                i_col = cols[step_mpc] + n_state;
                n_col = cols[step_mpc + 1] - cols[step_mpc] - n_state; // n_u
            } else if (static_cast<TasksNames>(i) == TasksNames::JointSingularities) {
                i_col = cols[step_mpc];
                n_col = control_tasks.get_nv() + 1; // n_q
            } else if (step_mpc == 0) {
                i_col = cols[step_mpc] + n_state;
                n_col = cols[step_mpc + 1] - cols[step_mpc] - n_state; // n_u
            } else {
                i_col = cols[step_mpc - 1];
                n_col = cols[step_mpc + 1] - cols[step_mpc - 1]; // 2 n_x
            }

            switch (static_cast<TasksNames>(i))
            {
            case TasksNames::FloatingBaseEOM:
                control_tasks.task_floating_base_eom(
                    A.block(ie_row + ne, i_col, ne_temp, n_col),
                    b.segment(ie_row + ne, ne_temp)
                );
                break;
            case TasksNames::MPCDynamics:
                control_tasks.task_mpc_dynamics(A.middleRows(ne, ne_temp), b.segment(ne, ne_temp), step_mpc);
                break;
            case TasksNames::TorqueLimits:
                control_tasks.task_torque_limits(
                    C.block(ii_row + ni, i_col, ni_temp, n_col),
                    d.segment(ii_row + ni, ni_temp)
                );
                break;
            case TasksNames::FrictionAndFcModulation:
                control_tasks.task_friction_Fc_modulation(
                    C.block(ii_row + ni, i_col, ni_temp, n_col),
                    d.segment(ii_row + ni, ni_temp)
                );
                break;
            case TasksNames::LinearBaseMotionTracking:
                control_tasks.task_linear_motion_tracking(
                    A.block(ie_row + ne, i_col, ne_temp, n_col),
                    b.segment(ie_row + ne, ne_temp),
                    gen_pose.base_acc, gen_pose.base_vel, gen_pose.base_pos,
                    step_mpc
                );
                break;
            case TasksNames::AngularBaseMotionTracking:
                control_tasks.task_angular_motion_tracking(
                    A.block(ie_row + ne, i_col, ne_temp, n_col),
                    b.segment(ie_row + ne, ne_temp),
                    gen_pose.base_angvel, gen_pose.base_quat,
                    step_mpc
                );
                break;
            case TasksNames::SwingFeetMotionTracking:
                control_tasks.task_swing_feet_tracking(
                    A.block(ie_row + ne, i_col, ne_temp, n_col),
                    b.segment(ie_row + ne, ne_temp),
                    gen_pose.feet_acc, gen_pose.feet_vel, gen_pose.feet_pos,
                    step_mpc
                );
                break;
            case TasksNames::ContactConstraints:
                if (contact_constraint_type == ContactConstraintType::soft_kv) {
                    control_tasks.task_contact_constraints_soft_kv(
                        A.block(ie_row + ne, i_col, ne_temp, n_col),
                        b.segment(ie_row + ne, ne_temp),
                        C.block(ii_row + ni, i_col, ni_temp, n_col),
                        d.segment(ii_row + ni, ni_temp),
                        d_k1, d_k2
                    );
                } else if (contact_constraint_type == ContactConstraintType::soft_sim) {
                    control_tasks.task_contact_constraints_soft_sim(
                        A.block(ie_row + ne, i_col, ne_temp, n_col),
                        b.segment(ie_row + ne, ne_temp),
                        C.block(ii_row + ni, i_col, ni_temp, n_col),
                        d.segment(ii_row + ni, ni_temp),
                        d_k1, d_k2
                    );
                } else if (contact_constraint_type == ContactConstraintType::rigid) {
                    control_tasks.task_contact_constraints_rigid(
                        A.block(ie_row + ne, i_col, ne_temp, n_col),
                        b.segment(ie_row + ne, ne_temp)
                    );
                }

                break;
            case TasksNames::JointSingularities:
                control_tasks.task_joint_singularities(
                    C.block(ii_row + ni, i_col, ni_temp, n_col),
                    d.segment(ii_row + ni, ni_temp)
                );
                break;
            case TasksNames::EnergyAndForcesOptimization:
                control_tasks.task_energy_forces_minimization(
                    A.block(ie_row + ne, i_col, ne_temp, n_col),
                    b.segment(ie_row + ne, ne_temp)
                );
                break;
            case TasksNames::SEPARATOR:
                break;
            }

            ne += ne_temp;
            ni += ni_temp;
        }
    }

    // std::cout << "MPC A: \n" << A.rightCols(A.cols() - 37) << "\n" << std::endl;
    // std::cout << "MPC b: \n" << b << "\n" << std::endl;
    // std::cout << "MPC C: \n" << C.rightCols(C.cols() - 37) << "\n" << std::endl;
    // std::cout << "MPC d: \n" << d << "\n" << std::endl;
}

} // namespace wbc