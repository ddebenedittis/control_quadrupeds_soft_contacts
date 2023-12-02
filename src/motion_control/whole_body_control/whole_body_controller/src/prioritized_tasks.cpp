#include "whole_body_controller/prioritized_tasks.hpp"



namespace wbc {

PrioritizedTasks::PrioritizedTasks(const std::string& robot_name, float dt)
: control_tasks(robot_name, dt)
{
    compute_prioritized_tasks_vector();
}

void PrioritizedTasks::reset(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const std::vector<std::string>& contact_feet_names)
{
    control_tasks.reset(q, v, contact_feet_names, this->contact_constraint_type);
}

void PrioritizedTasks::compute_task_p(
    int priority,
    Eigen::MatrixXd& A, Eigen::VectorXd& b,
    Eigen::MatrixXd& C, Eigen::VectorXd& d,
    const GeneralizedPose& gen_pose,
    const Eigen::VectorXd& d_k1, const Eigen::VectorXd& d_k2
) {
    std::pair<int,int> task_rows = get_prioritized_task_dimension(priority);

    int cols = control_tasks.get_nv() + control_tasks.get_nF() + control_tasks.get_nd();

    A.resize(task_rows.first, cols);
    A.setZero();
    b.resize(task_rows.first);
    b.setZero();

    C.resize(task_rows.second, cols);
    C.setZero();
    d.resize(task_rows.second);
    d.setZero();

    int ne = 0;
    int ni = 0;

    int ne_temp = 0;
    int ni_temp = 0;

    for (int i = 0; i < static_cast<int>(tasks_vector.size()); i++) {
        if (tasks_vector[i] == priority) {
                        auto n_temp = get_task_dimension(static_cast<TasksNames>(i));
            ne_temp = n_temp.first;     // number of rows of the equality control task
            ni_temp = n_temp.second;    // number of rows of the inequality control task

            switch (static_cast<TasksNames>(i))
            {
            case TasksNames::FloatingBaseEOM:
                                control_tasks.task_floating_base_eom(A.middleRows(ne, ne_temp), b.segment(ne, ne_temp));
                break;
            case TasksNames::TorqueLimits:
                                control_tasks.task_torque_limits(C.middleRows(ni, ni_temp), d.segment(ni, ni_temp));
                break;
            case TasksNames::FrictionAndFcModulation:
                                control_tasks.task_friction_Fc_modulation(C.middleRows(ni, ni_temp), d.segment(ni, ni_temp));
                break;
            case TasksNames::LinearBaseMotionTracking:
                                control_tasks.task_linear_motion_tracking(
                    A.middleRows(ne, ne_temp), b.segment(ne, ne_temp),
                    gen_pose.base_acc, gen_pose.base_vel, gen_pose.base_pos
                );
                break;
            case TasksNames::AngularBaseMotionTracking:
                                control_tasks.task_angular_motion_tracking(
                    A.middleRows(ne, ne_temp), b.segment(ne, ne_temp),
                    gen_pose.base_angvel, gen_pose.base_quat
                );
                break;
            case TasksNames::SwingFeetMotionTracking:
                                control_tasks.task_swing_feet_tracking(
                    A.middleRows(ne, ne_temp), b.segment(ne, ne_temp),
                    gen_pose.feet_acc, gen_pose.feet_vel, gen_pose.feet_pos
                );
                break;
            case TasksNames::ContactConstraints:
                                if (contact_constraint_type == ContactConstraintType::soft_kv) {
                    control_tasks.task_contact_constraints_soft_kv(
                        A.middleRows(ne, ne_temp), b.segment(ne, ne_temp),
                        C.middleRows(ni, ni_temp), d.segment(ni, ni_temp),
                        d_k1, d_k2
                    );
                } else if (contact_constraint_type == ContactConstraintType::soft_sim) {
                    control_tasks.task_contact_constraints_soft_sim(
                        A.middleRows(ne, ne_temp), b.segment(ne, ne_temp),
                        C.middleRows(ni, ni_temp), d.segment(ni, ni_temp),
                        d_k1, d_k2
                    );
                } else if (contact_constraint_type == ContactConstraintType::rigid) {
                    control_tasks.task_contact_constraints_rigid(
                        A.middleRows(ne, ne_temp), b.segment(ne, ne_temp)
                    );
                }

                break;
            case TasksNames::JointSingularities:
                                control_tasks.task_joint_singularities(C.middleRows(ni, ni_temp), d.segment(ni, ni_temp));
                break;
            case TasksNames::EnergyAndForcesOptimization:
                                control_tasks.task_energy_forces_minimization(A.middleRows(ne, ne_temp), b.segment(ne, ne_temp));
                break;
            case TasksNames::SEPARATOR:
                break;
            }

            ne += ne_temp;
            ni += ni_temp;
        }
    }
}

std::pair<int,int> PrioritizedTasks::get_task_dimension(TasksNames task_name)
{
    int ne = 0;
    int ni = 0;

    switch (task_name)
    {
    case TasksNames::FloatingBaseEOM:
        ne = 6;
        break;
    case TasksNames::TorqueLimits:
        ni = 2 * (control_tasks.get_nv() - 6);
        break;
    case TasksNames::FrictionAndFcModulation:
        ni = 6 * control_tasks.get_nc();
        break;
    case TasksNames::LinearBaseMotionTracking:
        ne = 3;
        break;
    case TasksNames::AngularBaseMotionTracking:
        ne = 3;
        break;
    case TasksNames::SwingFeetMotionTracking:
        ne = 12 - 3 * control_tasks.get_nc();
        break;
    case TasksNames::ContactConstraints:
        if (contact_constraint_type == ContactConstraintType::soft_kv) {
            ne = 2 * control_tasks.get_nF();
            ni = 2 * control_tasks.get_nF();
        } else if (contact_constraint_type == ContactConstraintType::soft_sim) {
            ne = control_tasks.get_nc() + control_tasks.get_nF();
            ni = 2 * control_tasks.get_nc();
        } else if (contact_constraint_type == ContactConstraintType::rigid) {
            ne = control_tasks.get_nF();
        }
        break;
    case TasksNames::JointSingularities:
        ni = 4;
        break;
    case TasksNames::EnergyAndForcesOptimization:
        ne = control_tasks.get_nv() - 6 + control_tasks.get_nF() + control_tasks.get_nd();
        break;
    case TasksNames::SEPARATOR:
        break;
    }

    return std::make_pair(ne, ni);
}

std::pair<int,int> PrioritizedTasks::get_prioritized_task_dimension(int priority)
{
    int ne = 0;
    int ni = 0;

    for (int i = 0; i < static_cast<int>(tasks_vector.size()); i++) {
        if (tasks_vector[i] == priority) {
            std::pair<int,int> n_temp = get_task_dimension(static_cast<TasksNames>(i));
            ne += n_temp.first;
            ni += n_temp.second;
        }
    }

    return std::make_pair(ne, ni);
}

void PrioritizedTasks::compute_prioritized_tasks_vector()
{
    int count = static_cast<int>(TasksNames::SEPARATOR);
    tasks_vector.assign(count, -1);

    {
        int priority = 0;

        for (TasksNames name : prioritized_tasks_list) {
            if (name == TasksNames::SEPARATOR) {
                priority++;
            } else {
                tasks_vector[static_cast<int>(name)] = priority;
            }
        }
    }
}

} // namespace wbc