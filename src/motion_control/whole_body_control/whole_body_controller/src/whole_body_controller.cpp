#include "whole_body_controller/whole_body_controller.hpp"
#include "whole_body_controller/control_tasks.hpp"
#include "whole_body_controller/prioritized_tasks.hpp"



namespace wbc {


/* ========================================================================== */
/*                       WHOLEBODYCONTROLLER CONSTRUCTOR                      */
/* ========================================================================== */

template <typename PrioritizedTasksType>
WholeBodyController<PrioritizedTasksType>::WholeBodyController(const std::string& robot_name, float dt)
: prioritized_tasks_(robot_name, dt),
  hierarchical_qp_(prioritized_tasks_.get_max_priority()),
  x_opt_(Eigen::VectorXd::Zero(prioritized_tasks_.get_nv())),
  tau_opt_(Eigen::VectorXd::Zero(12)),
  f_c_opt_(Eigen::VectorXd::Zero(12)),
  d_des_opt_(Eigen::VectorXd::Zero(0))
{}


/* ========================================================================== */
/*                                    STEP                                    */
/* ========================================================================== */

template <typename PrioritizedTasksType>
void WholeBodyController<PrioritizedTasksType>::step(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const GeneralizedPose& gen_pose)
{
    std::pair<Eigen::VectorXd, Eigen::VectorXd> defs_pair;

    if (prioritized_tasks_.get_contact_constraint_type() != ContactConstraintType::rigid) {
        deformations_history_manager_.initialize_deformations_after_planning(gen_pose.contact_feet_names);

        defs_pair = deformations_history_manager_.get_deformations_history();
    } else {
        defs_pair = std::make_pair(Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0));
    }

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::MatrixXd C;
    Eigen::VectorXd d;
    Eigen::VectorXd we;
    Eigen::VectorXd wi;

    prioritized_tasks_.reset(q, v, gen_pose.contact_feet_names);

    prioritized_tasks_.compute_task_p(0, A, b, C, d, gen_pose, defs_pair.first, defs_pair.second);

    for (int i = 1; i <= prioritized_tasks_.get_max_priority(); i++) {
        we = Eigen::VectorXd::Ones(A.rows());
        wi = Eigen::VectorXd::Ones(C.rows());

        hierarchical_qp_.solve_qp(i-1, A, b, C, d, we, wi);

        prioritized_tasks_.compute_task_p(i, A, b, C, d, gen_pose, defs_pair.first, defs_pair.second);
    }

    we = Eigen::VectorXd::Ones(A.rows());
    wi = Eigen::VectorXd::Ones(C.rows());

    hierarchical_qp_.solve_qp(prioritized_tasks_.get_max_priority(), A, b, C, d, we, wi);

    x_opt_ = hierarchical_qp_.get_sol();

    const int nv = prioritized_tasks_.get_nv();
    const int nF = prioritized_tasks_.get_nF();
    const int nd = prioritized_tasks_.get_nd();

    Eigen::VectorXd f_c_opt_var = x_opt_.segment(nv, nF);
    Eigen::VectorXd d_des_opt_var = x_opt_.segment(nv + nF, nd);

    {
        auto generic_feet_names = get_generic_feet_names();
        std::string generic_foot_name;

        f_c_opt_.setZero();
        d_des_opt_.setZero();

        const int def_size = deformations_history_manager_.get_def_size();

        for (int i=0; i<static_cast<int>(gen_pose.contact_feet_names.size()); i++) {
            generic_foot_name = gen_pose.contact_feet_names[i];

            auto it = std::find(generic_feet_names.begin(), generic_feet_names.end(), generic_foot_name);

            const int index = std::distance(generic_feet_names.begin(), it);

            f_c_opt_.segment(3*index,3) = f_c_opt_var.segment(3*i, 3);
            
            if (prioritized_tasks_.get_contact_constraint_type() != ContactConstraintType::rigid) {
                d_des_opt_.segment(def_size*index, def_size) = d_des_opt_var.segment(def_size*i, def_size);
            }
        }
    }

    if (prioritized_tasks_.get_contact_constraint_type() != ContactConstraintType::rigid) {
        deformations_history_manager_.update_deformations_after_optimization(d_des_opt_var);
    }

    compute_torques();
}


/* ========================================================================== */
/*                               COMPUTE_TORQUES                              */
/* ========================================================================== */

template <typename PrioritizedTasksType>
void WholeBodyController<PrioritizedTasksType>::compute_torques()
{
    const int nv = prioritized_tasks_.get_nv();
    const int nF = prioritized_tasks_.get_nF();

    tau_opt_ =   prioritized_tasks_.get_M().bottomRows(nv-6) * x_opt_.head(nv)
              + prioritized_tasks_.get_h().tail(nv-6)
              - prioritized_tasks_.get_Jc().rightCols(nv-6).transpose() * x_opt_.segment(nv, nF);
}



template class WholeBodyController<PrioritizedTasks<ControlTasks>>;
template class WholeBodyController<MPCPrioritizedTasks>;

} // namespace wbc