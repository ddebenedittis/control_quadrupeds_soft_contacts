#include "whole_body_controller/whole_body_controller.hpp"



namespace wbc {


/* ========================================================================== */
/*                       WHOLEBODYCONTROLLER CONSTRUCTOR                      */
/* ========================================================================== */

WholeBodyController::WholeBodyController(const std::string& robot_name, float dt)
: prioritized_tasks(robot_name, dt),
  hierarchical_qp(prioritized_tasks.get_max_priority()),
  x_opt(Eigen::VectorXd::Zero(prioritized_tasks.get_nv())),
  tau_opt(Eigen::VectorXd::Zero(12)),
  f_c_opt(Eigen::VectorXd::Zero(12)),
  d_des_opt(Eigen::VectorXd::Zero(0))
{}


/* ========================================================================== */
/*                                    STEP                                    */
/* ========================================================================== */

void WholeBodyController::step(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const GeneralizedPose& gen_pose)
{
    std::pair<Eigen::VectorXd, Eigen::VectorXd> defs_pair;

    if (prioritized_tasks.get_contact_constraint_type() != ContactConstraintType::rigid) {
        deformations_history_manager.initialize_deformations_after_planning(gen_pose.contact_feet_names);

        defs_pair = deformations_history_manager.get_deformations_history();
    } else {
        defs_pair = std::make_pair(Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0));
    }

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::MatrixXd C;
    Eigen::VectorXd d;
    Eigen::VectorXd we;
    Eigen::VectorXd wi;

    prioritized_tasks.reset(q, v, gen_pose.contact_feet_names);

    prioritized_tasks.compute_task_p(0, A, b, C, d, gen_pose, defs_pair.first, defs_pair.second);

    for (int i = 1; i <= prioritized_tasks.get_max_priority(); i++) {
        we = Eigen::VectorXd::Ones(A.rows());
        wi = Eigen::VectorXd::Ones(C.rows());

        hierarchical_qp.solve_qp(i-1, A, b, C, d, we, wi);

        prioritized_tasks.compute_task_p(i, A, b, C, d, gen_pose, defs_pair.first, defs_pair.second);
    }

    we = Eigen::VectorXd::Ones(A.rows());
    wi = Eigen::VectorXd::Ones(C.rows());

    hierarchical_qp.solve_qp(prioritized_tasks.get_max_priority(), A, b, C, d, we, wi);

    x_opt = hierarchical_qp.get_sol();

    const int nv = prioritized_tasks.get_nv();
    const int nF = prioritized_tasks.get_nF();
    const int nd = prioritized_tasks.get_nd();

    Eigen::VectorXd f_c_opt_var = x_opt.segment(nv, nF);
    Eigen::VectorXd d_des_opt_var = x_opt.segment(nv + nF, nd);

    {
        auto generic_feet_names = get_generic_feet_names();
        std::string generic_foot_name;

        f_c_opt.setZero();
        d_des_opt.setZero();

        const int def_size = deformations_history_manager.get_def_size();

        for (int i=0; i<static_cast<int>(gen_pose.contact_feet_names.size()); i++) {
            generic_foot_name = gen_pose.contact_feet_names[i];

            auto it = std::find(generic_feet_names.begin(), generic_feet_names.end(), generic_foot_name);

            const int index = std::distance(generic_feet_names.begin(), it);

            f_c_opt.segment(3*index,3) = f_c_opt_var.segment(3*i, 3);
            
            if (prioritized_tasks.get_contact_constraint_type() != ContactConstraintType::rigid) {
                d_des_opt.segment(def_size*index, def_size) = d_des_opt_var.segment(def_size*i, def_size);
            }
        }
    }

    if (prioritized_tasks.get_contact_constraint_type() != ContactConstraintType::rigid) {
        deformations_history_manager.update_deformations_after_optimization(d_des_opt_var);
    }

    compute_torques();
}


/* ========================================================================== */
/*                               COMPUTE_TORQUES                              */
/* ========================================================================== */

void WholeBodyController::compute_torques()
{
    const int nv = prioritized_tasks.get_nv();
    const int nF = prioritized_tasks.get_nF();

    tau_opt =   prioritized_tasks.get_M().bottomRows(nv-6) * x_opt.head(nv)
              + prioritized_tasks.get_h().tail(nv-6)
              - prioritized_tasks.get_Jc().rightCols(nv-6).transpose() * x_opt.segment(nv, nF);
}

} // namespace wbc