#include "whole_body_controller/whole_body_controller.hpp"


namespace wbc {


/* ========================================================================== */
/*                       WHOLEBODYCONTROLLER CONSTRUCTOR                      */
/* ========================================================================== */

WholeBodyController::WholeBodyController(std::string robot_name, float dt)
: prioritized_tasks(robot_name, dt),
  deformations_history_manager(prioritized_tasks.get_all_feet_names()),
  hierarchical_qp(prioritized_tasks.get_max_priority()) {}


/* ========================================================================== */
/*                                    STEP                                    */
/* ========================================================================== */

void WholeBodyController::step(Eigen::VectorXd& q, Eigen::VectorXd& v, GeneralizedPose& gen_pose)
{
    deformations_history_manager.initialize_deformations_after_planning(gen_pose.contact_feet_names);

    auto defs_pair = deformations_history_manager.get_deformations_history();

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

    int nv = prioritized_tasks.get_nv();
    int nF = prioritized_tasks.get_nF();
    int nd = prioritized_tasks.get_nd();

    d_des_opt = x_opt.segment(nv + nF, nd);

    deformations_history_manager.update_deformations_after_optimization(d_des_opt);

    compute_torques();
}


/* ========================================================================== */
/*                               COMPUTE_TORQUES                              */
/* ========================================================================== */

void WholeBodyController::compute_torques()
{
    int nv = prioritized_tasks.get_nv();
    int nF = prioritized_tasks.get_nF();

    tau_opt =   prioritized_tasks.get_M().bottomRows(nv-6) * x_opt.head(nv)
              + prioritized_tasks.get_h().tail(nv-6)
              - prioritized_tasks.get_Jc().rightCols(nv-6).transpose() * x_opt.segment(nv, nF);
}

} // namespace wbc