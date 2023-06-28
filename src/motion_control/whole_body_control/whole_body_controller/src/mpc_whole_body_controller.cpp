#include "whole_body_controller/mpc_whole_body_controller.hpp"
#include "whole_body_controller/whole_body_controller.hpp"



namespace wbc {

using namespace Eigen;


/* ========================= MPCWholeBodyController ========================= */

MPCWholeBodyController::MPCWholeBodyController(const std::string& robot_name, float dt, int horizon)
: WholeBodyController<MPCPrioritizedTasks>(robot_name, dt)
{
    prioritized_tasks_.set_optimization_horizon(horizon);

    const int nv = prioritized_tasks_.get_nv();
    x_opt_ = VectorXd::Zero(3*nv + 1);
}


/* ================================== Step ================================== */

void MPCWholeBodyController::step(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const std::vector<GeneralizedPose>& gen_poses)
{
    std::pair<Eigen::VectorXd, Eigen::VectorXd> defs_pair;

    if (prioritized_tasks_.get_contact_constraint_type() != ContactConstraintType::rigid) {
        deformations_history_manager_.initialize_deformations_after_planning(gen_poses[0].contact_feet_names);

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

    prioritized_tasks_.reset(q, v, gen_poses[0].contact_feet_names);

    prioritized_tasks_.compute_task_p(0, A, b, C, d, gen_poses, defs_pair.first, defs_pair.second);

    for (int i = 1; i <= prioritized_tasks_.get_max_priority(); i++) {
        we = Eigen::VectorXd::Ones(A.rows());
        wi = Eigen::VectorXd::Ones(C.rows());

        hierarchical_qp_.solve_qp(i-1, A, b, C, d, we, wi);

        prioritized_tasks_.compute_task_p(i, A, b, C, d, gen_poses, defs_pair.first, defs_pair.second);
    }

    we = Eigen::VectorXd::Ones(A.rows());
    wi = Eigen::VectorXd::Ones(C.rows());

    hierarchical_qp_.solve_qp(prioritized_tasks_.get_max_priority(), A, b, C, d, we, wi);

    x_opt_ = hierarchical_qp_.get_sol();

    const int nv = prioritized_tasks_.get_nv();
    const int nF = prioritized_tasks_.get_nF();
    const int nd = prioritized_tasks_.get_nd();

    Eigen::VectorXd f_c_opt_var = x_opt_.segment(nv+1 + nv + nv, nF);
    Eigen::VectorXd d_des_opt_var = x_opt_.segment(nv+1 + nv + nv + nF, nd);

    {
        auto generic_feet_names = get_generic_feet_names();
        std::string generic_foot_name;

        f_c_opt_.setZero();
        d_des_opt_.setZero();

        const int def_size = deformations_history_manager_.get_def_size();

        for (int i=0; i<static_cast<int>(gen_poses[0].contact_feet_names.size()); i++) {
            generic_foot_name = gen_poses[0].contact_feet_names[i];

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


/* ============================= Compute_torques ============================ */

void MPCWholeBodyController::compute_torques()
{
    const int nv = prioritized_tasks_.get_nv();
    const int nF = prioritized_tasks_.get_nF();

    tau_opt_ =   prioritized_tasks_.get_M().bottomRows(nv-6) * get_v_dot_opt()
              + prioritized_tasks_.get_h().tail(nv-6)
              - prioritized_tasks_.get_Jc().rightCols(nv-6).transpose() * x_opt_.segment(3*nv+1, nF);
}

} // namespace wbc