#pragma once

#include "hierarchical_optimization/hierarchical_qp.hpp"
#include "whole_body_controller/deformations_history_manager.hpp"
#include "whole_body_controller/mpc_prioritized_tasks.hpp"
#include "whole_body_controller/whole_body_controller.hpp"



namespace wbc {

/// @class @brief 
class MPCWholeBodyController: public WholeBodyController<MPCPrioritizedTasks> {
public:
    MPCWholeBodyController(const std::string& robot_name, float dt, int horizon = 1);

    ///@brief 
    ///
    ///@param q 
    ///@param v 
    ///@param gen_pose 
    void step(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const std::vector<GeneralizedPose>& gen_poses) override;

    /// @brief Return the optimal generalized acceleration vector.
    [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> get_v_dot_opt() override
    {
        const int nv = prioritized_tasks_.get_nv();
        return x_opt_.segment(2*nv + 1, nv);
    }

private:
    void compute_torques() override;
};

}