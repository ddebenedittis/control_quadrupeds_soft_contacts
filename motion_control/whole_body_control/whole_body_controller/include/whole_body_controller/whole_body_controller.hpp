#pragma once

#include "whole_body_controller/prioritized_tasks.hpp"
#include "whole_body_controller/deformations_history_manager.hpp"



namespace wbc {

class WholeBodyController {
    public:
        WholeBodyController(std::string robot_name);

        void step(Eigen::VectorXd& q, Eigen::VectorXd& v, GeneralizedPose& gen_pose);

    private:
        void compute_torques(Eigen::VectorXd& x_opt, Eigen::VectorXd& tau_opt);

        PrioritizedTasks prioritized_tasks;

        DeformationsHistoryManager deformations_history_manager;
};

}