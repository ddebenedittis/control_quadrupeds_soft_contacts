#pragma once

#include "whole_body_controller/prioritized_tasks.hpp"
#include "whole_body_controller/deformations_history_manager.hpp"
#include "hierarchical_optimization/hierarchical_qp.hpp"


namespace wbc {

/// @class @brief 
class WholeBodyController {
    public:
        WholeBodyController(std::string robot_name, float dt);

        void step(Eigen::VectorXd& q, Eigen::VectorXd& v, GeneralizedPose& gen_pose);

        Eigen::VectorXd& get_x_opt() {return x_opt;}

        Eigen::VectorXd& get_tau_opt() {return tau_opt;}

        Eigen::VectorXd& get_d_des_opt() {return d_des_opt;}

        int get_nv() {return prioritized_tasks.get_nv();}

    private:
        void compute_torques();

        PrioritizedTasks prioritized_tasks;

        DeformationsHistoryManager deformations_history_manager;

        hopt::HierarchicalQP hierarchical_qp;

        Eigen::VectorXd x_opt;

        Eigen::VectorXd tau_opt;

        Eigen::VectorXd d_des_opt;
};

}