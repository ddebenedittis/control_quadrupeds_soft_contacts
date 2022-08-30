#pragma once

#include "whole_body_controller/prioritized_tasks.hpp"
#include "whole_body_controller/deformations_history_manager.hpp"
#include "hierarchical_optimization/hierarchical_qp.hpp"


namespace wbc {

/// @class @brief 
class WholeBodyController {
    public:
        WholeBodyController(std::string robot_name, float dt);

        ///@brief 
        ///
        ///@param q 
        ///@param v 
        ///@param gen_pose 
        void step(Eigen::VectorXd& q, Eigen::VectorXd& v, GeneralizedPose& gen_pose);

        Eigen::VectorXd& get_x_opt() {return x_opt;}

        Eigen::VectorXd& get_tau_opt() {return tau_opt;}

        Eigen::VectorXd& get_d_des_opt() {return d_des_opt;}

        const std::vector<std::string>& get_all_feet_names() {return prioritized_tasks.get_all_feet_names();}

        int get_nv() {return prioritized_tasks.get_nv();}

        void set_kp_b_pos(Eigen::Vector3d kp_b_pos) {prioritized_tasks.set_kp_b_pos(kp_b_pos);}
        void set_kd_b_pos(Eigen::Vector3d kd_b_pos) {prioritized_tasks.set_kd_b_pos(kd_b_pos);}

        void set_kp_b_ang(Eigen::Vector3d kp_b_ang) {prioritized_tasks.set_kp_b_ang(kp_b_ang);}
        void set_kd_b_ang(Eigen::Vector3d kd_b_ang) {prioritized_tasks.set_kd_b_ang(kd_b_ang);}

        void set_kp_s_pos(Eigen::Vector3d kp_s_pos) {prioritized_tasks.set_kp_s_pos(kp_s_pos);}
        void set_kd_s_pos(Eigen::Vector3d kd_s_pos) {prioritized_tasks.set_kd_s_pos(kd_s_pos);}

        void set_Kp_terr(Eigen::Vector3d Kp_terr) {prioritized_tasks.set_Kp_terr(Kp_terr);}
        void set_Kd_terr(Eigen::Vector3d Kd_terr) {prioritized_tasks.set_Kd_terr(Kd_terr);}

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