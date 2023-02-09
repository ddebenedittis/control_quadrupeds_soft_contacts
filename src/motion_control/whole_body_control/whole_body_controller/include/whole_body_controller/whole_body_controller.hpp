#pragma once

#include "hierarchical_optimization/hierarchical_qp.hpp"
#include "whole_body_controller/deformations_history_manager.hpp"
#include "whole_body_controller/prioritized_tasks.hpp"



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
    void step(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const GeneralizedPose& gen_pose);

    Eigen::VectorXd& get_x_opt() {return x_opt;}

    Eigen::VectorXd& get_tau_opt() {return tau_opt;}

    Eigen::Ref<Eigen::VectorXd> get_v_dot_opt() {return x_opt.segment(0, prioritized_tasks.get_nv());}

    Eigen::VectorXd& get_f_c_opt() {return f_c_opt;}

    Eigen::VectorXd& get_d_des_opt() {return d_des_opt;}

    const Eigen::VectorXd get_feet_positions() { return prioritized_tasks.get_feet_positions(); }

    const Eigen::VectorXd get_feet_velocities(const Eigen::VectorXd& v) { return prioritized_tasks.get_feet_velocities(v); }

    const std::vector<std::string>& get_generic_feet_names() const {return prioritized_tasks.get_generic_feet_names();}

    const std::vector<std::string>& get_all_feet_names() {return prioritized_tasks.get_all_feet_names();}

    int get_nv() {return prioritized_tasks.get_nv();}

    int get_def_size() {return deformations_history_manager.get_def_size();}

    void set_contact_constraint_type(const std::string& contact_constraint_type)
    {
        if (contact_constraint_type == "soft_kv") {
            prioritized_tasks.set_contact_constraint_type(ContactConstraintType::soft_kv);
            deformations_history_manager.set_def_size(3);
            d_des_opt = Eigen::VectorXd::Zero(12);
        } else if (contact_constraint_type == "rigid") {
            prioritized_tasks.set_contact_constraint_type(ContactConstraintType::rigid);
            d_des_opt = Eigen::VectorXd::Zero(0);
        } else if (contact_constraint_type == "soft_sim") {
            prioritized_tasks.set_contact_constraint_type(ContactConstraintType::soft_sim);
            deformations_history_manager.set_def_size(1);
            d_des_opt = Eigen::VectorXd::Zero(4);
        } else {
            prioritized_tasks.set_contact_constraint_type(ContactConstraintType::invalid);
        }
    }

    void set_tau_max(const double tau_max) {prioritized_tasks.set_tau_max(tau_max);}
    void set_mu(const double mu) {prioritized_tasks.set_mu(mu);}
    void set_Fn_max(const double Fn_max) {prioritized_tasks.set_Fn_max(Fn_max);}
    void set_Fn_min(const double Fn_min) {prioritized_tasks.set_Fn_min(Fn_min);}

    void set_kp_b_pos(const Eigen::Ref<const Eigen::Vector3d>& kp_b_pos) {prioritized_tasks.set_kp_b_pos(kp_b_pos);}
    void set_kd_b_pos(const Eigen::Ref<const Eigen::Vector3d>& kd_b_pos) {prioritized_tasks.set_kd_b_pos(kd_b_pos);}

    void set_kp_b_ang(const Eigen::Ref<const Eigen::Vector3d>& kp_b_ang) {prioritized_tasks.set_kp_b_ang(kp_b_ang);}
    void set_kd_b_ang(const Eigen::Ref<const Eigen::Vector3d>& kd_b_ang) {prioritized_tasks.set_kd_b_ang(kd_b_ang);}

    void set_kp_s_pos(const Eigen::Ref<const Eigen::Vector3d>& kp_s_pos) {prioritized_tasks.set_kp_s_pos(kp_s_pos);}
    void set_kd_s_pos(const Eigen::Ref<const Eigen::Vector3d>& kd_s_pos) {prioritized_tasks.set_kd_s_pos(kd_s_pos);}

    void set_kp_terr(const Eigen::Ref<const Eigen::Vector3d>& kp_terr) {
        prioritized_tasks.set_kp_terr(kp_terr);

        double kp_z = kp_terr(2);
        double def_z = prioritized_tasks.get_mass() * 9.81 / (4. * kp_z);

        Eigen::VectorXd d_k = Eigen::VectorXd::Zero(prioritized_tasks.get_all_feet_names().size() * deformations_history_manager.get_def_size());
        for (int i=0; i<static_cast<int>(prioritized_tasks.get_all_feet_names().size()); i++) {
            d_k(2 + deformations_history_manager.get_def_size() * i) = def_z;
        }

        deformations_history_manager.set_deformations_history(
            d_k, d_k,
            prioritized_tasks.get_generic_feet_names()
        );
    }
    void set_kd_terr(const Eigen::Ref<const Eigen::Vector3d>& kd_terr) {prioritized_tasks.set_kd_terr(kd_terr);}

    void set_kc_v(const Eigen::Ref<const Eigen::Vector3d>& kc_v) {prioritized_tasks.set_kc_v(kc_v);}

private:
    void compute_torques();

    PrioritizedTasks prioritized_tasks;

    DeformationsHistoryManager deformations_history_manager;

    hopt::HierarchicalQP hierarchical_qp;

    Eigen::VectorXd x_opt;      /// @brief Optimal optimization vector

    Eigen::VectorXd tau_opt;    /// @brief Optimal joint torques

    Eigen::VectorXd f_c_opt;    /// @brief Optimal contact forces

    Eigen::VectorXd d_des_opt;  /// @brief Optimal desired feet deformations

    ContactConstraintType contact_constraint_type;
};

}